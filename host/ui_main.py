"""
ui_main.py —— 界面层（只负责显示与交互，不含协议细节）

对应任务书 4.3 的上位机功能：
  U1 串口连接     顶部工具栏：选择端口/波特率、打开关闭、连接状态
  U2 急停         红色大按钮 + 快捷键 F12 / Esc，任何状态都可用
  U3 运动操作     每轴一张卡片：连续/定长、速度、位移、正负点动、停止、回零、复位
  U4 状态显示     位置(mm,3位)、速度、运行状态、已回零、4 个限位、报警（全部来自 <ST>）
  U5 参数设置     导程、PPR(8 档)、方向取反、加速度、Vmax、软限位、回零速度；显示脉冲当量与拨码提示
  U6 运动曲线     读取 F12 记录，画 v-t / x-t，算最大速度与加速段平均加速度，导出 CSV
  U7 通信记录     带毫秒时间戳的收发原文，可隐藏状态上报、清空、保存

状态规则（4.3.2）全部由下位机上报驱动，不看“刚才点了哪个按钮”。
"""
import csv
import os
import time
from typing import List, Optional

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QFont, QKeySequence, QShortcut, QColor
from PySide6.QtWidgets import (
    QButtonGroup, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout,
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMainWindow,
    QMessageBox, QPlainTextEdit, QPushButton, QRadioButton, QSizePolicy,
    QSplitter, QStackedWidget, QTabWidget, QVBoxLayout, QWidget, QFrame,
)

import arc
import config
import protocol as proto
from arc_view import ArcView
from serial_link import SerialWorker, list_serial_ports
from trace_plot import TracePlot

CONN_IDLE = "未连接"
CONN_OK = "正常"
CONN_LOST = "通信中断"


def parse_number(text: str, lo: float, hi: float, name: str):
    """发送前检查（U5/4.3.3）：非法就返回 (None, 错误提示)"""
    s = text.strip()
    if not s:
        return None, f"{name}不能为空"
    try:
        v = float(s)
    except ValueError:
        return None, f"{name}必须是数字（当前输入：{s}）"
    if v < lo or v > hi:
        return None, f"{name}超出范围 {lo} ~ {hi}（当前输入：{v:g}）"
    return v, ""


# ======================================================================
#  单轴操作卡片
# ======================================================================
class AxisPanel(QGroupBox):
    jog = Signal(str, int, float)                 # 轴, 方向, 速度
    move = Signal(str, float, float, bool)        # 轴, 位移/坐标, 速度, 是否绝对
    stop = Signal(str)
    home = Signal(str)
    reset = Signal(str)

    def __init__(self, axis: str, title: str, parent=None):
        super().__init__(title, parent)
        self.axis = axis
        self._busy = False
        self._alarm = False
        self._homed = False
        self._connected = False

        root = QVBoxLayout(self)
        root.setSpacing(6)

        # ---- 状态显示 ----
        self.lbl_pos = QLabel("0.000")
        f = QFont()
        f.setPointSize(20)
        f.setBold(True)
        self.lbl_pos.setFont(f)
        self.lbl_pos.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        row = QHBoxLayout()
        row.addWidget(QLabel("位置 mm"))
        row.addWidget(self.lbl_pos, 1)

        self.lbl_state = QLabel("--")
        self.lbl_speed = QLabel("0.00 mm/s")
        self.lbl_homed = QLabel("未回零")
        self.lbl_lim = QLabel("限位 --")

        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.addWidget(QLabel("状态"), 0, 0)
        grid.addWidget(self.lbl_state, 0, 1)
        grid.addWidget(QLabel("速度"), 0, 2)
        grid.addWidget(self.lbl_speed, 0, 3)
        grid.addWidget(self.lbl_homed, 1, 0, 1, 2)
        grid.addWidget(self.lbl_lim, 1, 2, 1, 2)

        root.addLayout(row)
        root.addLayout(grid)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet("color:#e5e7eb")
        root.addWidget(line)

        # ---- 运动模式 ----
        self.rb_cont = QRadioButton("连续运动")
        self.rb_fixed = QRadioButton("定长运动")
        self.rb_abs = QRadioButton("绝对定位")
        self.rb_cont.setChecked(True)
        mode_row = QHBoxLayout()
        mode_row.addWidget(self.rb_cont)
        mode_row.addWidget(self.rb_fixed)
        mode_row.addWidget(self.rb_abs)
        mode_row.addStretch(1)
        root.addLayout(mode_row)

        self.ed_speed = QLineEdit("10")
        self.ed_speed.setPlaceholderText(f"0.1 ~ Vmax")
        self.ed_dist = QLineEdit("10")
        self.ed_dist.setPlaceholderText(f"{config.DIST_MIN} ~ {config.DIST_MAX}")

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignRight)
        form.addRow("速度 mm/s", self.ed_speed)
        form.addRow("位移 mm", self.ed_dist)
        root.addLayout(form)

        # ---- 按钮 ----
        b1 = QHBoxLayout()
        self.btn_neg = QPushButton("◀ 负向")
        self.btn_pos = QPushButton("正向 ▶")
        b1.addWidget(self.btn_neg)
        b1.addWidget(self.btn_pos)

        b2 = QHBoxLayout()
        self.btn_stop = QPushButton("停止")
        self.btn_home = QPushButton("回零")
        b2.addWidget(self.btn_stop)
        b2.addWidget(self.btn_home)

        b3 = QHBoxLayout()
        self.btn_reset = QPushButton("复位报警")
        b3.addWidget(self.btn_reset)

        root.addLayout(b1)
        root.addLayout(b2)
        root.addLayout(b3)

        self.lbl_hint = QLabel("")
        self.lbl_hint.setWordWrap(True)
        self.lbl_hint.setStyleSheet("color:#b45309")
        root.addWidget(self.lbl_hint)

        # ---- 连接 ----
        self.btn_pos.clicked.connect(lambda: self._on_jog(+1))
        self.btn_neg.clicked.connect(lambda: self._on_jog(-1))
        self.btn_stop.clicked.connect(lambda: self.stop.emit(self.axis))
        self.btn_home.clicked.connect(lambda: self.home.emit(self.axis))
        self.btn_reset.clicked.connect(lambda: self.reset.emit(self.axis))
        self.rb_cont.toggled.connect(self._on_mode)
        self.rb_fixed.toggled.connect(self._on_mode)
        self.rb_abs.toggled.connect(self._on_mode)
        self._on_mode()

    # ------------------------------------------------------------------
    def _on_mode(self):
        continuous = self.rb_cont.isChecked()
        self.ed_dist.setEnabled(not continuous)
        self.ed_dist.setPlaceholderText(
            "0 ~ 250（机械坐标）" if self.rb_abs.isChecked() else "0.001 ~ 250（相对位移）")

    def _on_jog(self, sign: int):
        speed, err = parse_number(self.ed_speed.text(), config.SPEED_MIN, 60.0, "速度")
        if err:
            self.lbl_hint.setText(err)
            return

        if self.rb_cont.isChecked():
            self.lbl_hint.setText("")
            self.jog.emit(self.axis, sign, speed)
            return

        dist, err = parse_number(self.ed_dist.text(), config.DIST_MIN, config.DIST_MAX,
                                 "坐标" if self.rb_abs.isChecked() else "位移")
        if err:
            self.lbl_hint.setText(err)
            return

        if self.rb_abs.isChecked() and not self._homed:
            self.lbl_hint.setText("绝对定位须先回零")
            return

        self.lbl_hint.setText("")
        # 绝对定位：正负按钮都发同一个坐标，方向由下位机按目标位置决定
        self.move.emit(self.axis, dist, speed, self.rb_abs.isChecked())

    # ------------------------------------------------------------------
    def update_status(self, st: proto.AxisStatus, limit_neg: bool, limit_pos: bool, connected: bool):
        self.lbl_pos.setText(f"{st.pos:9.3f}")
        self.lbl_state.setText(st.state)
        self.lbl_speed.setText(f"{st.speed:+.2f} mm/s")
        self.lbl_homed.setText("已回零 ✓" if st.homed else "未回零")
        self.lbl_homed.setStyleSheet("color:#15803d" if st.homed else "color:#b45309")

        neg = "●" if limit_neg else "○"
        pos = "●" if limit_pos else "○"
        self.lbl_lim.setText(f"限位 {neg}− / {pos}+")
        self.lbl_lim.setStyleSheet("color:#b91c1c" if (limit_neg or limit_pos) else "color:#6b7280")

        self._busy = st.busy
        self._homed = st.homed
        self._connected = connected
        self.refresh_enabled()

    def set_alarm(self, alarm: bool, connected: bool):
        self._alarm = alarm
        self._connected = connected
        self.refresh_enabled()

    def refresh_enabled(self):
        # 规则：未连接 / 报警 → 所有运动不可用；运动中的轴 → 该轴运动与回零不可用
        can_motion = self._connected and not self._alarm and not self._busy
        self.btn_pos.setEnabled(can_motion)
        self.btn_neg.setEnabled(can_motion)
        self.btn_home.setEnabled(can_motion)
        self.btn_stop.setEnabled(self._connected and self._busy)
        self.btn_reset.setEnabled(self._connected and self._alarm)

        # 绝对定位只在已回零时可用（4.3.2：未回零禁止绝对定位）
        self.rb_abs.setEnabled(self._homed and not self._alarm)
        if not self.rb_abs.isEnabled() and self.rb_abs.isChecked():
            self.rb_fixed.setChecked(True)

        if not self._connected:
            self.lbl_hint.setText("")
        elif self._alarm:
            self.lbl_hint.setText("报警锁定：请先复位")
        elif self._busy:
            self.lbl_hint.setText("该轴运动中")
        else:
            self.lbl_hint.setText("")


# ======================================================================
#  两轴联动面板（挑战要求②：同时启动、同时到达的斜线运动）
# ======================================================================
class LinePanel(QGroupBox):
    run_line = Signal(float, float, float)        # ΔX, ΔY, 合成进给速度
    stop_all = Signal()

    def __init__(self, parent=None):
        super().__init__("两轴联动（斜线插补）", parent)

        self._busy = False
        self._connected = False

        root = QVBoxLayout(self)
        root.setSpacing(6)

        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)

        self.ed_dx = QLineEdit("10")
        self.ed_dy = QLineEdit("10")
        self.ed_feed = QLineEdit("20")
        for ed in (self.ed_dx, self.ed_dy, self.ed_feed):
            ed.setFixedWidth(80)
            ed.setAlignment(Qt.AlignRight)

        grid.addWidget(QLabel("ΔX mm"), 0, 0)
        grid.addWidget(self.ed_dx, 0, 1)
        grid.addWidget(QLabel("ΔY mm"), 0, 2)
        grid.addWidget(self.ed_dy, 0, 3)
        grid.addWidget(QLabel("进给 F"), 0, 4)
        grid.addWidget(self.ed_feed, 0, 5)

        self.lbl_info = QLabel("—")
        self.lbl_info.setStyleSheet("color:#6b7280")
        grid.addWidget(self.lbl_info, 1, 0, 1, 6)

        root.addLayout(grid)

        row = QHBoxLayout()
        self.btn_run = QPushButton("执行斜线")
        self.btn_run.setMinimumHeight(32)
        self.btn_back = QPushButton("反向返回")
        self.btn_back.setMinimumHeight(32)
        self.btn_stop = QPushButton("停止两轴")
        self.btn_stop.setMinimumHeight(32)

        row.addWidget(self.btn_run, 2)
        row.addWidget(self.btn_back, 1)
        row.addWidget(self.btn_stop, 1)
        root.addLayout(row)

        self.lbl_hint = QLabel("")
        self.lbl_hint.setStyleSheet("color:#b91c1c")
        root.addWidget(self.lbl_hint)

        self.btn_run.clicked.connect(lambda: self._emit(1.0))
        self.btn_back.clicked.connect(lambda: self._emit(-1.0))
        self.btn_stop.clicked.connect(self.stop_all.emit)

        self.refresh_enabled()

    # ------------------------------------------------------------------
    def _emit(self, sign: float):
        dx, err = parse_number(self.ed_dx.text(), -240.0, 240.0, "ΔX")
        if err:
            self.lbl_hint.setText(err)
            return
        dy, err = parse_number(self.ed_dy.text(), -240.0, 240.0, "ΔY")
        if err:
            self.lbl_hint.setText(err)
            return
        feed, err = parse_number(self.ed_feed.text(), config.SPEED_MIN, 60.0, "进给速度")
        if err:
            self.lbl_hint.setText(err)
            return

        dx *= sign
        dy *= sign
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            self.lbl_hint.setText("ΔX 与 ΔY 不能同时为 0")
            return

        self.lbl_hint.setText("")
        self.run_line.emit(dx, dy, feed)

    # ------------------------------------------------------------------
    def update_info(self, vx: float, vy: float, busy: bool):
        """用下位机上报的两轴速度算合成速度（只做显示，不参与控制）"""
        if busy:
            v = (vx * vx + vy * vy) ** 0.5
            self.lbl_info.setText(f"轨迹速度 {v:.2f} mm/s　（vX {vx:.2f} / vY {vy:.2f}）")
        else:
            self.lbl_info.setText("待命")

    def set_state(self, connected: bool, busy: bool):
        self._connected = connected
        self._busy = busy
        self.refresh_enabled()

    def refresh_enabled(self):
        idle = self._connected and not self._busy
        self.btn_run.setEnabled(idle)
        self.btn_back.setEnabled(idle)
        self.btn_stop.setEnabled(self._connected and self._busy)


# ======================================================================
#  圆弧插补面板（项目二 挑战要求②：G02/G03 + DDA 基准脉冲）
# ======================================================================
class ArcPanel(QGroupBox):
    run_arc = Signal(object, float)          # arc.ArcGeom, 切向进给 F
    stop_all = Signal()

    def __init__(self, parent=None):
        super().__init__("圆弧插补（G02 / G03 · DDA 基准脉冲）", parent)

        self._connected = False
        self._busy = False
        self._alarm = False
        self._geom: Optional[arc.ArcGeom] = None
        self._last: Optional[arc.ArcGeom] = None
        self.get_accel = lambda: 200.0        # 由主窗口绑定到参数面板
        self.get_delta = lambda: config.pulse_equivalent(config.DEFAULT_LEAD,
                                                         config.DEFAULT_PPR)

        root = QHBoxLayout(self)
        root.setSpacing(10)

        # ---------------- 左：参数 ----------------
        left = QVBoxLayout()
        left.setSpacing(5)

        self.rb_ra = QRadioButton("半径 + 角度")
        self.rb_ij = QRadioButton("圆心 I/J + 终点")
        self.rb_ra.setChecked(True)
        # ⚠ 两组 radio 共用同一个父容器，必须显式分组，
        #   否则选"顺时针"会把"半径+角度"也互斥掉（Qt 按共同父自动成组）
        self.grp_form = QButtonGroup(self)
        self.grp_form.addButton(self.rb_ra)
        self.grp_form.addButton(self.rb_ij)
        head = QHBoxLayout()
        head.addWidget(self.rb_ra)
        head.addWidget(self.rb_ij)
        head.addStretch(1)
        left.addLayout(head)

        # 页 1：半径 / 起始角 / 扫角
        p0 = QWidget()
        h0 = QHBoxLayout(p0)
        h0.setContentsMargins(0, 0, 0, 0)
        h0.setSpacing(6)
        self.ed_r = self._mk(h0, "R mm", "10", 62)
        self.ed_a0 = self._mk(h0, "起始角°", "0", 56)
        self.ed_sw = self._mk(h0, "扫角°", "360", 56)
        h0.addStretch(1)

        # 页 2：I / J / ΔX / ΔY
        p1 = QWidget()
        h1 = QHBoxLayout(p1)
        h1.setContentsMargins(0, 0, 0, 0)
        h1.setSpacing(6)
        self.ed_i = self._mk(h1, "I", "-10", 56)
        self.ed_j = self._mk(h1, "J", "0", 56)
        self.ed_dx = self._mk(h1, "ΔX", "0", 56)
        self.ed_dy = self._mk(h1, "ΔY", "0", 56)
        h1.addStretch(1)

        self.stack = QStackedWidget()
        self.stack.addWidget(p0)
        self.stack.addWidget(p1)
        self.stack.setMaximumHeight(34)
        left.addWidget(self.stack)

        # 方向 + 进给
        line = QHBoxLayout()
        self.rb_ccw = QRadioButton("逆时针 G03")
        self.rb_cw = QRadioButton("顺时针 G02")
        self.rb_ccw.setChecked(True)
        self.grp_dir = QButtonGroup(self)
        self.grp_dir.addButton(self.rb_ccw)
        self.grp_dir.addButton(self.rb_cw)
        self.ed_feed = QLineEdit("10")
        self.ed_feed.setFixedWidth(56)
        self.ed_feed.setAlignment(Qt.AlignRight)
        line.addWidget(self.rb_ccw)
        line.addWidget(self.rb_cw)
        line.addSpacing(8)
        line.addWidget(QLabel("F"))
        line.addWidget(self.ed_feed)
        line.addWidget(QLabel("mm/s"))
        line.addStretch(1)
        left.addLayout(line)

        # 按钮
        btns = QHBoxLayout()
        self.btn_run = QPushButton("执行圆弧")
        self.btn_run.setMinimumHeight(30)
        self.btn_back = QPushButton("原路返回")
        self.btn_back.setMinimumHeight(30)
        self.btn_stop = QPushButton("停止两轴")
        self.btn_stop.setMinimumHeight(30)
        btns.addWidget(self.btn_run, 2)
        btns.addWidget(self.btn_back, 1)
        btns.addWidget(self.btn_stop, 1)
        left.addLayout(btns)

        self.lbl_info = QLabel("—")
        self.lbl_info.setWordWrap(True)
        self.lbl_info.setStyleSheet("color:#6b7280; font-size:11px")
        left.addWidget(self.lbl_info)

        self.lbl_note = QLabel("")
        self.lbl_note.setWordWrap(True)
        self.lbl_note.setStyleSheet("color:#b45309; font-size:11px")
        left.addWidget(self.lbl_note)
        left.addStretch(1)

        # ---------------- 右：预览画布 ----------------
        self.view = ArcView()
        self.view.setMinimumWidth(300)

        root.addLayout(left, 5)
        root.addWidget(self.view, 6)

        # ---------------- 信号 ----------------
        for ed in (self.ed_r, self.ed_a0, self.ed_sw, self.ed_i, self.ed_j,
                   self.ed_dx, self.ed_dy, self.ed_feed):
            ed.textChanged.connect(self._on_text)
        self.rb_ra.toggled.connect(self._on_mode)
        self.rb_ccw.toggled.connect(self._on_text)
        self.btn_run.clicked.connect(lambda: self._execute(None))
        self.btn_back.clicked.connect(self._on_back)
        self.btn_stop.clicked.connect(self.stop_all.emit)

        self._on_mode()
        self.refresh_enabled()

    # ------------------------------------------------------------------
    @staticmethod
    def _mk(layout: QHBoxLayout, label: str, text: str, width: int) -> QLineEdit:
        layout.addWidget(QLabel(label))
        ed = QLineEdit(text)
        ed.setFixedWidth(width)
        ed.setAlignment(Qt.AlignRight)
        layout.addWidget(ed)
        return ed

    def _on_mode(self):
        self.stack.setCurrentIndex(0 if self.rb_ra.isChecked() else 1)
        self._on_text()

    def _on_text(self):
        self.refresh()

    # ------------------------------------------------------------------
    def _current_geom(self) -> Optional[arc.ArcGeom]:
        """按当前输入构造几何；失败时把原因写进提示并返回 None"""
        # 进给
        feed, err = parse_number(self.ed_feed.text(), *config.ARC_FEED_RANGE, "进给 F")
        if err:
            self._fail(err)
            return None
        ccw = self.rb_ccw.isChecked()

        try:
            if self.rb_ra.isChecked():
                r, err = parse_number(self.ed_r.text(), *config.ARC_R_RANGE, "半径 R")
                if err:
                    self._fail(err)
                    return None
                a0, err = parse_number(self.ed_a0.text(), -360.0, 360.0, "起始角")
                if err:
                    self._fail(err)
                    return None
                sw, err = parse_number(self.ed_sw.text(), *config.ARC_SWEEP_RANGE, "扫角")
                if err:
                    self._fail(err)
                    return None
                return arc.from_r_angle(r, a0, sw, ccw)
            else:
                i, err = parse_number(self.ed_i.text(), -300.0, 300.0, "I")
                if err:
                    self._fail(err)
                    return None
                j, err = parse_number(self.ed_j.text(), -300.0, 300.0, "J")
                if err:
                    self._fail(err)
                    return None
                dx, err = parse_number(self.ed_dx.text(), -300.0, 300.0, "ΔX")
                if err:
                    self._fail(err)
                    return None
                dy, err = parse_number(self.ed_dy.text(), -300.0, 300.0, "ΔY")
                if err:
                    self._fail(err)
                    return None
                return arc.from_ij(i, j, dx, dy, ccw)
        except ValueError as e:
            self._fail(str(e))
            return None

    def _fail(self, msg: str):
        self._geom = None
        self.lbl_info.setText(f"✗ {msg}")
        self.lbl_info.setStyleSheet("color:#b91c1c; font-size:11px")
        self.view.set_caption(f"参数无效：{msg}")
        self.refresh_enabled()

    def refresh(self) -> bool:
        g = self._current_geom()
        if g is None:
            return False

        self._geom = g
        feed, _ = parse_number(self.ed_feed.text(), *config.ARC_FEED_RANGE, "F")
        accel = max(self.get_accel(), 1.0)

        self.lbl_info.setText(f"✓ {g.describe(feed, accel)}")
        self.lbl_info.setStyleSheet("color:#15803d; font-size:11px")
        if g.arc_len > 0 and feed > 0:
            self.view.set_delta(self.get_delta())
            self.view.set_arc(g, f"理论圆弧　R{g.r:.2f}　{g.sweep / arc.DEG:.0f}°"
                                 f"　{('逆时针 G03' if g.ccw else '顺时针 G02')}")
        self.refresh_enabled()
        return True

    def _execute(self, geom: Optional[arc.ArcGeom]):
        if geom is None:
            if not self.refresh():
                return
            geom = self._geom
        feed, err = parse_number(self.ed_feed.text(), *config.ARC_FEED_RANGE, "进给 F")
        if err:
            self.lbl_note.setText(err)
            return
        if not geom.ccw and self.rb_ccw.isChecked():
            pass                      # 用传进来的几何，不再看单选
        self.run_arc.emit(geom, feed)

    def _on_back(self):
        """原路返回：把上一段圆弧的终点当新起点，圆心搬过去，方向取反"""
        g = self._last
        if g is None:
            self.lbl_note.setText("还没有执行过圆弧")
            return
        try:
            back = arc.from_ij(g.i - g.dx, g.j - g.dy, -g.dx, -g.dy, not g.ccw)
        except ValueError as e:
            self.lbl_note.setText(f"无法原路返回：{e}")
            return
        self.lbl_note.setText("")
        self._execute(back)

    # ------------------------------------------------------------------
    def mark_sent(self, geom: arc.ArcGeom):
        self._last = geom
        self.lbl_note.setText("")
        self.refresh_enabled()

    def set_note(self, text: str):
        self.lbl_note.setText(text)

    def set_state(self, connected: bool, busy: bool, alarm: bool = False):
        self._connected = connected
        self._busy = busy
        self._alarm = alarm
        self.refresh_enabled()

    def refresh_enabled(self):
        ok = self._connected and not self._busy and not self._alarm
        self.btn_run.setEnabled(ok and self._geom is not None)
        self.btn_back.setEnabled(ok and self._last is not None)
        self.btn_stop.setEnabled(self._connected and self._busy)


# ======================================================================
#  参数面板
# ======================================================================
class ParamsPanel(QGroupBox):
    read_req = Signal(str)
    write_req = Signal(str, dict)

    def __init__(self, parent=None):
        super().__init__("参数设置（F9 / U5）", parent)

        root = QVBoxLayout(self)
        root.setSpacing(8)

        self.cb_axis = QComboBox()
        self.cb_axis.addItems(["X", "Y"])
        row = QHBoxLayout()
        row.addWidget(QLabel("轴"))
        row.addWidget(self.cb_axis, 1)
        root.addLayout(row)

        self.sp_lead = QDoubleSpinBox()
        self.sp_lead.setRange(*config.LEAD_RANGE)
        self.sp_lead.setDecimals(3)
        self.sp_lead.setSingleStep(0.5)
        self.sp_lead.setValue(config.DEFAULT_LEAD)
        self.sp_lead.setSuffix(" mm")

        self.cb_ppr = QComboBox()
        self.cb_ppr.addItems([str(p) for p in config.PPR_GEARS])
        self.cb_ppr.setCurrentText(str(config.DEFAULT_PPR))
        self.cb_ppr.setEditable(False)

        self.cb_dirinv = QCheckBox("方向取反")

        self.sp_acc = QDoubleSpinBox()
        self.sp_acc.setRange(*config.ACC_RANGE)
        self.sp_acc.setDecimals(1)
        self.sp_acc.setValue(200.0)
        self.sp_acc.setSuffix(" mm/s²")

        self.sp_vmax = QDoubleSpinBox()
        self.sp_vmax.setRange(*config.VMAX_RANGE)
        self.sp_vmax.setDecimals(2)
        self.sp_vmax.setValue(50.0)
        self.sp_vmax.setSuffix(" mm/s")

        self.sp_slmin = QDoubleSpinBox()
        self.sp_slmin.setRange(*config.SOFT_LIMIT_RANGE)
        self.sp_slmin.setDecimals(3)
        self.sp_slmin.setValue(0.0)

        self.sp_slmax = QDoubleSpinBox()
        self.sp_slmax.setRange(*config.SOFT_LIMIT_RANGE)
        self.sp_slmax.setDecimals(3)
        self.sp_slmax.setValue(240.0)

        self.sp_hf = QDoubleSpinBox()
        self.sp_hf.setRange(*config.HOME_FAST_RANGE)
        self.sp_hf.setDecimals(2)
        self.sp_hf.setValue(10.0)

        self.sp_hs = QDoubleSpinBox()
        self.sp_hs.setRange(*config.HOME_SLOW_RANGE)
        self.sp_hs.setDecimals(2)
        self.sp_hs.setValue(1.0)

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignRight)
        form.addRow("导程", self.sp_lead)
        form.addRow("每转脉冲数", self.cb_ppr)
        form.addRow("", self.cb_dirinv)
        form.addRow("加速度", self.sp_acc)
        form.addRow("最高速度", self.sp_vmax)
        form.addRow("软限位下限", self.sp_slmin)
        form.addRow("软限位上限", self.sp_slmax)
        form.addRow("回零快速", self.sp_hf)
        form.addRow("回零慢速", self.sp_hs)
        root.addLayout(form)

        self.lbl_delta = QLabel("脉冲当量：0.003125 mm/脉冲")
        self.lbl_delta.setStyleSheet("color:#0f766e; font-weight:bold")
        self.lbl_dip = QLabel("")
        self.lbl_dip.setWordWrap(True)
        self.lbl_dip.setStyleSheet("color:#6b7280; font-size:11px")
        root.addWidget(self.lbl_delta)
        root.addWidget(self.lbl_dip)

        self.btn_read = QPushButton("从下位机读取")
        self.btn_write = QPushButton("下发参数")
        row2 = QHBoxLayout()
        row2.addWidget(self.btn_read)
        row2.addWidget(self.btn_write)
        root.addLayout(row2)

        self.btn_read.clicked.connect(lambda: self.read_req.emit(self.cb_axis.currentText()))
        self.btn_write.clicked.connect(self._on_write)

        self.cb_ppr.currentTextChanged.connect(self._refresh_calc)
        self.sp_lead.valueChanged.connect(self._refresh_calc)
        self._refresh_calc()

    # ------------------------------------------------------------------
    def _refresh_calc(self):
        lead = self.sp_lead.value()
        ppr = int(self.cb_ppr.currentText())
        delta = config.pulse_equivalent(lead, ppr)
        limit = config.vmax_limit(lead, ppr)
        self.lbl_delta.setText(
            f"脉冲当量：{delta:.6f} mm/脉冲　　该细分速度上限：{limit:.1f} mm/s")
        self.lbl_dip.setText(
            f"拨码提示（{ppr} 脉冲/转）：{config.DIP_TABLE.get(ppr, '见驱动器标签')}　"
            f"— 请与驱动器面板核对")

    def _on_write(self):
        if self.sp_slmin.value() >= self.sp_slmax.value():
            QMessageBox.warning(self, "参数错误", "软限位下限必须小于上限，未下发。")
            return

        limit = config.vmax_limit(self.sp_lead.value(), int(self.cb_ppr.currentText()))
        if self.sp_vmax.value() > limit:
            QMessageBox.warning(
                self, "参数越界",
                f"该细分下最高速度不能超过 {limit:.1f} mm/s（25 kHz 输出上限），未下发。")
            return

        ret = QMessageBox.question(
            self, "确认下发参数",
            "下发后当前坐标将失效，需要重新回零。\n"
            f"同时请核对驱动器拨码：{config.DIP_TABLE.get(int(self.cb_ppr.currentText()), '见标签')}\n\n"
            "确定下发吗？",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if ret != QMessageBox.Yes:
            return

        params = dict(
            lead=self.sp_lead.value(),
            ppr=int(self.cb_ppr.currentText()),
            dirinv=1 if self.cb_dirinv.isChecked() else 0,
            acc=self.sp_acc.value(),
            vmax=self.sp_vmax.value(),
            slmin=self.sp_slmin.value(),
            slmax=self.sp_slmax.value(),
            hf=self.sp_hf.value(),
            hs=self.sp_hs.value(),
        )
        self.write_req.emit(self.cb_axis.currentText(), params)

    def apply_params(self, gp: proto.GetParams):
        self.sp_lead.setValue(gp.lead)
        self.cb_ppr.setCurrentText(str(gp.ppr))
        self.cb_dirinv.setChecked(bool(gp.dirinv))
        self.sp_acc.setValue(gp.acc)
        self.sp_vmax.setValue(gp.vmax)
        self.sp_slmin.setValue(gp.slmin)
        self.sp_slmax.setValue(gp.slmax)
        self.sp_hf.setValue(gp.hf)
        self.sp_hs.setValue(gp.hs)
        self._refresh_calc()

    def set_enabled(self, on: bool):
        self.btn_read.setEnabled(on)
        self.btn_write.setEnabled(on)


# ======================================================================
#  主窗口
# ======================================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(f"{config.APP_NAME}  v{config.APP_VERSION}")
        self.resize(1300, 920)

        self.worker: Optional[SerialWorker] = None
        self.conn_state = CONN_IDLE
        self.status = proto.StatusReport()
        self.last_status_ms = 0.0
        self.last_err = ""

        self._pending: List[dict] = []      # 等待应答的指令
        self._dump_lines: List[int] = []
        self._dump_header: Optional[proto.TraceHeader] = None
        self._dumping = False
        self._trace_axis = "X"
        self._arc_cap: Optional[dict] = None    # 圆弧实测采样上下文

        self._build_ui()
        self._setup_timers()
        self._setup_shortcuts()
        self.refresh_port_list()
        self._apply_state()

    # ------------------------------------------------------------------
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 8, 10, 8)
        root.setSpacing(8)

        # ===== 顶部：连接 + 急停 =====
        top = QHBoxLayout()
        top.addWidget(QLabel("串口"))
        self.cb_port = QComboBox()
        self.cb_port.setMinimumWidth(220)
        top.addWidget(self.cb_port)

        self.btn_refresh = QPushButton("刷新")
        self.btn_refresh.setFixedWidth(56)
        top.addWidget(self.btn_refresh)

        top.addWidget(QLabel("波特率"))
        self.cb_baud = QComboBox()
        self.cb_baud.addItems(config.BAUD_RATES)
        self.cb_baud.setCurrentText(config.DEFAULT_BAUD)
        top.addWidget(self.cb_baud)

        self.btn_connect = QPushButton("连接")
        self.btn_connect.setFixedWidth(90)
        top.addWidget(self.btn_connect)

        self.lbl_conn = QLabel(CONN_IDLE)
        self.lbl_conn.setAlignment(Qt.AlignCenter)
        self.lbl_conn.setFixedWidth(96)
        top.addWidget(self.lbl_conn)

        top.addStretch(1)

        self.btn_estop = QPushButton("急停  (F12)")
        self.btn_estop.setMinimumHeight(52)
        self.btn_estop.setMinimumWidth(180)
        self.btn_estop.setStyleSheet(
            "QPushButton{background:#dc2626;color:white;font-size:18px;font-weight:bold;"
            "border-radius:6px;} QPushButton:hover{background:#b91c1c;}"
            "QPushButton:disabled{background:#fca5a5;}")
        top.addWidget(self.btn_estop)

        root.addLayout(top)

        # ===== 中部：参数 | 轴卡片 + 曲线/记录 =====
        splitter = QSplitter(Qt.Horizontal)

        self.params = ParamsPanel()
        self.params.setMaximumWidth(360)
        splitter.addWidget(self.params)

        right = QWidget()
        rv = QVBoxLayout(right)
        rv.setContentsMargins(0, 0, 0, 0)
        rv.setSpacing(8)

        axes = QHBoxLayout()
        self.axis_x = AxisPanel("X", "X 轴")
        self.axis_y = AxisPanel("Y", "Y 轴")
        axes.addWidget(self.axis_x)
        axes.addWidget(self.axis_y)
        rv.addLayout(axes)

        self.line = LinePanel()
        self.arc = ArcPanel()
        # 圆弧的加速度与脉冲当量跟着参数面板走（只读引用）
        self.arc.get_accel = lambda: self.params.sp_acc.value()
        self.arc.get_delta = lambda: config.pulse_equivalent(
            self.params.sp_lead.value(), int(self.params.cb_ppr.currentText()))

        self.motion_tabs = QTabWidget()
        self.motion_tabs.addTab(self.line, "直线插补")
        self.motion_tabs.addTab(self.arc, "圆弧插补 G02/G03")
        self.motion_tabs.setMinimumHeight(290)
        self.motion_tabs.setMaximumHeight(340)
        rv.addWidget(self.motion_tabs)

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_trace_tab(), "运动曲线（F12 / U6）")
        self.tabs.addTab(self._build_log_tab(), "通信记录（U7）")
        rv.addWidget(self.tabs, 1)

        splitter.addWidget(right)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        root.addWidget(splitter, 1)

        self.statusBar().showMessage("就绪")

        # ===== 信号连接 =====
        self.btn_refresh.clicked.connect(self.refresh_port_list)
        self.btn_connect.clicked.connect(self.toggle_connect)
        self.btn_estop.clicked.connect(self.send_estop)

        for panel in (self.axis_x, self.axis_y):
            panel.jog.connect(self.on_jog)
            panel.move.connect(self.on_move)
            panel.stop.connect(lambda ax: self.send(proto.cmd_stop(ax), f"停止 {ax}"))
            panel.home.connect(lambda ax: self.send(proto.cmd_home(ax), f"回零 {ax}"))
            panel.reset.connect(self.on_reset)

        self.params.read_req.connect(self.on_read_params)
        self.params.write_req.connect(self.on_write_params)

        self.line.run_line.connect(self.on_line_move)
        self.line.stop_all.connect(lambda: self.send(proto.cmd_stop(), "停止两轴"))

        self.arc.run_arc.connect(self.on_arc_run)
        self.arc.stop_all.connect(lambda: self.send(proto.cmd_stop(), "停止两轴"))

    # ------------------------------------------------------------------
    def _build_trace_tab(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(6, 6, 6, 6)

        row = QHBoxLayout()
        self.cb_trace_axis = QComboBox()
        self.cb_trace_axis.addItems(["X", "Y"])
        row.addWidget(QLabel("记录轴"))
        row.addWidget(self.cb_trace_axis)

        self.btn_arm = QPushButton("武装记录")
        self.btn_dump = QPushButton("读取记录")
        self.btn_csv = QPushButton("导出 CSV")
        self.btn_csv.setEnabled(False)
        row.addWidget(self.btn_arm)
        row.addWidget(self.btn_dump)
        row.addWidget(self.btn_csv)
        row.addStretch(1)
        v.addLayout(row)

        self.plot = TracePlot()
        v.addWidget(self.plot, 1)

        self.lbl_trace_info = QLabel("提示：先“武装记录”→ 让该轴运动一次 → 运动结束后点“读取记录”")
        self.lbl_trace_info.setStyleSheet("color:#6b7280")
        v.addWidget(self.lbl_trace_info)

        self.btn_arm.clicked.connect(self.on_arm_trace)
        self.btn_dump.clicked.connect(self.on_dump_trace)
        self.btn_csv.clicked.connect(self.on_export_csv)
        return w

    # ------------------------------------------------------------------
    def _build_log_tab(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(6, 6, 6, 6)

        row = QHBoxLayout()
        self.chk_hide_st = QCheckBox("隐藏状态上报")
        self.chk_hide_st.setChecked(True)
        row.addWidget(self.chk_hide_st)
        row.addStretch(1)
        btn_clear = QPushButton("清空")
        btn_save = QPushButton("保存")
        btn_clear.setFixedWidth(72)
        btn_save.setFixedWidth(72)
        row.addWidget(btn_clear)
        row.addWidget(btn_save)
        v.addLayout(row)

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(5000)
        f = QFont("Consolas")
        f.setPointSize(9)
        self.log_view.setFont(f)
        v.addWidget(self.log_view, 1)

        btn_clear.clicked.connect(self.log_view.clear)
        btn_save.clicked.connect(self.on_save_log)
        return w

    # ------------------------------------------------------------------
    def _setup_timers(self):
        self.tm_heartbeat = QTimer(self)
        self.tm_heartbeat.setInterval(config.HEARTBEAT_MS)
        self.tm_heartbeat.timeout.connect(self.on_heartbeat)

        self.tm_watch = QTimer(self)
        self.tm_watch.setInterval(100)
        self.tm_watch.timeout.connect(self.on_watch)
        self.tm_watch.start()

    def _setup_shortcuts(self):
        for key in ("F12", "Esc", "Ctrl+Shift+E"):
            sc = QShortcut(QKeySequence(key), self)
            sc.activated.connect(self.send_estop)

    # ==================================================================
    #  连接管理（U1）
    # ==================================================================
    def refresh_port_list(self):
        cur = self.cb_port.currentData()
        self.cb_port.clear()
        ports = list_serial_ports()
        if not ports:
            self.cb_port.addItem("（未检测到串口）", None)
            return
        for dev, desc in ports:
            self.cb_port.addItem(f"{dev}  {desc}", dev)
        if cur:
            idx = self.cb_port.findData(cur)
            if idx >= 0:
                self.cb_port.setCurrentIndex(idx)

    def toggle_connect(self):
        if self.worker is None:
            port = self.cb_port.currentData()
            if not port:
                QMessageBox.warning(self, "无法连接", "没有可用串口，请检查 USB 线与 CH340 驱动。")
                return
            baud = int(self.cb_baud.currentText())
            self.worker = SerialWorker(port, baud)
            self.worker.line_received.connect(self.on_line)
            self.worker.opened.connect(self.on_opened)
            self.worker.closed.connect(self.on_closed)
            self.worker.failed.connect(self.on_failed)
            self.worker.start()
            self.log_tx(f"[连接 {port} @ {baud}]", internal=True)
        else:
            # 断开前先发停止指令（4.3.4 要求）
            self.log_tx("[断开前发送 STOP]", internal=True)
            self.worker.send(proto.cmd_stop())
            time.sleep(0.05)
            self.worker.stop()
            self.worker.wait(600)

    def on_opened(self):
        self.conn_state = CONN_OK
        self.last_status_ms = time.time() * 1000.0
        self.tm_heartbeat.start()
        self.log_tx("[串口已打开]", internal=True)
        self._apply_state()

    def on_closed(self):
        self.conn_state = CONN_IDLE
        self.worker = None
        self.tm_heartbeat.stop()
        self.log_tx("[串口已关闭]", internal=True)
        self._apply_state()

    def on_failed(self, msg: str):
        self.conn_state = CONN_IDLE
        self.last_err = msg
        self.log_tx(f"[错误] {msg}", internal=True)
        QMessageBox.critical(self, "串口错误", msg)
        self._apply_state()

    # ==================================================================
    #  发送与接收
    # ==================================================================
    def send(self, text: str, note: str = "", expect_ack=True):
        if self.worker is None:
            return False
        self.log_tx(text)
        self.worker.send(text)
        if expect_ack:
            self._pending.append({"cmd": text, "t": time.time() * 1000.0, "note": note})
            if len(self._pending) > 8:
                self._pending.pop(0)
        return True

    def log_tx(self, text: str, internal=False):
        ts = time.strftime("%H:%M:%S", time.localtime()) + f".{int(time.time() * 1000) % 1000:03d}"
        prefix = "--" if internal else "TX"
        self.log_view.appendPlainText(f"{ts} {prefix} {text}")

    def log_rx(self, line: str, ts_ms: float, hide_status: bool):
        if hide_status and line.startswith("<ST"):
            return
        t = time.localtime(ts_ms / 1000.0)
        stamp = time.strftime("%H:%M:%S", t) + f".{int(ts_ms) % 1000:03d}"
        self.log_view.appendPlainText(f"{stamp} RX {line}")

    def on_line(self, line: str, ts_ms: float):
        self.log_rx(line, ts_ms, self.chk_hide_st.isChecked())

        # 1) 状态上报
        st = proto.parse_status(line)
        if st is not None:
            self.status = st
            self.last_status_ms = ts_ms
            if self.conn_state != CONN_OK:
                self.conn_state = CONN_OK
                self._apply_state()
            self._refresh_status_widgets()
            return

        # 2) 记录头
        hdr = proto.parse_trace_header(line)
        if hdr is not None:
            self._dump_header = hdr
            self._dump_lines = []
            self._dumping = True
            self.lbl_trace_info.setText(f"正在接收记录：共 {hdr.count} 点…")
            return

        # 3) 记录结束
        if line.strip() == "<TREND>":
            self._dumping = False
            self._finish_trace()
            return

        # 4) DUMP 数据行
        if self._dumping:
            s = line.strip()
            if s.isdigit():
                self._dump_lines.append(int(s))
                if len(self._dump_lines) % 500 == 0:
                    self.lbl_trace_info.setText(
                        f"正在接收记录：{len(self._dump_lines)} 点…")
            return

        # 5) 应答
        kind, payload = proto.parse_reply(line)
        if kind is None:
            return

        pending = self._pending.pop(0) if self._pending else None

        if kind == "err":
            reason = proto.err_text(payload)
            self.statusBar().showMessage(f"指令失败：{reason}", 5000)
            cmd = pending["cmd"] if pending else ""
            if pending:
                self.log_view.appendPlainText(f"--     └ 失败：{reason}（{pending['cmd']}）")
            # 圆弧指令失败：停掉采样，把原因留在面板上（ERR,7 = 固件没实现 G02/G03）
            if cmd.startswith(("G02", "G03")):
                self._arc_cap = None
                self.arc.set_note(
                    "固件未识别 G02/G03 —— 请确认已烧录最新固件（HELP 里应能看到 G02 G03）"
                    if payload == 7 else f"圆弧被拒绝：{reason}")
                self.arc.refresh_enabled()
            # 运动过程中不弹对话框（4.3.4），只在状态栏/记录里提示；软限位这类用户操作错误才提示
            if payload == 4 and not self.any_busy():
                QMessageBox.information(self, "指令被拒绝", "超出软限位范围，指令未执行。")
            return

        if kind == "pong":
            return

        if kind == "ok_data" and payload and payload[0] in "XY":
            gp = proto.parse_get(payload)
            if gp:
                self.params.apply_params(gp)
                self.statusBar().showMessage(f"{gp.axis} 轴参数已读取", 3000)
            return

        if kind == "ok":
            self.statusBar().showMessage("指令成功", 1500)

    # ==================================================================
    #  定时任务：心跳 / 通信中断检测 / 应答超时
    # ==================================================================
    def on_heartbeat(self):
        if self.worker and self.conn_state == CONN_OK:
            self.worker.send(proto.cmd_ping())
            self.log_tx("PING")

    def on_watch(self):
        now = time.time() * 1000.0

        # 通信中断（UI-02：1 s 内提示）
        if self.conn_state == CONN_OK and now - self.last_status_ms > config.COMM_LOST_MS:
            self.conn_state = CONN_LOST
            self._apply_state()

        # 应答超时（4.3.4：300 ms；运动类指令不自动重发）
        while self._pending and now - self._pending[0]["t"] > config.CMD_TIMEOUT_MS:
            item = self._pending.pop(0)
            self.log_view.appendPlainText(f"--     └ 应答超时：{item['cmd']}")
            self.statusBar().showMessage(f"应答超时：{item['cmd']}", 4000)

    # ==================================================================
    #  界面状态刷新（4.3.2 的规则全部在这里落地）
    # ==================================================================
    def _apply_state(self):
        connected = self.conn_state in (CONN_OK, CONN_LOST)
        link_ok = self.conn_state == CONN_OK
        alarm = self.status.alarm != 0

        color = {CONN_IDLE: "#6b7280", CONN_OK: "#15803d", CONN_LOST: "#b91c1c"}[self.conn_state]
        self.lbl_conn.setText(self.conn_state)
        self.lbl_conn.setStyleSheet(
            f"background:{color};color:white;border-radius:4px;padding:3px 6px;font-weight:bold")

        self.btn_connect.setText("断开" if connected else "连接")
        self.cb_port.setEnabled(not connected)
        self.cb_baud.setEnabled(not connected)
        self.btn_refresh.setEnabled(not connected)

        self.btn_estop.setEnabled(connected)
        self.params.set_enabled(link_ok and not alarm and not self.any_busy())
        self.btn_arm.setEnabled(link_ok and not alarm)
        self.btn_dump.setEnabled(connected)
        self.btn_csv.setEnabled(connected)

        for panel in (self.axis_x, self.axis_y):
            panel.set_alarm(alarm, link_ok)
        self.arc.set_state(link_ok, self.any_busy(), alarm)

        if self.conn_state == CONN_LOST:
            self.statusBar().showMessage("⚠ 通信中断：500 ms 未收到状态上报，运动操作已禁用", 0)
        elif alarm:
            self.statusBar().showMessage(
                f"⚠ 报警：{config.ALARM_TEXT.get(self.status.alarm, '未知')}，请复位后重新回零", 0)
        elif self.conn_state == CONN_OK:
            self.statusBar().showMessage("已连接", 1500)
        else:
            self.statusBar().showMessage("未连接", 0)

    def any_busy(self) -> bool:
        return self.status.x.busy or self.status.y.busy

    def _refresh_status_widgets(self):
        st = self.status
        self.axis_x.update_status(st.x, st.limit_of("X", False), st.limit_of("X", True),
                                  self.conn_state == CONN_OK)
        self.axis_y.update_status(st.y, st.limit_of("Y", False), st.limit_of("Y", True),
                                  self.conn_state == CONN_OK)

        alarm = st.alarm != 0
        # 状态变化可能要重新算使能
        self.params.set_enabled(self.conn_state == CONN_OK and not alarm and not self.any_busy())
        self.axis_x.set_alarm(alarm, self.conn_state == CONN_OK)
        self.axis_y.set_alarm(alarm, self.conn_state == CONN_OK)

        # 联动面板：合成速度显示 + 使能（运动中禁用执行，停止可用）
        busy = self.any_busy()
        self.line.update_info(st.x.speed, st.y.speed, busy)
        self.line.set_state(self.conn_state == CONN_OK and not alarm, busy)

        # 圆弧面板：同一个 busy 判据；顺带做实测轨迹采样
        self.arc.set_state(self.conn_state == CONN_OK, busy, alarm)
        self._sample_arc(busy)

        if alarm:
            self.statusBar().showMessage(
                f"⚠ 报警：{config.ALARM_TEXT.get(st.alarm, '未知')}", 0)
        elif not st.x.homed or not st.y.homed:
            self.statusBar().showMessage("提示：未回零 —— 软限位与绝对定位不可用", 3000)

    # ==================================================================
    #  运动指令
    # ==================================================================
    def on_jog(self, axis: str, sign: int, speed: float):
        if self.conn_state != CONN_OK:
            return
        self.send(proto.cmd_jog(axis, sign, speed), f"{axis} 点动")

    def on_move(self, axis: str, dist: float, speed: float, absolute: bool):
        if self.conn_state != CONN_OK:
            return

        # 已回零时做软限位预判（UI-04：超限直接拒绝，不发指令）
        st = self.status.x if axis == "X" else self.status.y
        if st.homed:
            target = dist if absolute else st.pos + dist
            lo, hi = self.params.sp_slmin.value(), self.params.sp_slmax.value()
            if target < lo or target > hi:
                QMessageBox.information(
                    self, "超出软限位",
                    f"目标位置 {target:.3f} mm 超出软限位 {lo:.3f} ~ {hi:.3f} mm，指令未发送。")
                return

        if absolute:
            self.send(proto.cmd_mova(axis, dist, speed), f"{axis} 绝对定位")
        else:
            self.send(proto.cmd_mov(axis, dist, speed), f"{axis} 定长")

    def on_reset(self, axis: str):
        self.send(proto.cmd_reset(), "复位报警")

    def on_line_move(self, dx: float, dy: float, feed: float):
        """两轴联动：一条指令让两轴同时启动、同时到达（挑战要求②）

        注意：方法名不能叫 on_line —— 那是串口接收处理（本类里已有同名方法），
        Python 后定义的会覆盖先定义的，会导致状态帧被当成联动指令解析。"""
        if self.conn_state != CONN_OK:
            return

        # 报警是整机级的（固件只上报一个 ALM 字段），不在单轴对象上
        if self.status.alarm:
            QMessageBox.information(self, "报警中", "控制器处于报警状态，先复位报警再联动。")
            return

        # 软限位预判：任一轴越界就不发（回零后才判断）
        lo, hi = self.params.sp_slmin.value(), self.params.sp_slmax.value()
        for axis, delta in (("X", dx), ("Y", dy)):
            st = self.status.x if axis == "X" else self.status.y
            if st.homed:
                tgt = st.pos + delta
                if tgt < lo or tgt > hi:
                    QMessageBox.information(
                        self, "超出软限位",
                        f"{axis} 轴目标 {tgt:.3f} mm 超出软限位 {lo:.3f} ~ {hi:.3f} mm，"
                        f"联动指令未发送。")
                    return

        length = (dx * dx + dy * dy) ** 0.5
        self.send(proto.cmd_line(dx, dy, feed),
                  f"联动 ΔX{dx:+.3f} ΔY{dy:+.3f} 轨迹长 {length:.3f} mm")

    # ------------------------------------------------------------------
    #  圆弧插补（项目二 挑战要求②）
    # ------------------------------------------------------------------
    def on_arc_run(self, geom, feed: float):
        if self.conn_state != CONN_OK:
            return
        if self.status.alarm:
            QMessageBox.information(self, "报警中", "控制器处于报警状态，先复位报警再走圆弧。")
            return

        st = self.status
        # 软限位预判：用整段圆弧的包围盒（未回零时不做，避免误拦）
        if st.x.homed and st.y.homed:
            lo, hi = self.params.sp_slmin.value(), self.params.sp_slmax.value()
            sx, sy = geom.start_pt
            pts = geom.path_points(72)
            for name, base, vals in (
                    ("X", st.x.pos, [p[0] - sx for p in pts]),
                    ("Y", st.y.pos, [p[1] - sy for p in pts])):
                if base + min(vals) < lo or base + max(vals) > hi:
                    QMessageBox.information(
                        self, "超出软限位",
                        f"{name} 轴圆弧范围 {base + min(vals):.3f} ~ {base + max(vals):.3f} mm "
                        f"超出软限位 {lo:.3f} ~ {hi:.3f} mm，圆弧指令未发送。")
                    return

        # 采样上下文：以执行瞬间的两轴坐标为原点，后面全靠 <ST> 累积
        self._arc_cap = dict(x0=st.x.pos, y0=st.y.pos, geom=geom,
                             pts=[], started=False, idle=0)
        self.arc.view.set_arc(
            geom, f"运行中　R{geom.r:.2f} mm　{geom.sweep / arc.DEG:.0f}°　"
                  f"{'G03 逆时针' if geom.ccw else 'G02 顺时针'}")
        self.arc.mark_sent(geom)
        self.send(proto.cmd_arc(geom.dx, geom.dy, geom.i, geom.j, feed, geom.ccw),
                  f"圆弧 R{geom.r:.2f} 扫角 {geom.sweep / arc.DEG:.0f}° F{feed:g}")

    def _sample_arc(self, busy: bool):
        """每收到一帧 <ST> 就记一个实测点（10 Hz，够画趋势；固件日后支持 TRACE XY 可提速）"""
        c = self._arc_cap
        if c is None:
            return
        st = self.status

        if busy:
            c["started"] = True
            c["idle"] = 0
            p = (st.x.pos - c["x0"], st.y.pos - c["y0"])
            c["pts"].append(p)
            self.arc.view.add_point(p[0], p[1])
            return

        if not c["started"]:
            return                        # 指令刚下发，还没起步
        c["idle"] += 1
        if c["idle"] >= config.ARC_END_HOLD_FRAMES:
            self._finish_arc()

    def _finish_arc(self):
        c, self._arc_cap = self._arc_cap, None
        g, pts = c["geom"], list(c["pts"])

        # 圆弧结束瞬间固件会做一次 MoveRel 收尾（补 1~40 个脉冲到理论终点），
        # 那一小段只有几十毫秒，落在两个 <ST> 帧之间，所以 busy 采样抓不到。
        # 这里用最后一帧（已静止）的位置补上，否则终点误差会显示成 0.1 mm 量级。
        final = (self.status.x.pos - c["x0"], self.status.y.pos - c["y0"])
        if not pts or abs(final[0] - pts[-1][0]) > 1e-9 or abs(final[1] - pts[-1][1]) > 1e-9:
            pts.append(final)
            self.arc.view.add_point(final[0], final[1])

        if len(pts) < 3:
            self.arc.set_note("采样点太少（<3），未计算半径误差")
            self.arc.refresh_enabled()
            return

        errs = arc.radius_errors(pts, g.i, g.j, g.r)
        emax, emean, _ = arc.error_stats(errs)
        prog = arc.sweep_progress(pts, g.i, g.j, g.a0, g.sweep)

        # 终点误差：pts 是"相对起点"的位移，理论终点就是 (dx, dy)
        # ⚠ 不能用 g.end_pt（那是圆心坐标系下的点），整圆时它等于起点 (r,0)，
        #   减出来会得到 8 mm 这种荒谬值
        end_err = ((pts[-1][0] - g.dx) ** 2 + (pts[-1][1] - g.dy) ** 2) ** 0.5
        delta = max(self.arc.get_delta(), 1e-9)

        cap = (f"实测 {len(pts)} 点（<ST> 10 Hz）　半径误差 max {emax * 1000:.1f} µm"
               f"（{emax / delta:.1f} 当量）　mean {emean * 1000:.1f} µm"
               f"　终点 {end_err * 1000:.1f} µm")
        self.arc.view.set_errors(errs, prog, cap)
        self.arc.set_note("")
        self.statusBar().showMessage(
            f"圆弧完成：半径误差 max {emax * 1000:.1f} µm，终点误差 {end_err * 1000:.1f} µm", 8000)
        self.arc.refresh_enabled()

    def send_estop(self):
        if self.conn_state in (CONN_OK, CONN_LOST) and self.worker:
            self.log_tx(proto.cmd_estop())
            self.worker.send(proto.cmd_estop())
            self.statusBar().showMessage("已发送急停", 3000)

    # ==================================================================
    #  参数
    # ==================================================================
    def on_read_params(self, axis: str):
        self.send(proto.cmd_get(axis), f"读取 {axis} 参数")

    def on_write_params(self, axis: str, params: dict):
        self.send(proto.cmd_set(axis, **params), f"下发 {axis} 参数")
        # 参数下发后坐标失效：界面立即反映"需重新回零"
        st = self.status.x if axis == "X" else self.status.y
        st.homed = False
        self._refresh_status_widgets()

    # ==================================================================
    #  运动记录
    # ==================================================================
    def on_arm_trace(self):
        self._trace_axis = self.cb_trace_axis.currentText()
        self.plot.clear()
        self.btn_csv.setEnabled(False)
        self.lbl_trace_info.setText(f"已武装 {self._trace_axis} 轴：现在让该轴运动一次")
        self.send(proto.cmd_trace(self._trace_axis), "武装记录")

    def on_dump_trace(self):
        self._dump_lines = []
        self._dump_header = None
        self.send(proto.cmd_dump(), "读取记录", expect_ack=False)

    def _finish_trace(self):
        hdr = self._dump_header
        if hdr is None:
            return

        if not self._dump_lines:
            self.lbl_trace_info.setText("记录为空：可能该轴还没有运动，或记录前未武装")
            return

        axis = self._trace_axis
        delta_mm = config.pulse_equivalent(self.params.sp_lead.value(),
                                          int(self.params.cb_ppr.currentText()))

        t, v, x = proto.compute_trace(self._dump_lines, delta_mm, hdr.dt_ms)
        accel = proto.fit_acceleration(v, hdr.dt_ms)
        vmax = max(v)
        info = (f"{axis} 轴：{len(v)} 点 / {hdr.dt_ms} ms　最大速度 {vmax:.2f} mm/s　"
                f"加速段拟合 a = {accel:.1f} mm/s²　行程 {x[-1]:.3f} mm")
        self.plot.set_data(t, v, x, hdr.stop_index, info)
        self.lbl_trace_info.setText(info + ("　（含 STOP 标记）" if hdr.stop_index >= 0 else ""))
        self.btn_csv.setEnabled(True)

        self._trace_cache = (t, v, x, axis)

    def on_export_csv(self):
        cache = getattr(self, "_trace_cache", None)
        if not cache:
            return
        t, v, x, axis = cache

        path, _ = QFileDialog.getSaveFileName(
            self, "导出运动记录", os.path.expanduser("~/trace_%s.csv" % axis),
            "CSV 文件 (*.csv)")
        if not path:
            return

        try:
            with open(path, "w", newline="", encoding="utf-8-sig") as fp:
                wr = csv.writer(fp)
                wr.writerow(["t_ms", "dpulse", "speed_mm_s", "pos_mm"])
                for i in range(len(t)):
                    wr.writerow([t[i], self._dump_lines[i], f"{v[i]:.4f}", f"{x[i]:.4f}"])
            self.statusBar().showMessage(f"已导出 {path}", 5000)
        except OSError as e:
            QMessageBox.warning(self, "导出失败", str(e))

    def on_save_log(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "保存通信记录", os.path.expanduser("~/comm_log.txt"),
            "文本文件 (*.txt)")
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as fp:
                fp.write(self.log_view.toPlainText())
            self.statusBar().showMessage(f"已保存 {path}", 5000)
        except OSError as e:
            QMessageBox.warning(self, "保存失败", str(e))

    # ------------------------------------------------------------------
    def closeEvent(self, event):                 # noqa: N802
        if self.worker:
            try:
                self.worker.send(proto.cmd_stop())
                time.sleep(0.05)
                self.worker.stop()
                self.worker.wait(500)
            except Exception:                    # noqa: BLE001
                pass
        event.accept()
