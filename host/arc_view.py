"""
arc_view.py —— 圆弧预览与半径误差画布

上栏：轨迹（圆心坐标系）
    · 理论圆弧（蓝色，按当前参数实时预览，不需要连机）
    · 实测轨迹（橙色，运行中逐帧累积）
    · 圆心 / 起点 / 终点标记
    · 半径误差按 1000 倍放大后另画一圈虚线，肉眼可见偏差方向
下栏：半径误差曲线（µm），带 ±1 个脉冲当量的容差带

全部用 QPainter 手绘，不引入 matplotlib，和 trace_plot.py 保持一致。
"""
from typing import List, Optional, Tuple

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PySide6.QtGui import QPalette
from PySide6.QtWidgets import QWidget

from config import COLOR_GRID, COLOR_AXIS, COLOR_TEXT

C_THEORY = "#2563eb"       # 理论圆弧
C_MEAS = "#ea580c"         # 实测轨迹
C_ERR = "#7c3aed"          # 误差曲线
C_CENTER = "#0f766e"       # 圆心
C_START = "#15803d"        # 起点
C_END = "#dc2626"          # 终点
C_BAND = "#22c55e"         # 容差带（按主题加透明度使用）


class ArcView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(260, 250)
        self._geom = None                       # arc.ArcGeom
        self._pts: List[Tuple[float, float]] = []   # 实测点，圆心坐标系
        self._errs: List[float] = []            # 径向误差 mm
        self._prog: List[float] = []            # 对应进度 0~1
        self._delta = 0.003125                  # 脉冲当量，用于画参考带
        self._caption = "填好参数后这里会显示圆弧预览"
        self._show_zoom = True

    # ------------------------------------------------------------------
    #  数据入口
    # ------------------------------------------------------------------
    def set_delta(self, delta_mm: float):
        if delta_mm > 0:
            self._delta = delta_mm
            self.update()

    def set_arc(self, geom, caption: str = ""):
        """预览理论圆弧（会清掉实测数据）"""
        self._geom = geom
        self._pts, self._errs, self._prog = [], [], []
        self._caption = caption or (geom.describe(10.0, 200.0) if geom else "")
        self.update()

    def set_caption(self, caption: str):
        """只改标题文字（参数非法时用，保留上一次的图形）"""
        self._caption = caption
        self.update()

    def clear_measure(self):
        self._pts, self._errs, self._prog = [], [], []
        self.update()

    def add_point(self, relx: float, rely: float):
        """relx/rely 是相对起点的位移；内部换算到圆心坐标系"""
        if self._geom is None:
            return
        self._pts.append((relx - self._geom.i, rely - self._geom.j))
        self.update()

    def set_errors(self, errs: List[float], prog: List[float], caption: str = ""):
        self._errs = list(errs)
        self._prog = list(prog)
        if caption:
            self._caption = caption
        self.update()

    @property
    def point_count(self) -> int:
        return len(self._pts)

    # ------------------------------------------------------------------
    #  主题颜色（跟随系统深/浅色，不再硬编码白底）
    # ------------------------------------------------------------------
    def _theme(self):
        pal = self.palette()
        bg = pal.color(QPalette.Window)
        fg = pal.color(QPalette.WindowText)
        grid = QColor(fg); grid.setAlpha(34)
        axis = QColor(fg); axis.setAlpha(90)
        band = QColor(C_BAND); band.setAlpha(46)
        return bg, fg, grid, axis, band

    # ------------------------------------------------------------------
    #  绘制
    # ------------------------------------------------------------------
    def paintEvent(self, event):                    # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        bg, fg, grid_c, axis_c, band_c = self._theme()
        p.fillRect(self.rect(), bg)

        w, h = self.width(), self.height()
        f = QFont()
        f.setPointSize(8)
        p.setFont(f)
        p.setPen(fg)
        fm = p.fontMetrics()
        p.drawText(6, 14, fm.elidedText(self._caption, Qt.ElideRight, w - 12))

        top_h = int((h - 18) * 0.64)
        self._paint_path(p, 2, 18, w - 4, top_h)
        self._paint_err(p, 2, 18 + top_h + 6, w - 4, h - top_h - 30)

    # ------------------------------------------------------------------
    def _paint_path(self, p: QPainter, x0, y0, w, h):
        _, fg, grid_c, axis_c, band_c = self._theme()
        if self._geom is None:
            p.setPen(QPen(axis_c, 1, Qt.DashLine))
            p.drawRect(x0, y0, w, h)
            p.setPen(axis_c)
            p.drawText(x0 + 12, y0 + h // 2, "尚无圆弧参数")
            return

        g = self._geom
        span = g.r
        for (px, py) in self._pts:
            span = max(span, abs(px), abs(py))
        span *= 1.30
        if span <= 0:
            span = 1.0

        side = min(w, h) - 14
        if side < 20:
            return
        scale = side / (2.0 * span)
        cx = x0 + w / 2.0
        cy = y0 + h / 2.0

        def m(px, py):
            return QPointF(cx + px * scale, cy - py * scale)

        # 网格：以 R/2 为步长的一圈同心圆 + 十字轴
        p.setPen(QPen(grid_c, 1))
        for k in (0.5, 1.0):
            rr = g.r * k * scale
            p.drawEllipse(QPointF(cx, cy), rr, rr)
        p.setPen(QPen(grid_c, 1, Qt.DotLine))
        p.drawLine(QPointF(x0, cy), QPointF(x0 + w, cy))
        p.drawLine(QPointF(cx, y0), QPointF(cx, y0 + h))

        # 理论圆弧
        p.setPen(QPen(QColor(C_THEORY), 2.2))
        poly = QPolygonF()
        for (px, py) in g.path_points(240):
            poly.append(m(px, py))
        if not g.is_full_circle:
            p.drawPolyline(poly)
        else:
            p.drawPolyline(poly)          # 整圆首尾重合，polyline 已闭合

        # 圆心
        p.setPen(QPen(QColor(C_CENTER), 1.2))
        p.drawLine(QPointF(cx - 5, cy), QPointF(cx + 5, cy))
        p.drawLine(QPointF(cx, cy - 5), QPointF(cx, cy + 5))
        p.drawText(int(cx) + 6, int(cy) - 5, "C")

        # 起点 / 终点
        sx, sy = g.point_at(g.a0)
        ex, ey = g.point_at(g.a1)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(C_START))
        p.drawEllipse(m(sx, sy), 3.4, 3.4)
        p.setBrush(QColor(C_END))
        p.drawEllipse(m(ex, ey), 3.4, 3.4)
        p.setBrush(Qt.NoBrush)
        p.setPen(QColor(C_START))
        p.drawText(int(m(sx, sy).x()) + 5, int(m(sx, sy).y()) - 5, "S")
        p.setPen(QColor(C_END))
        p.drawText(int(m(ex, ey).x()) + 5, int(m(ex, ey).y()) + 13, "E")

        # 实测轨迹
        if len(self._pts) >= 2:
            p.setPen(QPen(QColor(C_MEAS), 1.6))
            mp = QPolygonF()
            for (px, py) in self._pts:
                mp.append(m(px, py))
            p.drawPolyline(mp)

        # 比例说明
        p.setPen(axis_c)
        p.drawText(int(x0 + 4), int(y0 + h - 4),
                   f"视野 ±{span:.1f} mm　R={g.r:.2f} mm")

    # ------------------------------------------------------------------
    def _paint_err(self, p: QPainter, x0, y0, w, h):
        _, fg, grid_c, axis_c, band_c = self._theme()
        if h < 40:
            return
        left = 46
        plot = QRectF(x0 + left, y0, max(w - left - 8, 10), h)

        p.setPen(QPen(axis_c, 1))
        p.drawRect(plot)

        if not self._errs:
            p.setPen(axis_c)
            p.drawText(plot.left() + 8, plot.center().y(), "运行后显示半径误差（µm）")
            return

        emax = max(abs(e) for e in self._errs)
        band = self._delta
        ymax = max(emax * 1.35, band * 2.5, 0.002)

        def ymap(v):
            return plot.center().y() - (v / ymax) * (plot.height() / 2.0)

        # 容差带 ±1 当量
        p.fillRect(QRectF(plot.left(), ymap(band), plot.width(),
                          ymap(-band) - ymap(band)), band_c)

        # 零线
        p.setPen(QPen(grid_c, 1))
        p.drawLine(QPointF(plot.left(), plot.center().y()),
                   QPointF(plot.right(), plot.center().y()))

        # 曲线
        n = len(self._errs)
        p.setPen(QPen(QColor(C_ERR), 1.6))
        poly = QPolygonF()
        for k, e in enumerate(self._errs):
            px = plot.left() + (k / max(n - 1, 1)) * plot.width()
            poly.append(QPointF(px, ymap(e)))
        p.drawPolyline(poly)

        # 刻度
        p.setPen(fg)
        p.drawText(int(x0), int(plot.top() + 9), f"+{ymax*1000:.0f}")
        p.drawText(int(x0), int(plot.center().y() + 4), "0")
        p.drawText(int(x0), int(plot.bottom()), f"-{ymax*1000:.0f}")
        fm = p.fontMetrics()
        tail = f"{n} 点  ±{band*1000:.1f} 当量"
        p.drawText(int(plot.left()), int(plot.bottom() + 12), "µm")
        p.drawText(int(plot.right() - fm.horizontalAdvance(tail) - 2),
                   int(plot.bottom() + 12), tail)
