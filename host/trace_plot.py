"""
trace_plot.py —— 运动曲线控件（F12 数据可视化）
用 QPainter 直接绘制，不依赖 matplotlib / pyqtgraph，打包和部署都简单。

上下两栏：
  上：速度—时间（v-t），标注最大值与加速段拟合结果
  下：位置—时间（x-t）
"""
from typing import List, Optional

from PySide6.QtCore import Qt, QPointF
from PySide6.QtGui import QPainter, QPen, QColor, QFont, QPolygonF
from PySide6.QtWidgets import QWidget

from PySide6.QtGui import QPalette

from config import COLOR_SPEED, COLOR_POS


class TracePlot(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(260)
        self._t: List[float] = []
        self._v: List[float] = []
        self._x: List[float] = []
        self._stop: Optional[int] = None
        self._info = "尚无数据：先点“武装记录”，再让轴运动一次，然后点“读取记录”"

    def set_data(self, t, v, x, stop_index=None, info=""):
        self._t, self._v, self._x = list(t), list(v), list(x)
        self._stop = stop_index if (stop_index is not None and stop_index >= 0) else None
        self._info = info
        self.update()

    def clear(self):
        self._t, self._v, self._x = [], [], []
        self._stop = None
        self._info = "尚无数据：先点“武装记录”，再让轴运动一次，然后点“读取记录”"
        self.update()

    # ------------------------------------------------------------------
    def _theme(self):
        """跟随系统深/浅色主题（与圆弧画布同一套规则）"""
        pal = self.palette()
        fg = pal.color(QPalette.WindowText)
        grid = QColor(fg); grid.setAlpha(34)
        axis = QColor(fg); axis.setAlpha(90)
        return pal.color(QPalette.Window), fg, grid, axis

    def paintEvent(self, event):                    # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        bg, fg, grid_c, axis_c = self._theme()
        p.fillRect(self.rect(), bg)

        w, h = self.width(), self.height()
        left, right, top, bottom = 56, 14, 26, 26
        plot_w = max(w - left - right, 10)
        gap = 18
        usable_h = max(h - top - bottom - gap, 20)
        h1 = int(usable_h * 0.55)
        h2 = usable_h - h1

        r1 = (left, top, plot_w, h1)
        r2 = (left, top + h1 + gap, plot_w, h2)

        p.setPen(fg)
        f = QFont()
        f.setPointSize(9)
        p.setFont(f)
        fm = p.fontMetrics()
        p.drawText(left, 16, fm.elidedText(self._info, Qt.ElideRight, w - left - 8))

        if not self._t:
            p.setPen(QPen(axis_c, 1, Qt.DashLine))
            p.drawRect(r1[0], r1[1], r1[2], r1[3])
            p.drawRect(r2[0], r2[1], r2[2], r2[3])
            return

        xmax = max(self._t[-1], 1.0)
        self._pane(p, r1, self._v, xmax, COLOR_SPEED, "v (mm/s)")
        self._pane(p, r2, self._x, xmax, COLOR_POS, "x (mm)")

        # 停止标记：两栏各画一条竖虚线
        if self._stop is not None and 0 <= self._stop < len(self._t):
            xs = r1[0] + (self._t[self._stop] / xmax) * r1[2]
            pen = QPen(QColor("#dc2626"), 1, Qt.DashLine)
            p.setPen(pen)
            p.drawLine(QPointF(xs, r1[1]), QPointF(xs, r1[1] + r1[3]))
            p.drawLine(QPointF(xs, r2[1]), QPointF(xs, r2[1] + r2[3]))
            p.setPen(QColor("#dc2626"))
            p.drawText(int(xs) + 4, r1[1] + 14, "STOP")

        # 时间轴
        p.setPen(fg)
        p.drawText(r2[0], r2[1] + r2[3] + 18, "t (ms)")
        p.drawText(r2[0] + r2[2] - 40, r2[1] + r2[3] + 18, f"{xmax:.0f}")

    # ------------------------------------------------------------------
    def _pane(self, p, rect, data, xmax, color, ylabel):
        left, top, w, h = rect
        _, fg, grid_c, axis_c = self._theme()

        ymax = max(data)
        if ymax <= 0:
            ymax = 1.0
        ymax *= 1.15

        # 边框与网格
        p.setPen(QPen(grid_c, 1))
        for k in range(1, 4):
            y = top + h - h * k / 4.0
            p.drawLine(QPointF(left, y), QPointF(left + w, y))
        p.setPen(QPen(axis_c, 1))
        p.drawRect(left, top, w, h)

        # 纵轴刻度
        p.setPen(fg)
        p.drawText(4, top + 10, ylabel)
        p.drawText(4, top + h // 2, f"{ymax / 1.15 / 2:.1f}")
        p.drawText(4, top + h - 3, "0")

        # 曲线
        pen = QPen(QColor(color), 2)
        p.setPen(pen)
        poly = QPolygonF()
        for i, val in enumerate(data):
            px = left + (self._t[i] / xmax) * w
            py = top + h - (val / ymax) * h
            poly.append(QPointF(px, py))
        p.drawPolyline(poly)
