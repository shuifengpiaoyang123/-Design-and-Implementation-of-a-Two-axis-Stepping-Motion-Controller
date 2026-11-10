"""
arc.py —— 圆弧几何与预估（纯 Python，不依赖 Qt / 串口，便于单元测试）

坐标约定（与固件一致，全部是"相对起点"的位移，不需要回零）：
    起点 S = (0, 0)              当前位置
    圆心 C = (I, J)              圆心相对起点的偏移 —— 与 G 代码 I/J 语义完全相同
    终点 E = (ΔX, ΔY)            终点相对起点的位移 —— 与 LINE 指令同风格

圆弧由 (I, J, ΔX, ΔY, 方向) 唯一确定；给半径/角度时由本模块反算出 I/J/ΔX/ΔY，
再交给下位机。下位机只认 I/J/ΔX/ΔY 一种形式，界面上的"半径+角度"只是输入便利。

硬件边界（与 sim/arc_sim.py 的验证结论一致）：
    · 切向速度必须由 v = ω·R 得到，不能直接用指令 F —— 否则起步瞬间跳速
    · 角度取本毫秒中点 θ + dθ/2 —— 用末端角会让轨迹持续外扩（R=10 一圈外扩 10 µm）
    · 加减速折成角加速度 α = A/R，与直线的 √(2as) 制动完全同构
"""
import math
from dataclasses import dataclass

TWO_PI = 2.0 * math.pi
DEG = math.pi / 180.0


# ======================================================================
#  几何对象
# ======================================================================
@dataclass
class ArcGeom:
    """一段圆弧的完整描述（角度均为弧度，sweep 带符号：正=逆时针）"""
    r: float = 0.0          # 半径 mm
    i: float = 0.0          # 圆心相对起点 X
    j: float = 0.0          # 圆心相对起点 Y
    dx: float = 0.0         # 终点相对起点 X
    dy: float = 0.0         # 终点相对起点 Y
    a0: float = 0.0         # 起点相对圆心的极角
    sweep: float = 0.0      # 扫角（正=逆时针）
    ccw: bool = True        # 方向

    # ------------------------------------------------------------------
    @property
    def arc_len(self) -> float:
        return abs(self.sweep) * self.r

    @property
    def chord(self) -> float:
        return math.hypot(self.dx, self.dy)

    @property
    def is_full_circle(self) -> bool:
        return abs(abs(self.sweep) - TWO_PI) < 1e-6

    @property
    def a1(self) -> float:
        return self.a0 + self.sweep

    def point_at(self, a: float):
        """圆心坐标系下的点"""
        return self.r * math.cos(a), self.r * math.sin(a)

    @property
    def start_pt(self):
        return self.point_at(self.a0)

    @property
    def end_pt(self):
        return self.point_at(self.a1)

    def path_points(self, n: int = 180):
        """理论圆弧上的 n 个点（圆心坐标系），供绘制"""
        if n < 2:
            n = 2
        return [self.point_at(self.a0 + self.sweep * k / (n - 1)) for k in range(n)]

    # ------------------------------------------------------------------
    def est_time(self, feed: float, accel: float) -> float:
        """估算用时：梯形速度规划（与固件的统一速度公式一致）"""
        s = self.arc_len
        if s <= 0 or feed <= 0:
            return 0.0
        if s >= feed * feed / max(accel, 1e-6):
            return s / feed + feed / max(accel, 1e-6)
        return 2.0 * math.sqrt(s / max(accel, 1e-6))

    def peak_feed(self, feed: float, accel: float) -> float:
        """短行程会退化成三角形，实际达不到 F，这里给界面显示真实峰值"""
        s = self.arc_len
        if s <= 0:
            return 0.0
        tri = math.sqrt(s * accel)
        return min(feed, tri)

    # ------------------------------------------------------------------
    def to_cmd_args(self):
        """给 protocol.cmd_arc 用"""
        return self.dx, self.dy, self.i, self.j

    def describe(self, feed: float, accel: float) -> str:
        vp = self.peak_feed(feed, accel)
        tail = "（整圆）" if self.is_full_circle else ""
        warn = "" if vp >= feed - 1e-6 else f"　实际峰值仅 {vp:.2f} mm/s（短弧退化）"
        return (f"R={self.r:.3f} mm　弧长 {self.arc_len:.3f} mm　扫角 "
                f"{self.sweep / DEG:.1f}°{tail}　预计 {self.est_time(feed, accel):.2f} s"
                f"{warn}")


# ======================================================================
#  构造方式一：半径 + 起始角 + 扫角（界面默认，最好用）
# ======================================================================
def from_r_angle(radius: float, start_deg: float, sweep_deg: float, ccw: bool = True) -> ArcGeom:
    """
    圆心固定在起点左/右侧，由半径与起始角决定起点位置。
        radius     半径 mm，必须 > 0
        start_deg  起点相对圆心的极角（0°=圆心的正右方，逆时针为正）
        sweep_deg  扫角大小（取绝对值，方向由 ccw 决定）
        ccw        True = G03 逆时针，False = G02 顺时针

    例：R=10, 起点 0°, 扫 360° → 从 (10,0) 出发逆时针走一整圈
    """
    r = abs(float(radius))
    if r <= 0:
        raise ValueError("半径必须大于 0")

    sw = abs(float(sweep_deg)) * DEG
    if sw <= 0:
        raise ValueError("扫角必须大于 0")
    if sw > TWO_PI + 1e-9:
        raise ValueError("扫角不能超过 360°")
    if abs(sw - TWO_PI) < 1e-9:
        sw = TWO_PI                      # 整圆：避免 359.9999 这种残差

    a0 = float(start_deg) * DEG
    dirn = 1.0 if ccw else -1.0
    sweep = dirn * sw

    # 起点在圆心坐标系 → 圆心相对起点就是它的相反数
    sx, sy = r * math.cos(a0), r * math.sin(a0)
    i, j = -sx + 0.0, -sy + 0.0          # +0.0 把 -0.0 归一成 0.0，指令里不出现 "J-0"

    a1 = a0 + sweep
    dx = r * math.cos(a1) - sx + 0.0
    dy = r * math.sin(a1) - sy + 0.0

    # 整圆时终点=起点；固件按"ΔX=ΔY=0 + 有 I/J"识别整圆
    if abs(sw - TWO_PI) < 1e-9:
        dx = dy = 0.0

    return ArcGeom(r=r, i=i, j=j, dx=dx, dy=dy, a0=a0, sweep=sweep, ccw=ccw)


# ======================================================================
#  构造方式二：直接给 I/J 与终点（高级 / 与 G 代码对齐）
# ======================================================================
def from_ij(i: float, j: float, dx: float, dy: float, ccw: bool = True) -> ArcGeom:
    """
    i, j   圆心相对起点
    dx, dy 终点相对起点；(0,0) 表示整圆

    反算扫角与半径，并检查终点是否落在圆上（误差 > 1% R 直接报错，
    避免把"打错一个数"变成一次撞机）。
    """
    r = math.hypot(i, j)
    if r < 1e-6:
        raise ValueError("I、J 不能同时为 0（圆心不能与起点重合）")

    # 起点相对圆心 = (-i, -j)
    a0 = math.atan2(-j, -i)
    full = abs(dx) < 1e-9 and abs(dy) < 1e-9

    if full:
        sweep = (1.0 if ccw else -1.0) * TWO_PI
    else:
        # 终点相对圆心
        ex, ey = dx - i, dy - j
        re = math.hypot(ex, ey)
        if abs(re - r) > 0.01 * r:
            raise ValueError(
                f"终点不在圆上：终点到圆心 {re:.3f} mm，半径 {r:.3f} mm"
                f"（偏差 {abs(re - r):.3f} mm 超过 1% R）")
        a1 = math.atan2(ey, ex)
        d = a1 - a0
        if ccw:
            while d <= 1e-9:
                d += TWO_PI
            while d > TWO_PI:
                d -= TWO_PI
            sweep = d
        else:
            while d >= -1e-9:
                d -= TWO_PI
            while d < -TWO_PI:
                d += TWO_PI
            sweep = d

    a1 = a0 + sweep
    return ArcGeom(r=r, i=i, j=j, dx=dx if not full else 0.0,
                   dy=dy if not full else 0.0, a0=a0, sweep=sweep, ccw=ccw)


# ======================================================================
#  实测数据分析
# ======================================================================
def radius_errors(points, i: float, j: float, r: float):
    """
    points: [(x, y), ...] 相对起点的实测位移
    返回每点的径向误差（mm，正=外扩）。圆心在 (i, j)。
    """
    out = []
    for x, y in points:
        out.append(math.hypot(x - i, y - j) - r)
    return out


def error_stats(errs):
    """(max_abs, mean_abs, rms) 单位 mm"""
    if not errs:
        return 0.0, 0.0, 0.0
    a = [abs(e) for e in errs]
    return max(a), sum(a) / len(a), math.sqrt(sum(e * e for e in errs) / len(errs))


def sweep_progress(points, i: float, j: float, a0: float, sweep: float):
    """
    每点相对起点的实际进度（0~1），用于横轴。
    逐点累加角度增量，避免 atan2 跨 ±π 的分支跳变。
    """
    if not points:
        return []
    dirn = 1.0 if sweep >= 0 else -1.0
    total = abs(sweep)
    prog = []
    acc = 0.0
    prev = a0
    for x, y in points:
        a = math.atan2(y - j, x - i)
        d = (a - prev) * dirn
        while d < -math.pi:
            d += TWO_PI
        while d > math.pi:
            d -= TWO_PI
        if d > 0:
            acc += d
        prev = a
        prog.append(min(acc / total, 1.0) if total > 0 else 0.0)
    return prog
