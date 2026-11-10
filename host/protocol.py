"""
protocol.py —— 协议处理层（不含任何界面代码）
任务书 4.3.5：协议处理负责指令打包、应答与状态报文解析。

协议（与下位机 protocol.c 严格对应，一条指令一行）：
    PING
    JOG  X+ F20
    MOV  X-12.5 F20
    MOVA X100 F30
    STOP [X]
    ESTOP
    HOME X
    RESET
    SET  X LEAD=5 PPR=1600 DIRINV=0 ACC=200 VMAX=50 SLMIN=0 SLMAX=240 HF=10 HS=1
    GET  X
    TRACE X
    DUMP

应答：OK / ERR,<code>
上报：<ST X=12.500,25.00,RUN,H1 Y=0.000,0.00,IDLE,H0 LIM=0000 ALM=0>
记录：<TR X N=1532 DT=1 STOP=-1> ... N 行整数 ... <TREND>
"""
from dataclasses import dataclass, field
from typing import Optional, List

from config import PROTO_ERR_TEXT


# ---------------------------------------------------------------- 指令打包
def _f(v: float) -> str:
    """参数格式化：去掉多余小数，避免下位机 strtof 之外的歧义"""
    s = f"{v:.4f}".rstrip("0").rstrip(".")
    return s if s not in ("", "-", "-0") else "0"


def cmd_ping() -> str:
    return "PING"


def cmd_jog(axis: str, sign: int, speed: float) -> str:
    return f"JOG {axis}{'+' if sign > 0 else '-'} F{_f(speed)}"


def cmd_mov(axis: str, dist: float, speed: float) -> str:
    # 位移自带符号，直接写 X-12.5
    return f"MOV {axis}{_f(dist)} F{_f(speed)}"


def cmd_mova(axis: str, pos: float, speed: float) -> str:
    return f"MOVA {axis}{_f(pos)} F{_f(speed)}"


def cmd_line(dx: float, dy: float, feed: float) -> str:
    """两轴联动斜线：两轴同时启动、同时到达，feed 为沿轨迹的合成速度"""
    return f"LINE X{_f(dx)} Y{_f(dy)} F{_f(feed)}"


def cmd_arc(dx: float, dy: float, i: float, j: float, feed: float, ccw: bool = True) -> str:
    """
    圆弧插补（项目二 挑战要求②）。I/J 为圆心相对起点的偏移，ΔX/ΔY 为终点相对起点。
        G03 X10 Y0 I-5 J0 F20      逆时针（CCW）
        G02 X10 Y0 I-5 J0 F20      顺时针（CW）
    整圆：G03 X0 Y0 I-10 J0 F10 —— ΔX=ΔY=0 且有 I/J 时按整圆处理。
    """
    g = "G03" if ccw else "G02"
    return f"{g} X{_f(dx)} Y{_f(dy)} I{_f(i)} J{_f(j)} F{_f(feed)}"


def cmd_stop(axis: Optional[str] = None) -> str:
    return f"STOP {axis}" if axis else "STOP"


def cmd_estop() -> str:
    return "ESTOP"


def cmd_home(axis: str) -> str:
    return f"HOME {axis}"


def cmd_reset() -> str:
    return "RESET"


def cmd_trace(axis: str) -> str:
    return f"TRACE {axis}"


def cmd_dump() -> str:
    return "DUMP"


def cmd_get(axis: str) -> str:
    return f"GET {axis}"


def cmd_set(axis: str, **params) -> str:
    """
    只拼装传入的参数，未传入的保持下位机原值。
    params 例：lead=5, ppr=1600, dirinv=0, acc=200, vmax=50,
              slmin=0, slmax=240, hf=10, hs=1
    """
    order = ["lead", "ppr", "dirinv", "acc", "vmax", "slmin", "slmax", "hf", "hs"]
    parts = []
    for key in order:
        if key in params and params[key] is not None:
            parts.append(f"{key.upper()}={_f(float(params[key]))}")
    return f"SET {axis} " + " ".join(parts)


# ---------------------------------------------------------------- 报文解析
@dataclass
class AxisStatus:
    pos: float = 0.0
    speed: float = 0.0
    state: str = "IDLE"
    homed: bool = False

    @property
    def busy(self) -> bool:
        return self.state.upper() in ("RUN", "ACC", "DEC", "CRUISE", "HOME")


@dataclass
class StatusReport:
    x: AxisStatus = field(default_factory=AxisStatus)
    y: AxisStatus = field(default_factory=AxisStatus)
    limits: str = "0000"          # X- X+ Y- Y+
    alarm: int = 0

    def limit(self, index: int) -> bool:
        """index: 0=X- 1=X+ 2=Y- 3=Y+"""
        return index < len(self.limits) and self.limits[index] == "1"

    def limit_of(self, axis: str, positive: bool) -> bool:
        if axis.upper() == "X":
            return self.limit(1 if positive else 0)
        return self.limit(3 if positive else 2)


def parse_status(line: str) -> Optional[StatusReport]:
    """解析 <ST ...>；非法返回 None"""
    line = line.strip()
    if not (line.startswith("<ST") and line.endswith(">")):
        return None

    body = line[3:-1].strip()          # 去掉 <ST 和 >
    rep = StatusReport()
    tokens = body.split()

    for tok in tokens:
        if tok.startswith("LIM="):
            rep.limits = tok[4:8].ljust(4, "0")
        elif tok.startswith("ALM="):
            try:
                rep.alarm = int(tok[4:])
            except ValueError:
                rep.alarm = 0
        elif len(tok) > 2 and tok[0] in "XY" and tok[1] == "=":
            parts = tok[2:].split(",")
            if len(parts) < 3:
                continue
            try:
                st = AxisStatus(
                    pos=float(parts[0]),
                    speed=float(parts[1]),
                    state=parts[2],
                    homed=(len(parts) > 3 and parts[3].upper() in ("H1", "1")),
                )
            except ValueError:
                continue
            if tok[0] == "X":
                rep.x = st
            else:
                rep.y = st

    return rep


def err_text(code: int) -> str:
    return PROTO_ERR_TEXT.get(code, f"未知错误 {code}")


def parse_reply(line: str):
    """
    解析应答：
        "OK"            -> ("ok", None)
        "ERR,4"         -> ("err", 4)
        "PONG"          -> ("pong", None)
        "OK X LEAD=..." -> ("ok_data", 后面的文本)
    其它返回 (None, None)（可能是 <ST> 或数据行）
    """
    s = line.strip()
    if s == "PONG":
        return "pong", None
    if s == "OK":
        return "ok", None
    if s.startswith("OK "):
        return "ok_data", s[3:]
    if s.startswith("ERR"):
        try:
            return "err", int(s.split(",")[1])
        except (IndexError, ValueError):
            return "err", 0
    return None, None


@dataclass
class GetParams:
    axis: str = "X"
    lead: float = 5.0
    ppr: int = 1600
    dirinv: int = 0
    acc: float = 200.0
    vmax: float = 50.0
    slmin: float = 0.0
    slmax: float = 240.0
    hf: float = 10.0
    hs: float = 1.0
    delta: float = 0.0


def parse_get(text: str) -> Optional[GetParams]:
    """解析 GET 的返回：OK X LEAD=5.000 PPR=1600 ... DELTA=0.003125"""
    if not text:
        return None
    toks = text.split()
    if len(toks) < 2:
        return None

    gp = GetParams(axis=toks[0])
    for tok in toks[1:]:
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        try:
            if k == "LEAD":
                gp.lead = float(v)
            elif k == "PPR":
                gp.ppr = int(float(v))
            elif k == "DIRINV":
                gp.dirinv = int(float(v))
            elif k == "ACC":
                gp.acc = float(v)
            elif k == "VMAX":
                gp.vmax = float(v)
            elif k == "SLMIN":
                gp.slmin = float(v)
            elif k == "SLMAX":
                gp.slmax = float(v)
            elif k == "HF":
                gp.hf = float(v)
            elif k == "HS":
                gp.hs = float(v)
            elif k == "DELTA":
                gp.delta = float(v)
        except ValueError:
            continue
    return gp


@dataclass
class TraceHeader:
    axis: str = "X"
    count: int = 0
    dt_ms: int = 1
    stop_index: int = -1


def parse_trace_header(line: str) -> Optional[TraceHeader]:
    """解析 <TR X N=1532 DT=1 STOP=-1>"""
    s = line.strip()
    if not (s.startswith("<TR ") and s.endswith(">")):
        return None

    toks = s[4:-1].split()
    if not toks:
        return None

    hdr = TraceHeader(axis=toks[0])
    for tok in toks[1:]:
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        try:
            if k == "N":
                hdr.count = int(v)
            elif k == "DT":
                hdr.dt_ms = int(v)
            elif k == "STOP":
                hdr.stop_index = int(v)
        except ValueError:
            continue
    return hdr


def compute_trace(deltas: List[int], delta_mm: float, dt_ms: int = 1):
    """
    由每毫秒脉冲增量换算速度与位置：
        v = Δ脉冲 · δ / (dt/1000)
        x = 累计脉冲 · δ
    返回 (t_ms 列表, v 列表, x 列表)
    """
    t, v, x = [], [], []
    pulses = 0
    dt_s = max(dt_ms, 1) / 1000.0
    for i, d in enumerate(deltas):
        t.append(i * dt_ms)
        v.append(d * delta_mm / dt_s)
        pulses += d
        x.append(pulses * delta_mm)
    return t, v, x


def fit_acceleration(v: List[float], dt_ms: int = 1) -> float:
    """
    加速段平均加速度：取速度首次达到最大值之前的点做最小二乘线性拟合。
    （任务书指标 6：对速度—时间曲线加速段线性拟合，偏差 ≤ 5%）
    """
    if len(v) < 4:
        return 0.0

    vmax = max(v)
    if vmax <= 0:
        return 0.0

    # 找到第一次达到 98% 最大速度的点，作为加速段终点
    end = 0
    for i, val in enumerate(v):
        if val >= 0.98 * vmax:
            end = i
            break
    if end < 3:
        return 0.0

    xs = [i * dt_ms / 1000.0 for i in range(end + 1)]
    ys = v[:end + 1]
    n = len(xs)
    mx = sum(xs) / n
    my = sum(ys) / n
    num = sum((xs[i] - mx) * (ys[i] - my) for i in range(n))
    den = sum((xs[i] - mx) ** 2 for i in range(n))
    if den == 0:
        return 0.0
    return num / den
