"""
config.py —— 集中定义所有可调常量
任务书 4.3.5 明确要求：超时时间、心跳周期、参数范围等可调常量集中定义，不要散落各处。

修改这里就能调整整套上位机的行为，不需要翻界面代码。
"""

APP_NAME = "两轴步进运动控制器 上位机"
APP_VERSION = "1.0"

# ---------------- 串口 ----------------
BAUD_RATES = ["115200", "57600", "38400", "19200", "9600"]
DEFAULT_BAUD = "115200"

# ---------------- 通信时序 ----------------
CMD_TIMEOUT_MS = 300        # 指令应答超时（任务书建议 300 ms）
HEARTBEAT_MS = 200          # 心跳周期（F8 下位机超时 500 ms）
COMM_LOST_MS = 500          # 超过该时间没收到 <ST> 上报 → 判通信中断
PORT_SCAN_MS = 1000         # 串口列表刷新

# ---------------- 协议 ----------------
PROTO_ERR_TEXT = {
    1: "语法错误",
    2: "参数越界",
    3: "未回零",
    4: "超出软限位",
    5: "报警锁定中（需复位）",
    6: "该轴忙",
    7: "未知指令",
}

ALARM_TEXT = {
    0: "无报警",
    1: "硬限位触发",
    2: "急停",
    3: "回零失败",
}

# ---------------- 机械与参数范围（任务书 4.3.3）----------------
PPR_GEARS = [400, 800, 1000, 1600, 2000, 3200, 4000, 6400]
DEFAULT_PPR = 1600
DEFAULT_LEAD = 5.0

LEAD_RANGE = (1.0, 20.0)
ACC_RANGE = (10.0, 500.0)
VMAX_RANGE = (1.0, 60.0)
SPEED_MIN = 0.1
SOFT_LIMIT_RANGE = (0.0, 250.0)
HOME_FAST_RANGE = (5.0, 20.0)
HOME_SLOW_RANGE = (0.5, 2.0)
DIST_MAX = 250.0
DIST_MIN = 0.001

# 50 kHz 节拍、脉宽占 2 拍 → 输出频率上限 25 kHz，故 v_max = 25 kHz × 脉冲当量
FREQ_LIMIT_HZ = 25000.0

# 每转脉冲数 → 驱动器拨码提示（SW4~SW6）
# ⚠ 不同驱动器丝印不同，验收前请按面板标签核对；这里给的是常见 DM542/TB6600 组合
DIP_TABLE = {
    400:  "SW4=ON  SW5=ON  SW6=ON",
    800:  "SW4=OFF SW5=ON  SW6=ON",
    1000: "SW4=ON  SW5=OFF SW6=ON",
    1600: "SW4=OFF SW5=OFF SW6=ON",
    2000: "SW4=ON  SW5=ON  SW6=OFF",
    3200: "SW4=OFF SW5=ON  SW6=OFF",
    4000: "SW4=ON  SW5=OFF SW6=OFF",
    6400: "SW4=OFF SW5=OFF SW6=OFF",
}

# ---------------- 圆弧插补（项目二 挑战要求②）----------------
ARC_R_RANGE = (1.0, 120.0)          # 半径范围 mm
ARC_SWEEP_RANGE = (1.0, 360.0)      # 扫角范围 °
ARC_FEED_RANGE = (0.5, 60.0)        # 圆弧进给 mm/s（切向）
ARC_DEFAULT_R = 10.0
ARC_DEFAULT_FEED = 10.0
ARC_DEFAULT_ACC = 200.0             # 圆弧用的加速度，与参数面板默认值一致
ARC_END_HOLD_FRAMES = 4             # 连续 4 帧（≈400 ms）不忙 → 判定圆弧走完

# 界面配色（浅色主题）
COLOR_SPEED = "#2563eb"
COLOR_POS = "#ea580c"
COLOR_GRID = "#e5e7eb"
COLOR_AXIS = "#9ca3af"
COLOR_TEXT = "#374151"


def pulse_equivalent(lead: float, ppr: int) -> float:
    """脉冲当量 δ = 导程 / 每转脉冲数 (mm)"""
    return lead / float(ppr)


def vmax_limit(lead: float, ppr: int) -> float:
    """该细分下允许的最高速度（受 25 kHz 输出频率上限约束）"""
    return FREQ_LIMIT_HZ * pulse_equivalent(lead, ppr)
