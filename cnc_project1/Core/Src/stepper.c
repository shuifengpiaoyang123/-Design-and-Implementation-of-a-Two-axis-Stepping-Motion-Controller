/**
  ******************************************************************************
  * @file    stepper.c
  * @brief   两轴步进运动控制器 —— 脉冲发生层 + 运动规划层 + I/O 保护
  *
  *  分层说明：
  *    脉冲发生层（TIM2 中断，50 kHz）：累加器 += K，溢出出一个脉冲，位置 ±1
  *    运动规划层（SysTick 中断，1 ms） ：按梯形规律算当前速度，换算成 K 交给上面
  *    I/O 保护   （EXTI 中断 + 1 ms 轮询）：硬限位、急停、回零状态机
  *
  *  速度换算：
  *    δ = h / Nppr                        每脉冲对应位移
  *    f = v / δ                           脉冲频率
  *    K = f · 2^32 / ft = v · 2^32/(δ·ft)
  *    h=5 mm，Nppr=1600 → δ=0.003125 mm，K 增益 ≈ 2.749e7
  *    v=50 mm/s → K≈1.374e9 → 输出频率 16 kHz
  *
  *  加减速核心（一条式子解决三件事）：
  *    v = min( v + a·Tp , v_cmd , √(2·a·剩余距离) )
  *    ① 加速段按 a 上升
  *    ② 剩余距离不够时自动转减速 → 短行程自动变成三角形曲线
  *    ③ 减速段每毫秒用剩余距离反算速度 → 误差自校正，必然停在目标脉冲数上
  *
  *  限位开关接法（FML-V2 常闭输出，故障安全）：
  *    未遮挡：NC 导通 → 输入低电平；遮挡：NC 断开 → 上拉为高电平 → 判为触发
  *    因此外部中断设为上升沿，软件里"高电平 = 触发"
  ******************************************************************************
  */
#include "stepper.h"
#include "tim.h"
#include <math.h>
#include <stdint.h>

/* 机械参数默认值：本平台 1600 脉冲/转、导程 5 mm。
   现场改拨码后用 SET 指令改 PPR，程序会重算脉冲当量与 K 增益 */
#define PULSE_PER_REV   1600.0f
#define SCREW_LEAD_MM   5.0f

/* 8 个可选细分档位（F9 / U5），必须与驱动器 SW4~SW6 拨码一致 */
const uint16_t STEPPER_PPR_TABLE[STEPPER_PPR_GEARS] =
    {400, 800, 1000, 1600, 2000, 3200, 4000, 6400};

/* 每节拍最多累加半个量程 → 输出频率上限 25 kHz（脉宽占 2 个节拍） */
#define INCREMENT_MAX   0x7FFFFFFFUL

/* 点动在没有软限位时的兜底行程 */
#define JOG_LIMIT_MM    300.0f

typedef struct
{
    /* 引脚 */
    GPIO_TypeDef *step_port;
    uint16_t      step_pin;
    GPIO_TypeDef *dir_port;
    uint16_t      dir_pin;

    GPIO_TypeDef *lim_neg_port;      /* 负限位（兼回零开关） */
    uint16_t      lim_neg_pin;
    GPIO_TypeDef *lim_pos_port;      /* 正限位 */
    uint16_t      lim_pos_pin;

    /* 参数 */
    float acc;          /* 本次运动实际生效的加速度 mm/s^2（联动时按比例缩放） */
    float acc_cmd;      /* 用户设定的加速度 mm/s^2（SET ACC 写这里） */
    float vmax;         /* 最高速度 mm/s */
    float delta;        /* 脉冲当量 mm */
    float k_gain;       /* K = v · k_gain */
    float home_fast;    /* 回零快速 mm/s */
    float home_slow;    /* 回零慢速 mm/s */
    float sl_min;       /* 软限位下限 mm */
    float sl_max;       /* 软限位上限 mm */
    float lead;         /* 丝杠导程 mm */
    uint16_t ppr;       /* 每转脉冲数（8 档之一） */
    uint8_t  dir_inv;   /* 方向取反：1 = 反向 */

    /* 运动记录（F12） */
    volatile uint8_t  tr_on;        /* 1 = 正在记录 */
    volatile uint16_t tr_n;         /* 已记录点数 */
    volatile int32_t  tr_last;      /* 上次采样位置，脉冲 */

    /* 脉冲发生层 */
    volatile uint32_t phase;        /* 32 位相位累加器 */
    volatile uint32_t increment;    /* 当前累加增量 K */
    volatile uint8_t  pulse_active; /* 1 = 上一拍拉低，本拍要释放 */
    volatile uint16_t start_wait;   /* 方向建立等待（拍数，50 拍 = 1 ms） */

    /* 运动规划层 */
    volatile int32_t  position;     /* 指令位置，脉冲 */
    volatile int32_t  target;       /* 目标位置，脉冲 */
    volatile float    v_cmd;        /* 设定速度 mm/s */
    volatile float    v_plan;       /* 规划速度 mm/s（≥0） */
    volatile int8_t   direction;    /* +1 / -1 */
    volatile uint8_t  state;        /* StepperState */

    /* 圆弧插补（项目二）：1 = 本轴的脉冲由 Stepper_ArcTick 统一分配，
       此时不做"position == target 即到位"的判定（圆弧中途会多次换向） */
    volatile uint8_t  arc_mode;

    /* 回零与报警 */
    volatile uint8_t  home_state;   /* StepperHomeState */
    volatile uint16_t home_timer;   /* 当前阶段已用时，ms */
    volatile uint8_t  homed;        /* 1 = 已回零 */
    volatile uint8_t  alarm;        /* 1 = 报警锁定 */
    volatile uint8_t  alarm_code;

} StepperAxis;

static StepperAxis motor[STEPPER_NUM];

/* --------------------------------------------------------------------------
 *  圆弧插补状态（项目二 挑战要求②）
 *
 *  为什么单独一个结构、不复用两轴的 state/acc：
 *    · 直线的"比例分配"是常数 kx/ky，圆弧的比例每毫秒都在变（切线在转）
 *    · Stepper_MoveLine 启动时会把 acc 覆盖成 a·kx，圆弧没有固定 kx
 *    · 圆弧的"传动链"是角速度 ω，不是两轴各自的线速度
 *  所以圆弧自己管一条 ω 梯形，两轴退化成 ω 的执行器：v = ω·R 分解到 X/Y。
 *  两轴的 state 只用来表示"忙"，让 Stepper_IsBusy / 状态上报照常工作。
 * -------------------------------------------------------------------------- */
typedef struct
{
    volatile uint8_t on;        /* 1 = 圆弧进行中 */
    uint8_t  phase;             /* 复用 StepperState：ACC / CRUISE / DEC */
    float    r;                 /* 半径 mm */
    float    a0;                /* 起点相对圆心的极角 rad */
    float    theta;             /* 当前角 rad（按方向累积，不回绕） */
    float    sweep;             /* 应扫总角 rad（正数） */
    float    done;              /* 已扫角 rad（正数） */
    int8_t   dir;               /* +1 = 逆时针 G03，-1 = 顺时针 G02 */
    float    omega;             /* 当前角速度 rad/s */
    float    omega_cmd;         /* 目标角速度 rad/s = F / R */
    float    alpha;             /* 角加速度 rad/s² = A / R */
    int32_t  tx;                /* 终点脉冲 X（收尾补足用） */
    int32_t  ty;                /* 终点脉冲 Y */
} StepperArc;

static StepperArc s_arc;

/* 前置声明：Stepper_Stop 在 Stepper_Halt 之前定义，但圆弧中止要用到它 */
static void  Stepper_HaltRaw(uint8_t id);
static void  Stepper_AbortArc(void);
static float Stepper_VmaxLimit(const StepperAxis *m);
static void  Stepper_ArcTick(void);
static void  Stepper_ArcFinish(void);
static void  Stepper_ArcSetDir(StepperAxis *m, float v);

/* 圆弧用到的常量（不用 M_PI，避免 math.h 的宏在不同编译器下缺失） */
#define STEPPER_PI   3.14159265f
#define STEPPER_2PI  6.28318531f

/* 限位总开关：默认关闭，接好开关后由 Stepper_EnableLimits(1) 打开 */
static volatile uint8_t s_limits_enabled = 0;

/* 运动记录缓冲：两轴共用一个轴，3000 点 × 2 字节 = 6 KB（20 KB RAM 内） */
static uint16_t s_trace_buf[STEPPER_TRACE_MAX];
static volatile int8_t   s_trace_axis = -1;         /* -1 = 未武装 */
static volatile uint16_t s_trace_stop = 0xFFFFu;    /* 停止指令对应的采样序号 */

/* 通信超时保护（F8） */
static volatile uint32_t s_last_rx_ms = 0;
static volatile uint8_t  s_host_seen  = 0;          /* 是否已收到过上位机报文 */
static volatile uint8_t  s_comm_lost  = 0;
static volatile uint8_t  s_comm_trip  = 0;          /* 超时已处理过（一次性），避免每毫秒重复改目标 */

/* --------------------------------------------------------------------------
 *  内部工具
 * -------------------------------------------------------------------------- */
/* 脉冲当量 δ = h / Nppr，K 增益 = 2^32/(δ·ft) */
static void Stepper_Recompute(StepperAxis *m)
{
    if (m->ppr == 0)
        m->ppr = (uint16_t)PULSE_PER_REV;

    m->delta  = m->lead / (float)m->ppr;
    m->k_gain = 4294967296.0f / (m->delta * (float)STEPPER_TICK_HZ);
}

static void Stepper_ApplyDirection(StepperAxis *m, int8_t direction)
{
    uint8_t level;

    /* 共阳接法：方向端电平决定转向；dir_inv 用于现场反接时软件纠正 */
    level = (direction > 0) ? 1 : 0;
    if (m->dir_inv)
        level = (uint8_t)!level;

    /* 此刻速度必为 0，允许切换方向 */
    HAL_GPIO_WritePin(m->dir_port, m->dir_pin,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 限位开关状态：1 = 遮挡/触发（常闭断开，输入被上拉为高） */
static uint8_t Stepper_LimitNegActive(const StepperAxis *m)
{
    return (HAL_GPIO_ReadPin(m->lim_neg_port, m->lim_neg_pin) == GPIO_PIN_SET) ? 1u : 0u;
}

static uint8_t Stepper_LimitPosActive(const StepperAxis *m)
{
    return (HAL_GPIO_ReadPin(m->lim_pos_port, m->lim_pos_pin) == GPIO_PIN_SET) ? 1u : 0u;
}

/* 校验速度并换算成增量 K（v 为 0 时返回 0） */
static uint32_t Stepper_SpeedToIncrement(const StepperAxis *m, float v)
{
    float k;

    if (v <= 0.0f)
        return 0;

    k = v * m->k_gain;
    if (k > (float)INCREMENT_MAX)
        k = (float)INCREMENT_MAX;

    return (uint32_t)k;
}

/* 启动前的公共设置：方向、等待、目标、速度、状态 */
static uint8_t Stepper_Begin(StepperAxis *m, int8_t direction,
                             int32_t target, float speed)
{
    uint8_t homing;

    homing = (m->home_state != STEPPER_HOME_NONE &&
              m->home_state != STEPPER_HOME_DONE &&
              m->home_state != STEPPER_HOME_FAIL) ? 1u : 0u;

    /* 硬限位方向禁止：只允许向"离开限位"的方向运动（F5）
       回零流程例外——回零本身就是要往负限位上撞 */
    if (s_limits_enabled && !homing)
    {
        if (direction < 0 && Stepper_LimitNegActive(m))
            return 0;
        if (direction > 0 && Stepper_LimitPosActive(m))
            return 0;
    }

    __disable_irq();

    if (m->state != STEPPER_IDLE || m->alarm)   /* 再判一次，防止竞争 */
    {
        __enable_irq();
        return 0;
    }

    m->direction    = direction;
    Stepper_ApplyDirection(m, direction);

    m->increment    = 0;
    m->phase        = 0;
    m->pulse_active = 0;
    m->start_wait   = 50;             /* 50 拍 × 20 µs = 1 ms 方向建立时间 */

    m->target       = target;
    m->v_cmd        = speed;
    m->v_plan       = 0.0f;
    m->acc          = m->acc_cmd;     /* 恢复用户设定值；联动的按比例缩放由 Stepper_MoveLine 启动后覆盖 */
    m->state        = STEPPER_ACC;

    __enable_irq();
    return 1;
}

/* --------------------------------------------------------------------------
 *  初始化
 * -------------------------------------------------------------------------- */
void Stepper_Init(void)
{
    motor[STEPPER_X].step_port = X_STEP_GPIO_Port;
    motor[STEPPER_X].step_pin  = X_STEP_Pin;
    motor[STEPPER_X].dir_port  = X_DIR_GPIO_Port;
    motor[STEPPER_X].dir_pin   = X_DIR_Pin;
    motor[STEPPER_X].lim_neg_port = X_LIMIT_NEG_GPIO_Port;
    motor[STEPPER_X].lim_neg_pin  = X_LIMIT_NEG_Pin;
    motor[STEPPER_X].lim_pos_port = X_LIMIT_POS_GPIO_Port;
    motor[STEPPER_X].lim_pos_pin  = X_LIMIT_POS_Pin;

    motor[STEPPER_Y].step_port = Y_STEP_GPIO_Port;
    motor[STEPPER_Y].step_pin  = Y_STEP_Pin;
    motor[STEPPER_Y].dir_port  = Y_DIR_GPIO_Port;
    motor[STEPPER_Y].dir_pin   = Y_DIR_Pin;
    motor[STEPPER_Y].lim_neg_port = Y_LIMIT_NEG_GPIO_Port;
    motor[STEPPER_Y].lim_neg_pin  = Y_LIMIT_NEG_Pin;
    motor[STEPPER_Y].lim_pos_port = Y_LIMIT_POS_GPIO_Port;
    motor[STEPPER_Y].lim_pos_pin  = Y_LIMIT_POS_Pin;

    for (int i = 0; i < STEPPER_NUM; i++)
    {
        motor[i].phase        = 0;
        motor[i].increment    = 0;
        motor[i].pulse_active = 0;
        motor[i].start_wait   = 0;
        motor[i].direction    = 1;
        motor[i].position     = 0;
        motor[i].target       = 0;
        motor[i].v_cmd        = 0.0f;
        motor[i].v_plan       = 0.0f;
        motor[i].state        = STEPPER_IDLE;

        motor[i].home_state   = STEPPER_HOME_NONE;
        motor[i].home_timer   = 0;
        motor[i].homed        = 0;
        motor[i].alarm        = 0;
        motor[i].alarm_code   = STEPPER_ALM_NONE;

        motor[i].tr_on        = 0;
        motor[i].tr_n         = 0;
        motor[i].tr_last      = 0;

        motor[i].acc       = STEPPER_ACC_DEFAULT;
        motor[i].acc_cmd   = STEPPER_ACC_DEFAULT;
        motor[i].vmax      = STEPPER_VMAX_DEFAULT;
        motor[i].home_fast = STEPPER_HOME_FAST_DEFAULT;
        motor[i].home_slow = STEPPER_HOME_SLOW_DEFAULT;
        motor[i].sl_min    = STEPPER_SLIMIT_MIN_DEFAULT;
        motor[i].sl_max    = STEPPER_SLIMIT_MAX_DEFAULT;
        motor[i].lead      = SCREW_LEAD_MM;
        motor[i].ppr       = (uint16_t)PULSE_PER_REV;
        motor[i].dir_inv   = 0;

        Stepper_Recompute(&motor[i]);

        /* STEP 空闲为高电平（共阳，关断） */
        HAL_GPIO_WritePin(motor[i].step_port, motor[i].step_pin, GPIO_PIN_SET);
    }
}

/* --------------------------------------------------------------------------
 *  连续运动（点动）：目标设到行程尽头，由 Stepper_Stop 的减速曲线收尾
 * -------------------------------------------------------------------------- */
uint8_t Stepper_Start(uint8_t id, int8_t direction, float speed)
{
    int32_t target;
    int32_t span;

    if (id >= STEPPER_NUM)
        return 0;

    if (direction != 1 && direction != -1)
        return 0;

    if (speed < STEPPER_VMIN || speed > motor[id].vmax)
        return 0;

    span = (int32_t)(JOG_LIMIT_MM / motor[id].delta);
    if (direction > 0)
        target = motor[id].position + span;
    else
        target = motor[id].position - span;

    /* 软限位（F6）：回零后点动也不许越过行程，超出则把目标钳在边界上 */
    if (motor[id].homed)
    {
        int32_t lo = (int32_t)(motor[id].sl_min / motor[id].delta);
        int32_t hi = (int32_t)(motor[id].sl_max / motor[id].delta);

        if (direction > 0)
        {
            if (motor[id].position >= hi)
                return 0;
            if (target > hi)
                target = hi;
        }
        else
        {
            if (motor[id].position <= lo)
                return 0;
            if (target < lo)
                target = lo;
        }
    }

    return Stepper_Begin(&motor[id], direction, target, speed);
}

/* --------------------------------------------------------------------------
 *  定长运动（增量）：N = round(L/δ)
 * -------------------------------------------------------------------------- */
uint8_t Stepper_MoveRel(uint8_t id, float distance_mm, float speed)
{
    StepperAxis *m;
    float    nd;
    int32_t  n;
    int32_t  target;
    float    target_mm;

    if (id >= STEPPER_NUM)
        return 0;

    m = &motor[id];

    if (speed < STEPPER_VMIN || speed > m->vmax)
        return 0;

    if (m->state != STEPPER_IDLE)
        return 0;

    /* 位移换算成脉冲数，四舍五入 */
    nd = distance_mm / m->delta;
    n  = (int32_t)(nd + ((nd >= 0.0f) ? 0.5f : -0.5f));

    if (n == 0)
        return 0;

    target = m->position + n;

    /* 软限位保护（F6）：回零后才生效 */
    if (m->homed)
    {
        target_mm = (float)target * m->delta;
        if (target_mm < m->sl_min || target_mm > m->sl_max)
            return 0;
    }

    return Stepper_Begin(m, (n > 0) ? 1 : -1, target, speed);
}

/* --------------------------------------------------------------------------
 *  绝对定位（提高要求）：目标为机械坐标 mm，必须已回零
 * -------------------------------------------------------------------------- */
uint8_t Stepper_MoveAbs(uint8_t id, float pos_mm, float speed)
{
    StepperAxis *m;
    int32_t target;
    float   posd;
    int32_t n;

    if (id >= STEPPER_NUM)
        return 0;

    m = &motor[id];

    if (!m->homed || m->state != STEPPER_IDLE)
        return 0;

    if (speed < STEPPER_VMIN || speed > m->vmax)
        return 0;

    if (pos_mm < m->sl_min || pos_mm > m->sl_max)
        return 0;

    posd = pos_mm / m->delta;
    n = (int32_t)(posd + ((posd >= 0.0f) ? 0.5f : -0.5f));

    if (n == m->position)
        return 0;

    target = n;

    return Stepper_Begin(m, (target > m->position) ? 1 : -1, target, speed);
}

/* --------------------------------------------------------------------------
 *  两轴联动（挑战要求②）：同时启动、同时到达的斜线运动
 *
 *  做法：把进给速度 F 按位移比例分配到两轴
 *      k_x = |dx| / L,  k_y = |dy| / L,  L = √(dx² + dy²)
 *      v_x = F·k_x,     v_y = F·k_y
 *      a_x = A·k_x,     a_y = A·k_y     （A 取各轴 acc_cmd/k 的最小值，任何一轴都不超设定）
 *
 *  因为两轴的速度曲线形状完全一样（时间轴相同），任意时刻都有 v_x/v_y ≡ |dx|/|dy|，
 *  所以轨迹是一条严格的直线；加速时间 = F/A 相同、匀速时间 = 剩余/F 相同 → 同时到达。
 *
 *  这也是项目二 DDA 插补的最简形式：不用专门的插补器，
 *  只靠"按比例缩放每轴的速度和加速度"，就能在现有四层架构里实现联动。
 * -------------------------------------------------------------------------- */
uint8_t Stepper_MoveLine(float dx, float dy, float feed)
{
    StepperAxis *mx = &motor[STEPPER_X];
    StepperAxis *my = &motor[STEPPER_Y];
    float   lx, ly, len, kx, ky, v, a, t, vx, vy;
    int32_t nx, ny, tx, ty;

    if (mx->state != STEPPER_IDLE || my->state != STEPPER_IDLE)
        return 0;

    if (mx->alarm || my->alarm)
        return 0;

    if (feed < STEPPER_VMIN || feed > 200.0f)
        return 0;

    /* 位移换算成整数脉冲，再用整数脉冲反算实际位移：
       保证"比例"与"最终到达的脉冲数"严格一致，不会因为是浮点四舍五入而某一轴先到 */
    lx = dx / mx->delta;
    nx = (int32_t)(lx + ((lx >= 0.0f) ? 0.5f : -0.5f));
    ly = dy / my->delta;
    ny = (int32_t)(ly + ((ly >= 0.0f) ? 0.5f : -0.5f));

    if (nx == 0 && ny == 0)
        return 0;

    lx = fabsf((float)nx * mx->delta);
    ly = fabsf((float)ny * my->delta);
    len = sqrtf(lx * lx + ly * ly);
    if (len < 0.001f)
        return 0;

    kx = lx / len;
    ky = ly / len;

    /* 软限位预判（回零后才生效）：任一轴越界，整条斜线都不执行 */
    if (mx->homed)
    {
        float mm = (float)(mx->position + nx) * mx->delta;
        if (mm < mx->sl_min || mm > mx->sl_max)
            return 0;
    }
    if (my->homed)
    {
        float mm = (float)(my->position + ny) * my->delta;
        if (mm < my->sl_min || mm > my->sl_max)
            return 0;
    }

    /* 进给速度按比例分配；任一轴超自身 Vmax 时整体降速（比例不能变，否则轨迹会拐弯） */
    v = feed;
    if (kx > 0.0f && v * kx > mx->vmax)
        v = mx->vmax / kx;
    if (ky > 0.0f && v * ky > my->vmax)
        v = my->vmax / ky;

    vx = v * kx;
    vy = v * ky;

    /* 速度低于下限时无法保证脉冲均匀，直接拒绝，避免"看起来动了其实在爬" */
    if ((nx != 0 && vx < STEPPER_VMIN) || (ny != 0 && vy < STEPPER_VMIN))
        return 0;

    /* 公共加速度 A：保证每轴实际加速度 A·k 不超过该轴设定值 */
    a = 1.0e9f;
    if (kx > 0.0f) { t = mx->acc_cmd / kx; if (t < a) a = t; }
    if (ky > 0.0f) { t = my->acc_cmd / ky; if (t < a) a = t; }

    tx = mx->position + nx;
    ty = my->position + ny;

    /* 两轴依次启动：start_wait 都置 50 拍，方向建立时间相同，
       所以两轴的第一个脉冲落在同一个 50 kHz 节拍上 → 真正同时启动 */
    if (nx != 0 && !Stepper_Begin(mx, (nx > 0) ? 1 : -1, tx, vx))
        return 0;

    if (ny != 0 && !Stepper_Begin(my, (ny > 0) ? 1 : -1, ty, vy))
    {
        Stepper_Halt(STEPPER_X);          /* 第二轴启动失败，撤掉第一轴，不留下半条斜线 */
        return 0;
    }

    /* Stepper_Begin 里已把 acc 恢复成 acc_cmd，这里覆盖成联动缩放值 */
    if (nx != 0) mx->acc = a * kx;
    if (ny != 0) my->acc = a * ky;

    return 1;
}

/* --------------------------------------------------------------------------
 *  圆弧插补 G02 / G03（项目二 挑战要求②，DDA 基准脉冲）
 *
 *  坐标约定（全部相对起点，不需要回零）：
 *      起点 S = 0，圆心 C = (i, j)  →  r = |C|，起点极角 a0 = atan2(-j, -i)
 *      dx/dy 为终点相对位移；(0,0) 且有 I/J = 整圆
 *
 *  每 1 ms 的算式（与直线的梯形完全同构，只是 a→α、s→剩余角）：
 *      ω  = min( ω + α·T , ω_cmd , √(2·α·剩余角) )
 *      dθ = ω·T，θ += dir·dθ，done += dθ
 *      v  = ω·r                    ← 必须用 ω·r，不能用指令 F（否则起步跳速）
 *      vx = -dir·sin(θ_mid)·v      ← θ_mid 取本毫秒中点，用末端角会持续外扩
 *      vy =  dir·cos(θ_mid)·v
 *
 *  三个必须照做的细节（仿真验证过，见 docs/圆弧插补仿真验证.md）：
 *    ① v 用 ω·r：起步时 ω=0 → v=0，切向速度从 0 平滑升到 F，不会瞬间满速
 *    ② 方向角取中点：R=10 走一圈，用末端角会外扩约 10 µm，用中点角降到 7 µm（量化噪声级）
 *    ③ 换向只在"该轴几乎不出脉冲"时做：整圆正好落在 θ=90°/270° 的正弦过零点
 * -------------------------------------------------------------------------- */
uint8_t Stepper_MoveArc(float dx, float dy, float i, float j,
                        float feed, uint8_t ccw)
{
    StepperAxis *mx = &motor[STEPPER_X];
    StepperAxis *my = &motor[STEPPER_Y];
    float   r, a0, a1, sweep, d, px, py, q, vmax, vlim, a_lin;
    float   cxm, cym, xm, ym, aa, vx0, vy0;
    int32_t nx, ny;
    int8_t  dirn, dx0, dy0;
    uint8_t k;

    if (s_arc.on)
        return 0;

    if (mx->state != STEPPER_IDLE || my->state != STEPPER_IDLE)
        return 0;

    if (mx->alarm || my->alarm)
        return 0;

    /* 回零流程中不接受圆弧 */
    if ((mx->home_state != STEPPER_HOME_NONE && mx->home_state != STEPPER_HOME_DONE &&
         mx->home_state != STEPPER_HOME_FAIL) ||
        (my->home_state != STEPPER_HOME_NONE && my->home_state != STEPPER_HOME_DONE &&
         my->home_state != STEPPER_HOME_FAIL))
        return 0;

    r = sqrtf(i * i + j * j);
    if (r < 1.0f || r > 200.0f)          /* 半径太小无意义，太大超出行程 */
        return 0;

    if (feed < STEPPER_VMIN)
        return 0;

    /* 切向速度上限：|vx|、|vy| 都 ≤ ω·r，所以 F ≤ 两轴 Vmax 与 25 kHz 上限的最小值 */
    vmax = mx->vmax;
    if (my->vmax < vmax) vmax = my->vmax;
    vlim = Stepper_VmaxLimit(mx);
    if (vlim < vmax) vmax = vlim;
    vlim = Stepper_VmaxLimit(my);
    if (vlim < vmax) vmax = vlim;
    if (feed > vmax) feed = vmax;
    if (feed < STEPPER_VMIN) return 0;

    /* 起点极角：起点相对圆心 = (-i, -j) */
    a0 = atan2f(-j, -i);

    if (fabsf(dx) < 0.0005f && fabsf(dy) < 0.0005f)
    {
        sweep = STEPPER_2PI;                       /* 整圆 */
    }
    else
    {
        px = dx - i;                               /* 终点相对圆心 */
        py = dy - j;
        q  = sqrtf(px * px + py * py);
        if (q < 0.5f * r)                          /* 终点落在圆心附近，非法 */
            return 0;
        if (fabsf(q - r) > 0.02f * r + 0.02f)      /* 终点不在圆上（2% R + 0.02 mm 容差） */
            return 0;

        a1 = atan2f(py, px);
        d  = a1 - a0;
        if (ccw)
        {
            while (d <= 1.0e-5f) d += STEPPER_2PI;
            while (d > STEPPER_2PI) d -= STEPPER_2PI;
        }
        else
        {
            while (d >= -1.0e-5f) d -= STEPPER_2PI;
            while (d < -STEPPER_2PI) d += STEPPER_2PI;
        }
        sweep = fabsf(d);
    }

    dirn = ccw ? 1 : -1;

    /* 软限位预判：沿弧取 25 个采样点，任一越界整段拒绝（回零后才生效） */
    if (mx->homed && my->homed)
    {
        cxm = (float)mx->position * mx->delta + i;     /* 圆心绝对坐标 mm */
        cym = (float)my->position * my->delta + j;
        for (k = 0; k <= 25; k++)
        {
            aa = a0 + (float)dirn * sweep * (float)k / 25.0f;
            xm = cxm + r * cosf(aa);
            ym = cym + r * sinf(aa);
            if (xm < mx->sl_min || xm > mx->sl_max) return 0;
            if (ym < my->sl_min || ym > my->sl_max) return 0;
        }
    }

    /* 终点脉冲数：圆弧结束时用它把不足一个脉冲的残量补到位 */
    {
        float lx = dx / mx->delta;
        float ly = dy / my->delta;
        nx = (int32_t)(lx + ((lx >= 0.0f) ? 0.5f : -0.5f));
        ny = (int32_t)(ly + ((ly >= 0.0f) ? 0.5f : -0.5f));
    }

    /* 起步方向：用起点处的切向分量符号（此时 v=0，只借方向） */
    vx0 = -(float)dirn * sinf(a0);
    vy0 =  (float)dirn * cosf(a0);
    dx0 = (vx0 > 1.0e-6f) ? 1 : ((vx0 < -1.0e-6f) ? -1 : dirn);
    dy0 = (vy0 > 1.0e-6f) ? 1 : ((vy0 < -1.0e-6f) ? -1 : dirn);

    a_lin = mx->acc_cmd;
    if (my->acc_cmd < a_lin) a_lin = my->acc_cmd;

    /* 填好参数再置 on，避免中断里读到半成品 */
    s_arc.r         = r;
    s_arc.a0        = a0;
    s_arc.theta     = a0;
    s_arc.sweep     = sweep;
    s_arc.done      = 0.0f;
    s_arc.dir       = dirn;
    s_arc.omega     = 0.0f;
    s_arc.omega_cmd = feed / r;
    s_arc.alpha     = a_lin / r;
    s_arc.phase     = STEPPER_ACC;
    s_arc.tx        = mx->position + nx;
    s_arc.ty        = my->position + ny;

    if (!Stepper_Begin(mx, dx0, mx->position, feed))
        return 0;

    if (!Stepper_Begin(my, dy0, my->position, feed))
    {
        Stepper_HaltRaw(STEPPER_X);            /* 第二轴起不来，撤掉第一轴 */
        return 0;
    }

    mx->arc_mode = 1;
    my->arc_mode = 1;
    mx->state    = STEPPER_CRUISE;             /* 只表示"忙"，真实曲线由 s_arc 管 */
    my->state    = STEPPER_CRUISE;

    s_arc.on = 1;
    return 1;
}

uint8_t Stepper_IsArcMode(void)
{
    return s_arc.on;
}

/* 换向：只有该轴"几乎不出脉冲"时才改 DIR（等价于速度为 0 时换向） */
static void Stepper_ArcSetDir(StepperAxis *m, float v)
{
    int8_t want;

    if (v > 0.0f)      want = 1;
    else if (v < 0.0f) want = -1;
    else               return;

    if (want == m->direction)
        return;

    /* 阈值 1 mm/s 对应的 K：此时出一个脉冲要 1500 拍（30 ms），
       期间改 DIR 绝对安全，驱动器有充足建立时间 */
    if ((float)m->increment > 1.0f * m->k_gain)
        return;

    m->increment  = 0;
    m->phase      = 0;              /* 丢掉不足一个脉冲的残量，误差 < 1 个当量 */
    m->direction  = want;
    Stepper_ApplyDirection(m, want);
    m->start_wait = 2;              /* 2 拍 × 20 µs = 40 µs 方向建立 */
}

/* 圆弧的 1 ms 规划：两轴耦合，所以不在按轴循环里做 */
static void Stepper_ArcTick(void)
{
    StepperAxis *mx = &motor[STEPPER_X];
    StepperAxis *my = &motor[STEPPER_Y];
    float rem, w_brake, dth, mid, v, vx, vy;

    if (!s_arc.on)
        return;

    if (mx->alarm || my->alarm)
    {
        Stepper_AbortArc();
        return;
    }

    rem = s_arc.sweep - s_arc.done;
    if (rem <= 0.0f)
    {
        Stepper_ArcFinish();
        return;
    }

    /* 角速度梯形：α 上升、ω_cmd 封顶、√(2α·剩余角) 制动 */
    w_brake = sqrtf(2.0f * s_arc.alpha * rem);

    switch (s_arc.phase)
    {
    case STEPPER_ACC:
        s_arc.omega += s_arc.alpha * STEPPER_TP_S;
        if (s_arc.omega >= s_arc.omega_cmd)
        {
            s_arc.omega = s_arc.omega_cmd;
            s_arc.phase = STEPPER_CRUISE;
        }
        if (s_arc.omega > w_brake)
        {
            s_arc.omega = w_brake;
            s_arc.phase = STEPPER_DEC;         /* 短弧 → 三角形 */
        }
        break;

    case STEPPER_CRUISE:
        s_arc.omega = s_arc.omega_cmd;
        if (s_arc.omega > w_brake)
        {
            s_arc.omega = w_brake;
            s_arc.phase = STEPPER_DEC;
        }
        break;

    default:
        s_arc.omega = w_brake;                 /* 每毫秒按剩余角反算，误差自校正 */
        break;
    }

    dth = s_arc.omega * STEPPER_TP_S;
    if (dth > rem)
        dth = rem;
    if (dth <= 0.0f)
        return;

    /* ★ 中点角：方向取本毫秒扫过的角的中点，而不是末端 */
    mid = s_arc.theta + (float)s_arc.dir * dth * 0.5f;
    s_arc.theta += (float)s_arc.dir * dth;
    s_arc.done  += dth;

    v  = s_arc.omega * s_arc.r;                /* ★ 切向速度 = ω·r */
    vx = -(float)s_arc.dir * sinf(mid) * v;
    vy =  (float)s_arc.dir * cosf(mid) * v;

    Stepper_ArcSetDir(mx, vx);
    Stepper_ArcSetDir(my, vy);

    mx->v_plan = fabsf(vx);
    my->v_plan = fabsf(vy);
    mx->increment = Stepper_SpeedToIncrement(mx, fabsf(vx));
    my->increment = Stepper_SpeedToIncrement(my, fabsf(vy));
}

/* 圆弧正常结束：收尾把 1~3 个当量的残量补到位，保证往返一致 */
static void Stepper_ArcFinish(void)
{
    StepperAxis *mx = &motor[STEPPER_X];
    StepperAxis *my = &motor[STEPPER_Y];
    int32_t ex, ey;

    s_arc.on    = 0;
    s_arc.phase = STEPPER_IDLE;

    mx->arc_mode = 0;
    my->arc_mode = 0;
    mx->increment = 0;
    my->increment = 0;
    mx->v_plan = 0.0f;
    my->v_plan = 0.0f;
    mx->state  = STEPPER_IDLE;
    my->state  = STEPPER_IDLE;

    ex = s_arc.tx - mx->position;
    ey = s_arc.ty - my->position;

    if (ex != 0)
        Stepper_MoveRel(STEPPER_X, (float)ex * mx->delta, 5.0f);
    if (ey != 0)
        Stepper_MoveRel(STEPPER_Y, (float)ey * my->delta, 5.0f);
}

/* 中止圆弧（STOP / ESTOP / 限位 / 报警）：不做减速收尾，两轴直接封脉冲 */
static void Stepper_AbortArc(void)
{
    if (!s_arc.on)
        return;

    s_arc.on    = 0;
    s_arc.phase = STEPPER_IDLE;

    Stepper_HaltRaw(STEPPER_X);
    Stepper_HaltRaw(STEPPER_Y);
}

/* --------------------------------------------------------------------------
 *  减速停止：把目标改到"当前速度下的制动距离终点"，
 *  再交给 √(2as) 曲线自己收尾 —— 点动和定长共用同一套减速逻辑
 * -------------------------------------------------------------------------- */
void Stepper_Stop(uint8_t id)
{
    StepperAxis *m;
    float   d;
    int32_t n;

    if (id >= STEPPER_NUM)
        return;

    /* 圆弧没有"改目标再减速"这条路（它的目标是一段角度，不是脉冲数），
       收到停止就直接两轴同时封脉冲 */
    if (s_arc.on)
    {
        Stepper_AbortArc();
        return;
    }

    m = &motor[id];

    __disable_irq();

    if (m->state != STEPPER_IDLE)
    {
        if (m->v_plan < 0.5f)             /* 还没真正动起来，直接停 */
        {
            m->increment = 0;
            m->v_plan    = 0.0f;
            m->state     = STEPPER_IDLE;
        }
        else
        {
            d = m->v_plan * m->v_plan / (2.0f * m->acc);   /* 制动距离 mm */
            n = (int32_t)(d / m->delta) + 1;

            m->target = m->position + m->direction * n;
            m->state  = STEPPER_DEC;
        }
    }

    __enable_irq();
}

/* --------------------------------------------------------------------------
 *  立即停止：不减速，直接封脉冲（急停、硬限位用）
 * -------------------------------------------------------------------------- */
static void Stepper_HaltRaw(uint8_t id)
{
    if (id >= STEPPER_NUM)
        return;

    __disable_irq();

    motor[id].increment    = 0;
    motor[id].v_plan       = 0.0f;
    motor[id].state        = STEPPER_IDLE;
    motor[id].start_wait   = 0;
    motor[id].pulse_active = 0;
    motor[id].arc_mode     = 0;

    HAL_GPIO_WritePin(motor[id].step_port, motor[id].step_pin, GPIO_PIN_SET);

    __enable_irq();
}

void Stepper_Halt(uint8_t id)
{
    /* 圆弧是两轴耦合的，停一根轴等于整段作废：两轴一起停 */
    if (s_arc.on)
    {
        Stepper_AbortArc();
        return;
    }

    Stepper_HaltRaw(id);
}

void Stepper_StopAll(void)
{
    Stepper_Stop(STEPPER_X);
    Stepper_Stop(STEPPER_Y);
}

void Stepper_HaltAll(void)
{
    Stepper_Halt(STEPPER_X);
    Stepper_Halt(STEPPER_Y);
}

/* --------------------------------------------------------------------------
 *  硬限位 / 急停（F5、F7）：EXTI 里调，必须极短
 * -------------------------------------------------------------------------- */
static void Stepper_OnLimitHit(uint8_t id, int8_t side)
{
    StepperAxis *m = &motor[id];

    if (!s_limits_enabled)
        return;

    /* 回零过程中负限位是预期信号，不报警，交给 1 ms 状态机处理 */
    if (m->home_state != STEPPER_HOME_NONE &&
        m->home_state != STEPPER_HOME_DONE &&
        m->home_state != STEPPER_HOME_FAIL)
    {
        if (side < 0)
            return;
    }

    Stepper_Halt(id);
    m->alarm      = 1;
    m->alarm_code = STEPPER_ALM_LIMIT;

    if (m->home_state != STEPPER_HOME_NONE)
        m->home_state = STEPPER_HOME_FAIL;
}

/* 急停：硬件按钮（EXTI）与上位机 ESTOP 指令共用这一条路径（F7） */
void Stepper_EmergencyStop(void)
{
    for (int i = 0; i < STEPPER_NUM; i++)
    {
        Stepper_Halt((uint8_t)i);
        motor[i].alarm      = 1;
        motor[i].alarm_code = STEPPER_ALM_ESTOP;
        motor[i].homed      = 0;      /* 急停后必须重新回零 */
        if (motor[i].home_state != STEPPER_HOME_NONE)
            motor[i].home_state = STEPPER_HOME_FAIL;
    }
}

/* HAL 外部中断回调：4 个限位 + 1 个急停 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
    case X_LIMIT_NEG_Pin:  Stepper_OnLimitHit(STEPPER_X, -1); break;
    case X_LIMIT_POS_Pin:  Stepper_OnLimitHit(STEPPER_X, +1); break;
    case Y_LIMIT_NEG_Pin:  Stepper_OnLimitHit(STEPPER_Y, -1); break;
    case Y_LIMIT_POS_Pin:  Stepper_OnLimitHit(STEPPER_Y, +1); break;
    case EMERGENCY_STOP_Pin: Stepper_EmergencyStop();        break;
    default: break;
    }
}

void Stepper_EnableLimits(uint8_t enable)
{
    s_limits_enabled = (enable != 0) ? 1 : 0;
}

/* --------------------------------------------------------------------------
 *  回零状态机（F4）：1 ms 节拍驱动
 *    ① 负向快速趋近负限位 → ② 触发后减速停 → ③ 正向慢速退出至开关释放
 *    → ④ 负向 1 mm/s 二次趋近，触发点即机械零点 → ⑤ 退离至 +2 mm
 *  为什么最后一段必须 1 mm/s：光电开关的触发点随速度变化，速度越快离散越大，
 *  只有低速趋近才能把回零重复性控制在 0.05 mm 以内。
 * -------------------------------------------------------------------------- */
static void Stepper_HomeTick(uint8_t id)
{
    StepperAxis *m = &motor[id];
    uint8_t neg;

    if (m->home_timer < 0xFFFFu)
        m->home_timer++;

    if (m->home_timer > STEPPER_HOME_TIMEOUT_MS)
    {
        Stepper_Halt(id);
        m->home_state = STEPPER_HOME_FAIL;
        m->alarm      = 1;
        m->alarm_code = STEPPER_ALM_HOME;
        return;
    }

    neg = Stepper_LimitNegActive(m);

    switch (m->home_state)
    {
    case STEPPER_HOME_FAST:                 /* ① 快速趋近 */
        if (neg)
        {
            Stepper_Stop(id);               /* 触发后减速停（10 mm/s 不快，允许减速） */
            m->home_state = STEPPER_HOME_WAIT1;
            m->home_timer = 0;
        }
        break;

    case STEPPER_HOME_WAIT1:                /* ② 等停稳，然后慢速退出 */
        if (!Stepper_IsBusy(id))
        {
            if (Stepper_Start(id, +1, m->home_slow))
            {
                m->home_state = STEPPER_HOME_BACKOFF;
                m->home_timer = 0;
            }
            else
            {
                m->home_state = STEPPER_HOME_FAIL;
                m->alarm      = 1;
                m->alarm_code = STEPPER_ALM_HOME;
            }
        }
        break;

    case STEPPER_HOME_BACKOFF:              /* ③ 等开关释放，立即停 */
        if (!neg)
        {
            Stepper_Halt(id);
            m->home_state = STEPPER_HOME_WAIT2;
            m->home_timer = 0;
        }
        break;

    case STEPPER_HOME_WAIT2:                /* 等停稳，然后二次低速趋近 */
        if (!Stepper_IsBusy(id))
        {
            if (Stepper_Start(id, -1, m->home_slow))
            {
                m->home_state = STEPPER_HOME_SLOW;
                m->home_timer = 0;
            }
            else
            {
                m->home_state = STEPPER_HOME_FAIL;
                m->alarm      = 1;
                m->alarm_code = STEPPER_ALM_HOME;
            }
        }
        break;

    case STEPPER_HOME_SLOW:                 /* ④ 二次趋近，触发点即机械零点 */
        if (neg)
        {
            Stepper_Halt(id);               /* 1 mm/s 直接停，不会丢步，重复性最好 */
            m->position   = 0;              /* 机械零点 = 当前点 */
            m->home_state = STEPPER_HOME_WAIT3;
            m->home_timer = 0;
        }
        break;

    case STEPPER_HOME_WAIT3:                /* ⑤ 退离至 +2 mm */
        if (!Stepper_IsBusy(id))
        {
            if (Stepper_MoveRel(id, STEPPER_HOME_OFFSET_MM, m->home_fast))
            {
                m->home_state = STEPPER_HOME_TOZERO;
                m->home_timer = 0;
            }
            else
            {
                m->home_state = STEPPER_HOME_FAIL;
                m->alarm      = 1;
                m->alarm_code = STEPPER_ALM_HOME;
            }
        }
        break;

    case STEPPER_HOME_TOZERO:
        if (!Stepper_IsBusy(id))
        {
            m->homed      = 1;
            m->home_state = STEPPER_HOME_DONE;
        }
        break;

    default:
        break;
    }
}

uint8_t Stepper_HomeStart(uint8_t id)
{
    StepperAxis *m;

    if (id >= STEPPER_NUM)
        return 0;

    m = &motor[id];

    if (m->alarm || m->state != STEPPER_IDLE)
        return 0;

    m->homed      = 0;
    m->home_state = STEPPER_HOME_FAST;
    m->home_timer = 0;

    return Stepper_Start(id, -1, m->home_fast);
}

void Stepper_HomeAbort(uint8_t id)
{
    if (id >= STEPPER_NUM)
        return;

    Stepper_Halt(id);
    motor[id].home_state = STEPPER_HOME_NONE;
}

uint8_t Stepper_IsHomed(uint8_t id)
{
    if (id >= STEPPER_NUM)
        return 0;
    return motor[id].homed;
}

uint8_t Stepper_IsAlarm(uint8_t id)
{
    if (id >= STEPPER_NUM)
        return 0;
    return motor[id].alarm;
}

uint8_t Stepper_GetAlarmCode(uint8_t id)
{
    if (id >= STEPPER_NUM)
        return 0;
    return motor[id].alarm_code;
}

/* 清除报警：注意"已回零"同时作废，必须重新回零（任务书 F7 要求） */
void Stepper_ResetAlarm(uint8_t id)
{
    if (id >= STEPPER_NUM)
        return;

    __disable_irq();
    motor[id].alarm      = 0;
    motor[id].alarm_code = STEPPER_ALM_NONE;
    motor[id].homed      = 0;
    if (motor[id].home_state == STEPPER_HOME_FAIL)
        motor[id].home_state = STEPPER_HOME_NONE;
    __enable_irq();
}

/* --------------------------------------------------------------------------
 *  状态查询
 * -------------------------------------------------------------------------- */
int32_t Stepper_GetPosition(uint8_t id)
{
    if (id >= STEPPER_NUM)
        return 0;
    return motor[id].position;
}

float Stepper_GetPositionMM(uint8_t id)
{
    if (id >= STEPPER_NUM)
        return 0.0f;
    return (float)motor[id].position * motor[id].delta;
}

/* 当前规划速度，带方向符号（空闲时恒为 0） */
float Stepper_GetSpeedMM(uint8_t id)
{
    if (id >= STEPPER_NUM || motor[id].state == STEPPER_IDLE)
        return 0.0f;
    return motor[id].v_plan * (float)motor[id].direction;
}

uint8_t Stepper_IsBusy(uint8_t id)
{
    if (id >= STEPPER_NUM)
        return 0;
    return (motor[id].state != STEPPER_IDLE) ? 1 : 0;
}

uint8_t Stepper_GetState(uint8_t id)
{
    if (id >= STEPPER_NUM)
        return STEPPER_IDLE;
    return motor[id].state;
}

/* --------------------------------------------------------------------------
 *  参数设置（F9）：全部做合法性检查，越界返回 0
 *  说明：Vmax 还受"50 kHz 节拍、脉宽占 2 拍"限制，上限 = 25 kHz × δ
 * -------------------------------------------------------------------------- */
float Stepper_GetLead(uint8_t id)     { return (id < STEPPER_NUM) ? motor[id].lead : 0.0f; }
uint16_t Stepper_GetPPR(uint8_t id)   { return (id < STEPPER_NUM) ? motor[id].ppr : 0; }
uint8_t Stepper_GetDirInv(uint8_t id) { return (id < STEPPER_NUM) ? motor[id].dir_inv : 0; }
float Stepper_GetAccel(uint8_t id)    { return (id < STEPPER_NUM) ? motor[id].acc_cmd : 0.0f; }
float Stepper_GetVmax(uint8_t id)     { return (id < STEPPER_NUM) ? motor[id].vmax : 0.0f; }
float Stepper_GetSoftMin(uint8_t id)  { return (id < STEPPER_NUM) ? motor[id].sl_min : 0.0f; }
float Stepper_GetSoftMax(uint8_t id)  { return (id < STEPPER_NUM) ? motor[id].sl_max : 0.0f; }
float Stepper_GetHomeFast(uint8_t id) { return (id < STEPPER_NUM) ? motor[id].home_fast : 0.0f; }
float Stepper_GetHomeSlow(uint8_t id) { return (id < STEPPER_NUM) ? motor[id].home_slow : 0.0f; }
float Stepper_GetDelta(uint8_t id)    { return (id < STEPPER_NUM) ? motor[id].delta : 0.0f; }

/* 该细分下允许的最高速度：K ≤ 2^31 且脉宽占 2 拍 → 输出频率上限 25 kHz */
static float Stepper_VmaxLimit(const StepperAxis *m)
{
    return 25000.0f * m->delta;
}

uint8_t Stepper_SetAccel(uint8_t id, float acc)
{
    if (id >= STEPPER_NUM || acc < 10.0f || acc > 500.0f)
        return 0;

    motor[id].acc     = acc;
    motor[id].acc_cmd = acc;
    return 1;
}

uint8_t Stepper_SetVmax(uint8_t id, float vmax)
{
    if (id >= STEPPER_NUM || vmax < 1.0f || vmax > 60.0f)
        return 0;

    if (vmax > Stepper_VmaxLimit(&motor[id]))
        return 0;

    motor[id].vmax = vmax;
    return 1;
}

void Stepper_SetSoftLimit(uint8_t id, float min_mm, float max_mm)
{
    if (id >= STEPPER_NUM || min_mm >= max_mm)
        return;

    motor[id].sl_min = min_mm;
    motor[id].sl_max = max_mm;
}

uint8_t Stepper_SetLead(uint8_t id, float lead_mm)
{
    if (id >= STEPPER_NUM || lead_mm < 1.0f || lead_mm > 20.0f)
        return 0;

    motor[id].lead = lead_mm;
    Stepper_Recompute(&motor[id]);

    /* 机械参数变了，原坐标不再可信 → 作废回零标志（需重新回零） */
    motor[id].homed = 0;
    return 1;
}

uint8_t Stepper_SetPPR(uint8_t id, uint16_t ppr)
{
    uint8_t ok = 0;

    if (id >= STEPPER_NUM)
        return 0;

    for (uint8_t i = 0; i < STEPPER_PPR_GEARS; i++)
    {
        if (STEPPER_PPR_TABLE[i] == ppr)
        {
            ok = 1;
            break;
        }
    }

    if (!ok)
        return 0;

    motor[id].ppr = ppr;
    Stepper_Recompute(&motor[id]);

    /* 细分变了，脉冲当量变 ⇒ 坐标与软限位全部作废 */
    motor[id].homed = 0;

    if (motor[id].vmax > Stepper_VmaxLimit(&motor[id]))
        motor[id].vmax = Stepper_VmaxLimit(&motor[id]);

    return 1;
}

uint8_t Stepper_SetDirInv(uint8_t id, uint8_t inv)
{
    if (id >= STEPPER_NUM || inv > 1 || motor[id].state != STEPPER_IDLE)
        return 0;

    motor[id].dir_inv = inv;
    return 1;
}

uint8_t Stepper_SetHomeFast(uint8_t id, float v)
{
    if (id >= STEPPER_NUM || v < 5.0f || v > 20.0f)
        return 0;

    motor[id].home_fast = v;
    return 1;
}

uint8_t Stepper_SetHomeSlow(uint8_t id, float v)
{
    if (id >= STEPPER_NUM || v < 0.5f || v > 2.0f)
        return 0;

    motor[id].home_slow = v;
    return 1;
}

/* --------------------------------------------------------------------------
 *  通信超时保护（F8）：1 ms 规划里判断，超过 500 ms 没报文就减速停
 * -------------------------------------------------------------------------- */
void Stepper_NoteHostMessage(void)
{
    s_last_rx_ms = HAL_GetTick();
    s_host_seen  = 1;
    s_comm_trip  = 0;        /* 重新收到报文，允许下一次超时再次触发 */
    s_comm_lost  = 0;
}

uint8_t Stepper_IsCommLost(void)
{
    return s_comm_lost;
}

/* --------------------------------------------------------------------------
 *  运动记录（F12）
 * -------------------------------------------------------------------------- */
uint8_t Stepper_TraceArm(uint8_t id)
{
    if (id >= STEPPER_NUM)
        return 0;

    __disable_irq();
    s_trace_axis = (int8_t)id;
    s_trace_stop = 0xFFFFu;
    for (uint8_t i = 0; i < STEPPER_NUM; i++)
    {
        motor[i].tr_on   = 0;
        motor[i].tr_n    = 0;
        motor[i].tr_last = motor[i].position;
    }
    __enable_irq();
    return 1;
}

void Stepper_TraceMarkStop(void)
{
    if (s_trace_axis >= 0)
    {
        uint16_t n = motor[(uint8_t)s_trace_axis].tr_n;
        if (s_trace_stop == 0xFFFFu)
            s_trace_stop = n;
    }
}

int8_t Stepper_TraceAxis(void)          { return s_trace_axis; }
uint16_t Stepper_TraceStopIndex(void)   { return s_trace_stop; }

uint16_t Stepper_TraceCount(void)
{
    if (s_trace_axis < 0)
        return 0;
    return motor[(uint8_t)s_trace_axis].tr_n;
}

uint16_t Stepper_TraceGet(uint16_t i)
{
    if (s_trace_axis < 0 || i >= STEPPER_TRACE_MAX)
        return 0;
    return s_trace_buf[i];
}

/* 1 ms 采样：记录本毫秒内的脉冲增量（幅度），运动结束自动停止记录 */
static void Stepper_TraceSample(uint8_t id)
{
    StepperAxis *m = &motor[id];
    int32_t d;

    if (s_trace_axis != (int8_t)id)
        return;

    if (!m->tr_on)
    {
        if (m->state != STEPPER_IDLE)      /* 运动刚开始：从这一拍开始记 */
        {
            m->tr_on   = 1;
            m->tr_n    = 0;
            m->tr_last = m->position;
        }
        return;
    }

    d = m->position - m->tr_last;
    m->tr_last = m->position;

    if (d < 0)
        d = -d;

    if (m->tr_n < STEPPER_TRACE_MAX)
        s_trace_buf[m->tr_n++] = (uint16_t)d;

    if (m->state == STEPPER_IDLE)          /* 到位：记录结束 */
        m->tr_on = 0;
}

uint8_t Stepper_GetLimitMask(void)
{
    uint8_t mask = 0;

    if (HAL_GPIO_ReadPin(motor[STEPPER_X].lim_neg_port, motor[STEPPER_X].lim_neg_pin) == GPIO_PIN_SET)
        mask |= 0x01;   /* X- */
    if (HAL_GPIO_ReadPin(motor[STEPPER_X].lim_pos_port, motor[STEPPER_X].lim_pos_pin) == GPIO_PIN_SET)
        mask |= 0x02;   /* X+ */
    if (HAL_GPIO_ReadPin(motor[STEPPER_Y].lim_neg_port, motor[STEPPER_Y].lim_neg_pin) == GPIO_PIN_SET)
        mask |= 0x04;   /* Y- */
    if (HAL_GPIO_ReadPin(motor[STEPPER_Y].lim_pos_port, motor[STEPPER_Y].lim_pos_pin) == GPIO_PIN_SET)
        mask |= 0x08;   /* Y+ */

    return mask;
}

/* --------------------------------------------------------------------------
 *  1 ms 运动规划（SysTick 中断里调用）
 * -------------------------------------------------------------------------- */
void Stepper_Planner_1ms(void)
{
    StepperAxis *m;
    int32_t  remain;
    float    v_brake;

    /* F8 通信超时：收到过上位机报文、超过 500 ms 无报文 → 减速停
       ★ 必须是一次性动作：Stepper_Stop 会把目标改成"当前位置 + 制动距离"，
         如果每毫秒重复调用，目标就跟位置一起往前跑，永远追不上，电机会一直转下去 */
    if (s_host_seen && !s_comm_trip &&
        (HAL_GetTick() - s_last_rx_ms) > STEPPER_COMM_TIMEOUT_MS)
    {
        s_comm_trip = 1;
        s_comm_lost = 1;
        Stepper_StopAll();
    }

    /* 圆弧：两轴耦合，必须在按轴循环之前统一算一次，
       算完把两轴的 increment / v_plan 都设好，循环里跳过常规梯形 */
    if (s_arc.on)
        Stepper_ArcTick();

    for (int i = 0; i < STEPPER_NUM; i++)
    {
        m = &motor[i];

        /* F12：每毫秒采样一次脉冲增量 */
        Stepper_TraceSample((uint8_t)i);

        /* 圆弧模式下这一轴的速度由 ArcTick 分配，常规梯形不参与 */
        if (m->arc_mode)
            continue;

        /* 报警锁定：不产生任何脉冲 */
        if (m->alarm)
        {
            m->increment = 0;
            continue;
        }

        /* 回零流程优先 */
        if (m->home_state != STEPPER_HOME_NONE &&
            m->home_state != STEPPER_HOME_DONE &&
            m->home_state != STEPPER_HOME_FAIL)
        {
            Stepper_HomeTick((uint8_t)i);
        }

        if (m->state == STEPPER_IDLE)
        {
            m->increment = 0;
            m->v_plan    = 0.0f;
            continue;
        }

        /* 方向建立期间不出脉冲 */
        if (m->start_wait > 0)
            continue;

        remain = m->target - m->position;
        if (remain < 0)
            remain = -remain;

        /* 制动速度上限：v ≤ √(2·a·s)，s = remain·δ */
        v_brake = sqrtf(2.0f * m->acc * (float)remain * m->delta);

        switch (m->state)
        {
        case STEPPER_ACC:
            m->v_plan += m->acc * STEPPER_TP_S;
            if (m->v_plan >= m->v_cmd)
            {
                m->v_plan = m->v_cmd;
                m->state  = STEPPER_CRUISE;
            }
            if (m->v_plan > v_brake)
            {
                m->v_plan = v_brake;
                m->state  = STEPPER_DEC;    /* 短行程 → 三角形曲线 */
            }
            break;

        case STEPPER_CRUISE:
            m->v_plan = m->v_cmd;
            if (m->v_plan > v_brake)
            {
                m->v_plan = v_brake;
                m->state  = STEPPER_DEC;
            }
            break;

        case STEPPER_DEC:
        default:
            m->v_plan = v_brake;            /* 每毫秒用剩余距离反算，误差自校正 */
            break;
        }

        m->increment = Stepper_SpeedToIncrement(m, m->v_plan);
    }
}

/* --------------------------------------------------------------------------
 *  50 kHz 节拍：累加 → 溢出出脉冲 → 位置 ±1 → 判断到位
 * -------------------------------------------------------------------------- */
static void Stepper_Axis_Tick(StepperAxis *m)
{
    uint32_t old_phase;

    /* 释放上一拍的脉冲（脉宽 20 µs） */
    if (m->pulse_active)
    {
        HAL_GPIO_WritePin(m->step_port, m->step_pin, GPIO_PIN_SET);
        m->pulse_active = 0;
    }

    /* 方向建立等待 */
    if (m->start_wait > 0)
    {
        m->start_wait--;
        return;
    }

    if (m->increment == 0)
        return;

    old_phase = m->phase;

    /* 32 位相位累加 */
    m->phase += m->increment;

    /* 溢出 = 走一个脉冲当量 */
    if (m->phase < old_phase)
    {
        HAL_GPIO_WritePin(m->step_port, m->step_pin, GPIO_PIN_RESET);
        m->pulse_active = 1;

        m->position += m->direction;

        /* 到达目标脉冲数，精确停止。
           圆弧模式下不做这个判定：圆弧中途会多次换向，position 来回走，
           用 position == target 会在第一个过零点就误判到位 */
        if (!m->arc_mode && m->state != STEPPER_IDLE && m->position == m->target)
        {
            m->increment = 0;
            m->v_plan    = 0.0f;
            m->state     = STEPPER_IDLE;
        }
    }
}

/* TIM2 更新中断回调体 */
void Stepper_TIM_Callback(void)
{
    Stepper_Axis_Tick(&motor[STEPPER_X]);
    Stepper_Axis_Tick(&motor[STEPPER_Y]);
}
