/**
  ******************************************************************************
  * @file    stepper.h
  * @brief   两轴步进运动控制 —— 运动规划层 + 脉冲发生层 + I/O 保护
  *          脉冲发生：固定节拍相位累加器，ft = 50 kHz
  *          运动规划：Tp = 1 ms，梯形加减速（短行程自动退化三角形）
  ******************************************************************************
  */
#ifndef __STEPPER_H
#define __STEPPER_H

#include "main.h"

/* 轴号 */
#define STEPPER_X   0
#define STEPPER_Y   1
#define STEPPER_NUM 2

/* 节拍频率：TIM2 = 72 MHz / (71+1) / (19+1) = 50 kHz */
#define STEPPER_TICK_HZ 50000UL

/* 规划周期 */
#define STEPPER_TP_S    0.001f

/* 默认运动参数（可由上位机 SET 指令改写） */
#define STEPPER_ACC_DEFAULT   200.0f    /* 加速度 mm/s^2 */
#define STEPPER_VMAX_DEFAULT  50.0f     /* 最高速度 mm/s */
#define STEPPER_VMIN          0.1f      /* 最低速度 mm/s */

/* 回零参数 */
#define STEPPER_HOME_FAST_DEFAULT  10.0f    /* 快速趋近 mm/s */
#define STEPPER_HOME_SLOW_DEFAULT   1.0f    /* 二次低速趋近 mm/s */
#define STEPPER_HOME_OFFSET_MM      2.0f    /* 回零后离开机械零点的距离 */
#define STEPPER_HOME_TIMEOUT_MS    20000u   /* 单阶段超时 */

/* 软限位默认范围（回零后生效） */
#define STEPPER_SLIMIT_MIN_DEFAULT  0.0f
#define STEPPER_SLIMIT_MAX_DEFAULT  240.0f

/* 每转脉冲数可选档位（F9）：只能取这 8 个值，且必须与驱动器拨码一致 */
#define STEPPER_PPR_GEARS   8
extern const uint16_t STEPPER_PPR_TABLE[STEPPER_PPR_GEARS];

/* 运动记录（F12）：1 ms 采样一次脉冲增量，最多 3000 点，单轴共用一块缓冲 */
#define STEPPER_TRACE_MAX   3000

/* 通信超时（F8）：连续运动时超过该时间没收到任何报文即减速停 */
#define STEPPER_COMM_TIMEOUT_MS  500u

/* 报警码 */
#define STEPPER_ALM_NONE    0
#define STEPPER_ALM_LIMIT   1   /* 硬限位 */
#define STEPPER_ALM_ESTOP   2   /* 急停 */
#define STEPPER_ALM_HOME    3   /* 回零失败 */

/* 运动状态机 */
typedef enum
{
    STEPPER_IDLE = 0,   /* 空闲 */
    STEPPER_ACC,        /* 加速段 */
    STEPPER_CRUISE,     /* 匀速段 */
    STEPPER_DEC         /* 减速段（三角形曲线的后半段也在这一段里） */
} StepperState;

/* 回零状态机 */
typedef enum
{
    STEPPER_HOME_NONE = 0,  /* 未回零 / 不在回零流程中 */
    STEPPER_HOME_FAST,      /* ① 负向快速趋近负限位 */
    STEPPER_HOME_WAIT1,     /* ② 减速停止中，等停稳 */
    STEPPER_HOME_BACKOFF,   /* ③ 正向慢速退出，等开关释放 */
    STEPPER_HOME_WAIT2,     /*    等停稳 */
    STEPPER_HOME_SLOW,      /* ④ 负向 1 mm/s 二次趋近（触发点即机械零点） */
    STEPPER_HOME_WAIT3,     /*    等停稳 */
    STEPPER_HOME_TOZERO,    /* ⑤ 退离至 +2 mm 处 */
    STEPPER_HOME_DONE,      /* 回零完成 */
    STEPPER_HOME_FAIL       /* 回零失败 */
} StepperHomeState;

/* 初始化（上电调用一次） */
void Stepper_Init(void);

/* 1 ms 运动规划：必须在 SysTick 中断里调用（见 stm32f1xx_it.c） */
void Stepper_Planner_1ms(void);

/* 连续运动（点动）：direction 取 +1 / -1，speed 单位 mm/s
   返回 1 = 已启动，0 = 参数非法或该轴忙 */
uint8_t Stepper_Start(uint8_t axis, int8_t direction, float speed);

/* 定长运动：distance_mm 正负表示方向，N = round(L/δ)
   返回 1 = 已启动，0 = 参数非法或该轴忙 */
uint8_t Stepper_MoveRel(uint8_t axis, float distance_mm, float speed);

/* 绝对定位（提高要求 MOVA）：须已回零，目标为机械坐标 mm */
uint8_t Stepper_MoveAbs(uint8_t axis, float pos_mm, float speed);

/* 两轴联动斜线（挑战要求②）：同时启动、同时到达
   dx / dy 为两轴相对位移 mm，feed 为沿轨迹的合成进给速度 mm/s
   两轴速度与加速度按位移比例分配，轨迹为严格直线
   返回 1 = 已启动（两轴同时），0 = 参数非法或任一轴忙/报警 */
uint8_t Stepper_MoveLine(float dx, float dy, float feed);

/* 圆弧插补（项目二 挑战要求②）：G02/G03，DDA 基准脉冲
   dx / dy  终点相对起点的位移 mm
   i  / j   圆心相对起点的偏移 mm（与 G 代码 I/J 语义相同）
   dx=dy=0 且有 I/J 时按整圆处理
   feed     切向进给速度 mm/s（沿弧长，不是单轴速度）
   返回 1 = 已启动，0 = 参数非法 / 终点不在圆上 / 超软限位 / 轴忙 */
uint8_t Stepper_MoveArc(float dx, float dy, float i, float j,
                        float feed, uint8_t ccw);

/* 1 = 当前正在走圆弧（上位机查询用） */
uint8_t Stepper_IsArcMode(void);

/* 减速停止：按设定加速度减速到 0（F1/F2 通用） */
void Stepper_Stop(uint8_t axis);

/* 立即停止：不减速直接封脉冲，急停/硬限位用 */
void Stepper_Halt(uint8_t axis);

void Stepper_StopAll(void);
void Stepper_HaltAll(void);

/* ---- 限位保护（F5） ---- */
/* 限位开关接好后才打开；未打开时程序行为与之前完全一致，不会误报警 */
void Stepper_EnableLimits(uint8_t enable);

/* ---- 回零（F4） ---- */
uint8_t Stepper_HomeStart(uint8_t axis);    /* 1 = 已启动回零流程 */
void    Stepper_HomeAbort(uint8_t axis);
uint8_t Stepper_IsHomed(uint8_t axis);

/* ---- 报警 ---- */
uint8_t Stepper_IsAlarm(uint8_t axis);
uint8_t Stepper_GetAlarmCode(uint8_t axis);
void    Stepper_ResetAlarm(uint8_t axis);   /* 清除报警，同时作废"已回零"状态 */

/* 急停（F7）：硬件急停按钮与上位机 ESTOP 指令共用 */
void    Stepper_EmergencyStop(void);

/* 状态查询 */
int32_t Stepper_GetPosition(uint8_t axis);          /* 指令位置，脉冲 */
float   Stepper_GetPositionMM(uint8_t axis);        /* 指令位置，mm */
float   Stepper_GetSpeedMM(uint8_t axis);           /* 当前规划速度，带方向符号 */
uint8_t Stepper_IsBusy(uint8_t axis);               /* 1 = 运动中 */
uint8_t Stepper_GetState(uint8_t axis);             /* StepperState */
uint8_t Stepper_GetLimitMask(void);                 /* bit0..3 = X- X+ Y- Y+ */

/* 参数读取（F9 的 GET 指令用） */
float    Stepper_GetLead(uint8_t axis);
uint16_t Stepper_GetPPR(uint8_t axis);
uint8_t  Stepper_GetDirInv(uint8_t axis);
float    Stepper_GetAccel(uint8_t axis);
float    Stepper_GetVmax(uint8_t axis);
float    Stepper_GetSoftMin(uint8_t axis);
float    Stepper_GetSoftMax(uint8_t axis);
float    Stepper_GetHomeFast(uint8_t axis);
float    Stepper_GetHomeSlow(uint8_t axis);
float    Stepper_GetDelta(uint8_t axis);            /* 脉冲当量 mm */

/* 参数设置（F9）：返回 1 = 成功，0 = 越界。改机械参数后坐标失效，须重新回零 */
uint8_t Stepper_SetAccel(uint8_t axis, float acc);
uint8_t Stepper_SetVmax(uint8_t axis, float vmax);
void    Stepper_SetSoftLimit(uint8_t axis, float min_mm, float max_mm);
uint8_t Stepper_SetLead(uint8_t axis, float lead_mm);
uint8_t Stepper_SetPPR(uint8_t axis, uint16_t ppr);
uint8_t Stepper_SetDirInv(uint8_t axis, uint8_t inv);
uint8_t Stepper_SetHomeFast(uint8_t axis, float v);
uint8_t Stepper_SetHomeSlow(uint8_t axis, float v);

/* ---- 通信超时保护（F8） ---- */
void    Stepper_NoteHostMessage(void);      /* 每收到一条报文调用一次 */
uint8_t Stepper_IsCommLost(void);           /* 1 = 曾因超时停车 */

/* ---- 运动记录（F12） ---- */
uint8_t  Stepper_TraceArm(uint8_t axis);    /* 武装：该轴下一次运动开始记录 */
void     Stepper_TraceMarkStop(void);       /* 收到停止指令时打标记 */
int8_t   Stepper_TraceAxis(void);           /* 记录所属轴，-1 = 无 */
uint16_t Stepper_TraceCount(void);
uint16_t Stepper_TraceStopIndex(void);      /* 停止时刻的采样序号，0xFFFF = 无 */
uint16_t Stepper_TraceGet(uint16_t i);      /* 第 i 个采样点的脉冲增量 */

/* TIM2 更新中断回调体 */
void Stepper_TIM_Callback(void);

#endif /* __STEPPER_H */
