/**
  ******************************************************************************
  * @file    protocol.h
  * @brief   串口通信协议层（F1~F12）
  *
  *  格式：一条指令一行 ASCII 文本，以 \n 结尾（\r 忽略），115200 8N1。
  *  设计理由：可以直接用串口助手手工发送和阅读，便于第 3 周"先单独测下位机"。
  *
  *  指令表：
  *    PING                                    心跳，回 PONG
  *    JOG  X+ [F20]                           连续运动（点动）
  *    MOV  X-12.5 F20                          定长运动（相对，mm）
  *    MOVA X100 F30                           绝对定位（须已回零）
  *    LINE X10 Y5 F20                         两轴联动斜线（同时启动、同时到达）
  *    STOP [X]                                减速停止（不带轴号 = 两轴）
  *    ESTOP                                   急停：立即停 + 报警锁定
  *    HOME X                                  回零
  *    RESET                                   复位报警
  *    SET  X LEAD=5 PPR=1600 ACC=200 ...      参数设置（可只带部分）
  *    GET  X                                  参数读取
  *    TRACE X                                 武装运动记录（下一次运动开始记录）
  *    DUMP                                    上传记录数据
  *    HELP                                    指令列表
  *
  *  应答：OK  /  ERR,<code>
  *    1 语法错误  2 参数越界  3 未回零  4 超软限位  5 报警锁定中  6 该轴忙  7 未知指令
  *
  *  状态上报：每 100 ms 一条
  *    <ST X=12.500,25.00,RUN,H1 Y=0.000,0.00,IDLE,H0 LIM=0000 ALM=0>
  ******************************************************************************
  */
#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "main.h"

/* 错误码 */
#define PROTO_ERR_SYNTAX    1
#define PROTO_ERR_RANGE     2
#define PROTO_ERR_NOHOME    3
#define PROTO_ERR_SOFTLIMIT 4
#define PROTO_ERR_ALARM     5
#define PROTO_ERR_BUSY      6
#define PROTO_ERR_UNKNOWN   7

/* 状态上报周期 */
#define PROTO_STATUS_MS     100u

void Protocol_Init(void);

/* 主循环调用：取行 → 分派 → 定时状态上报 */
void Protocol_Poll(void);

#endif /* __PROTOCOL_H */
