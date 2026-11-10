/**
  ******************************************************************************
  * @file    protocol.c
  * @brief   串口通信协议层（F1~F12）：按行接收、取首个词查表分派
  *
  *  分层位置：本文件在"运动规划层"之上、"串口驱动"之下，
  *            只做文本解析与应答拼装，不直接碰脉冲引脚（任务书 4.1 要求）。
  *
  *  中断与主循环的分工：
  *    串口接收中断：只把字节塞进环形缓冲，立刻返回
  *    主循环 Poll  ：组行 → 查表 → 调运动层函数 → 回 OK / ERR,n
  *    100 ms 上报  ：主循环里按 HAL_GetTick 节拍发送，不在中断里发
  ******************************************************************************
  */
#include "protocol.h"
#include "stepper.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define PROTO_RX_SIZE   256
#define PROTO_LINE_MAX  96

/* ---- 接收环形缓冲（中断写、主循环读） ---- */
static volatile uint8_t  s_rx_byte;
static volatile uint8_t  s_rx_buf[PROTO_RX_SIZE];
static volatile uint16_t s_rx_head = 0;
static volatile uint16_t s_rx_tail = 0;

/* ---- 行缓冲 ---- */
static char    s_line[PROTO_LINE_MAX];
static uint16_t s_line_len = 0;

/* ---- 上报节拍 ---- */
static uint32_t s_status_last = 0;

/* --------------------------------------------------------------------------
 *  发送工具
 * -------------------------------------------------------------------------- */
static void Send(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 200);
}

static void ReplyOK(void)
{
    Send("OK\r\n");
}

static void ReplyErr(uint8_t code)
{
    char buf[16];
    sprintf(buf, "ERR,%u\r\n", (unsigned)code);
    Send(buf);
}

/* 浮点转文本：不用 %f，避免 MicroLIB 浮点格式化缺失的问题
   v 保留 dec 位小数，返回新的写指针 */
static char *FmtF(char *p, float v, uint8_t dec)
{
    uint32_t ip, fp, scale = 1;
    char     tmp[8];
    uint8_t  n = 0;

    if (v < 0.0f)
    {
        *p++ = '-';
        v = -v;
    }

    for (uint8_t i = 0; i < dec; i++)
        scale *= 10u;

    ip = (uint32_t)v;
    fp = (uint32_t)((v - (float)ip) * (float)scale + 0.5f);
    if (fp >= scale)
    {
        ip++;
        fp -= scale;
    }

    p += sprintf(p, "%lu", (unsigned long)ip);
    *p++ = '.';

    do
    {
        tmp[n++] = (char)('0' + (fp % 10u));
        fp /= 10u;
    } while (fp != 0 && n < sizeof(tmp));

    while (n < dec && n < sizeof(tmp))
        tmp[n++] = '0';

    while (n > 0)
        *p++ = tmp[--n];

    *p = '\0';
    return p;
}

/* --------------------------------------------------------------------------
 *  解析工具
 * -------------------------------------------------------------------------- */
static char *SkipSpace(char *p)
{
    while (*p == ' ' || *p == '\t')
        p++;
    return p;
}

/* 大写化，便于 "jog x+ f20" 也认 */
static void ToUpper(char *p)
{
    while (*p)
    {
        if (*p >= 'a' && *p <= 'z')
            *p = (char)(*p - 'a' + 'A');
        p++;
    }
}

/* 取轴号：X→0，Y→1，其它→-1；成功后 *pp 移到下一字符 */
static int8_t ParseAxis(char **pp)
{
    char *p = SkipSpace(*pp);

    if (*p == 'X' || *p == 'x') { *pp = p + 1; return STEPPER_X; }
    if (*p == 'Y' || *p == 'y') { *pp = p + 1; return STEPPER_Y; }

    return -1;
}

/* 在 p 中找关键字 F/G 后面跟的数字（F 为速度），找到返回 1 */
static uint8_t ParseFloatAfter(char *p, char key, float *out)
{
    while (*p)
    {
        if ((*p == key) || (*p == (char)(key + 32)))
        {
            char *q = p + 1;
            char *end;
            float v = strtof(q, &end);
            if (end != q)
            {
                *out = v;
                return 1;
            }
        }
        p++;
    }
    return 0;
}

/* --------------------------------------------------------------------------
 *  状态上报
 * -------------------------------------------------------------------------- */
static const char *StateName(uint8_t id)
{
    if (Stepper_IsAlarm(id))
        return "ALM";
    if (Stepper_IsHomed(id) == 0 && Stepper_IsBusy(id))
        return "RUN";
    if (Stepper_IsBusy(id))
        return "RUN";
    return "IDLE";
}

static void SendStatus(void)
{
    char  buf[144];
    char *p = buf;
    uint8_t mask = Stepper_GetLimitMask();
    uint8_t alm;

    *p++ = '<';
    *p++ = 'S';
    *p++ = 'T';
    *p++ = ' ';

    for (uint8_t i = 0; i < STEPPER_NUM; i++)
    {
        const char *st;

        *p++ = (i == STEPPER_X) ? 'X' : 'Y';
        *p++ = '=';
        p = FmtF(p, Stepper_GetPositionMM(i), 3);
        *p++ = ',';
        p = FmtF(p, Stepper_GetSpeedMM(i), 2);
        *p++ = ',';

        st = StateName(i);
        while (*st)
            *p++ = *st++;
        *p++ = ',';

        *p++ = 'H';
        *p++ = Stepper_IsHomed(i) ? '1' : '0';
        *p++ = ' ';
    }

    *p++ = 'L'; *p++ = 'I'; *p++ = 'M'; *p++ = '=';
    for (uint8_t b = 0; b < 4; b++)
        *p++ = (mask & (1u << b)) ? '1' : '0';

    *p++ = ' ';
    *p++ = 'A'; *p++ = 'L'; *p++ = 'M'; *p++ = '=';

    alm = Stepper_GetAlarmCode(STEPPER_X);
    if (Stepper_GetAlarmCode(STEPPER_Y) > alm)
        alm = Stepper_GetAlarmCode(STEPPER_Y);
    *p++ = (char)('0' + (alm > 9 ? 9 : alm));

    *p++ = '>';
    *p++ = '\r';
    *p++ = '\n';
    *p = '\0';

    Send(buf);
}

/* --------------------------------------------------------------------------
 *  指令处理
 * -------------------------------------------------------------------------- */
static void CmdPing(char *args)
{
    (void)args;
    Send("PONG\r\n");
}

static void CmdJog(char *args)
{
    int8_t  axis;
    char   *p = args;
    int8_t  dir;
    float   speed = 10.0f;

    axis = ParseAxis(&p);
    if (axis < 0)
    {
        ReplyErr(PROTO_ERR_SYNTAX);
        return;
    }

    p = SkipSpace(p);
    if (*p == '+')
    {
        dir = 1;
        p++;
    }
    else if (*p == '-')
    {
        dir = -1;
        p++;
    }
    else
    {
        ReplyErr(PROTO_ERR_SYNTAX);
        return;
    }

    ParseFloatAfter(p, 'F', &speed);

    if (Stepper_IsAlarm((uint8_t)axis))
    {
        ReplyErr(PROTO_ERR_ALARM);
        return;
    }

    if (speed < STEPPER_VMIN || speed > Stepper_GetVmax((uint8_t)axis))
    {
        ReplyErr(PROTO_ERR_RANGE);
        return;
    }

    if (!Stepper_Start((uint8_t)axis, dir, speed))
    {
        ReplyErr(PROTO_ERR_BUSY);
        return;
    }

    ReplyOK();
}

/* MOV / MOVA 共用：relative=1 → 相对位移，否则绝对坐标 */
static void MoveCommon(char *args, uint8_t relative)
{
    int8_t  axis;
    char   *p = args;
    char   *end;
    float   dist;
    float   speed = 10.0f;
    uint8_t ok;

    axis = ParseAxis(&p);
    if (axis < 0)
    {
        ReplyErr(PROTO_ERR_SYNTAX);
        return;
    }

    p = SkipSpace(p);
    dist = strtof(p, &end);
    if (end == p)
    {
        ReplyErr(PROTO_ERR_SYNTAX);
        return;
    }

    if (ParseFloatAfter(end, 'F', &speed) == 0)
        speed = 10.0f;

    if (Stepper_IsAlarm((uint8_t)axis))
    {
        ReplyErr(PROTO_ERR_ALARM);
        return;
    }

    if (!relative && !Stepper_IsHomed((uint8_t)axis))
    {
        ReplyErr(PROTO_ERR_NOHOME);
        return;
    }

    if (speed < STEPPER_VMIN || speed > Stepper_GetVmax((uint8_t)axis))
    {
        ReplyErr(PROTO_ERR_RANGE);
        return;
    }

    if (Stepper_IsBusy((uint8_t)axis))
    {
        ReplyErr(PROTO_ERR_BUSY);
        return;
    }

    if (relative)
    {
        /* 已回零时先做一次软限位预判，好把"超软限位"和"参数非法"区分开 */
        if (Stepper_IsHomed((uint8_t)axis))
        {
            float tgt = Stepper_GetPositionMM((uint8_t)axis) + dist;
            if (tgt < Stepper_GetSoftMin((uint8_t)axis) ||
                tgt > Stepper_GetSoftMax((uint8_t)axis))
            {
                ReplyErr(PROTO_ERR_SOFTLIMIT);
                return;
            }
        }
        ok = Stepper_MoveRel((uint8_t)axis, dist, speed);
    }
    else
    {
        ok = Stepper_MoveAbs((uint8_t)axis, dist, speed);
    }

    if (ok)
        ReplyOK();
    else
        ReplyErr(PROTO_ERR_SOFTLIMIT);
}

static void CmdMov(char *args)
{
    MoveCommon(args, 1);
}

static void CmdMova(char *args)
{
    MoveCommon(args, 0);
}

/* --------------------------------------------------------------------------
 *  LINE X10 Y5 F20 —— 两轴联动斜线（挑战要求②）
 *  两轴同时启动、同时到达，合成进给速度为 F mm/s
 * -------------------------------------------------------------------------- */
static void CmdLine(char *args)
{
    char   *p = args;
    char   *end;
    float   dx = 0.0f;
    float   dy = 0.0f;
    float   feed = 20.0f;
    uint8_t got = 0;

    p = SkipSpace(p);

    if (*p == 'X')
    {
        p++;
        dx = strtof(p, &end);
        if (end == p)
        {
            ReplyErr(PROTO_ERR_SYNTAX);
            return;
        }
        p = end;
        got = 1;
    }

    p = SkipSpace(p);

    if (*p == 'Y')
    {
        p++;
        dy = strtof(p, &end);
        if (end == p)
        {
            ReplyErr(PROTO_ERR_SYNTAX);
            return;
        }
        p = end;
        got = 1;
    }

    if (!got)
    {
        ReplyErr(PROTO_ERR_SYNTAX);
        return;
    }

    if (ParseFloatAfter(p, 'F', &feed) == 0)
        feed = 20.0f;

    if (Stepper_IsAlarm(STEPPER_X) || Stepper_IsAlarm(STEPPER_Y))
    {
        ReplyErr(PROTO_ERR_ALARM);
        return;
    }

    if (Stepper_IsBusy(STEPPER_X) || Stepper_IsBusy(STEPPER_Y))
    {
        ReplyErr(PROTO_ERR_BUSY);
        return;
    }

    if (Stepper_MoveLine(dx, dy, feed))
        ReplyOK();
    else
        ReplyErr(PROTO_ERR_RANGE);
}

/* --------------------------------------------------------------------------
 *  G02 / G03 X-10 Y10 I-10 J0 F10 —— 圆弧插补（项目二 挑战要求②）
 *    I / J   圆心相对起点的偏移（必须给，且不能同时为 0）
 *    X / Y   终点相对起点的位移；X0 Y0 + I/J = 整圆
 *    F       切向进给速度 mm/s，默认 10
 *    G03 = 逆时针，G02 = 顺时针
 * -------------------------------------------------------------------------- */
static void ArcCommon(char *args, uint8_t ccw)
{
    char   *p = args;
    char   *end;
    float   dx = 0.0f, dy = 0.0f, i = 0.0f, j = 0.0f, feed = 10.0f;
    uint8_t got_ij = 0;

    p = SkipSpace(p);

    while (*p)
    {
        char key = *p;

        if (key != 'X' && key != 'Y' && key != 'I' && key != 'J')
            break;

        p++;
        if (key == 'X')      dx = strtof(p, &end);
        else if (key == 'Y') dy = strtof(p, &end);
        else if (key == 'I') { i = strtof(p, &end); got_ij = 1; }
        else                 { j = strtof(p, &end); got_ij = 1; }

        if (end == p)
        {
            ReplyErr(PROTO_ERR_SYNTAX);
            return;
        }
        p = SkipSpace(end);
    }

    if (!got_ij)
    {
        ReplyErr(PROTO_ERR_SYNTAX);       /* 圆弧必须给 I/J */
        return;
    }

    if (ParseFloatAfter(p, 'F', &feed) == 0)
        feed = 10.0f;

    if (i == 0.0f && j == 0.0f)
    {
        ReplyErr(PROTO_ERR_RANGE);
        return;
    }

    if (Stepper_IsAlarm(STEPPER_X) || Stepper_IsAlarm(STEPPER_Y))
    {
        ReplyErr(PROTO_ERR_ALARM);
        return;
    }

    if (Stepper_IsBusy(STEPPER_X) || Stepper_IsBusy(STEPPER_Y))
    {
        ReplyErr(PROTO_ERR_BUSY);
        return;
    }

    if (Stepper_MoveArc(dx, dy, i, j, feed, ccw))
        ReplyOK();
    else
        ReplyErr(PROTO_ERR_RANGE);        /* 半径非法 / 终点不在圆上 / 超软限位 */
}

static void CmdG02(char *args)
{
    ArcCommon(args, 0);
}

static void CmdG03(char *args)
{
    ArcCommon(args, 1);
}

static void CmdStop(char *args)
{
    char  *p = args;
    int8_t axis;

    /* 记录停止时刻，供 F12 曲线上标注（须在真正停车前打标记） */
    Stepper_TraceMarkStop();

    axis = ParseAxis(&p);
    if (axis < 0)
    {
        Stepper_StopAll();
    }
    else if (Stepper_IsAlarm((uint8_t)axis))
    {
        ReplyErr(PROTO_ERR_ALARM);
        return;
    }
    else
    {
        Stepper_Stop((uint8_t)axis);
    }

    ReplyOK();
}

static void CmdEstop(char *args)
{
    (void)args;

    /* 与硬件急停按钮走同一条路径：立即封脉冲、报警码 2、作废"已回零" */
    Stepper_EmergencyStop();
    ReplyOK();
}

static void CmdHome(char *args)
{
    char  *p = args;
    int8_t axis;

    axis = ParseAxis(&p);
    if (axis < 0)
    {
        ReplyErr(PROTO_ERR_SYNTAX);
        return;
    }

    if (Stepper_IsAlarm((uint8_t)axis))
    {
        ReplyErr(PROTO_ERR_ALARM);
        return;
    }

    if (Stepper_IsBusy((uint8_t)axis))
    {
        ReplyErr(PROTO_ERR_BUSY);
        return;
    }

    if (!Stepper_HomeStart((uint8_t)axis))
    {
        ReplyErr(PROTO_ERR_BUSY);
        return;
    }

    ReplyOK();
}

static void CmdReset(char *args)
{
    (void)args;

    Stepper_ResetAlarm(STEPPER_X);
    Stepper_ResetAlarm(STEPPER_Y);
    ReplyOK();
}

static void CmdTrace(char *args)
{
    char  *p = args;
    int8_t axis;

    axis = ParseAxis(&p);
    if (axis < 0)
    {
        ReplyErr(PROTO_ERR_SYNTAX);
        return;
    }

    if (Stepper_IsBusy((uint8_t)axis))
    {
        ReplyErr(PROTO_ERR_BUSY);
        return;
    }

    if (!Stepper_TraceArm((uint8_t)axis))
    {
        ReplyErr(PROTO_ERR_RANGE);
        return;
    }

    ReplyOK();
}

static void CmdDump(char *args)
{
    char     buf[64];
    int8_t   ax = Stepper_TraceAxis();
    uint16_t n  = Stepper_TraceCount();
    uint16_t stop = Stepper_TraceStopIndex();

    (void)args;

    if (ax < 0)
        n = 0;

    if (stop == 0xFFFFu)
        sprintf(buf, "<TR %c N=%u DT=1 STOP=-1>\r\n",
                (ax == STEPPER_Y) ? 'Y' : 'X', (unsigned)n);
    else
        sprintf(buf, "<TR %c N=%u DT=1 STOP=%u>\r\n",
                (ax == STEPPER_Y) ? 'Y' : 'X', (unsigned)n, (unsigned)stop);
    Send(buf);

    for (uint16_t i = 0; i < n; i++)
    {
        sprintf(buf, "%u\r\n", (unsigned)Stepper_TraceGet(i));
        Send(buf);
    }

    Send("<TREND>\r\n");
}

static void CmdGet(char *args)
{
    char  *p = args;
    char   out[120];
    char  *q;
    int8_t axis;

    axis = ParseAxis(&p);
    if (axis < 0)
    {
        ReplyErr(PROTO_ERR_SYNTAX);
        return;
    }

    q  = out;
    q += sprintf(q, "OK %c LEAD=", (axis == STEPPER_X) ? 'X' : 'Y');
    q = FmtF(q, Stepper_GetLead((uint8_t)axis), 3);
    q += sprintf(q, " PPR=%u", (unsigned)Stepper_GetPPR((uint8_t)axis));
    q += sprintf(q, " DIRINV=%u", (unsigned)Stepper_GetDirInv((uint8_t)axis));
    q += sprintf(q, " ACC=");
    q = FmtF(q, Stepper_GetAccel((uint8_t)axis), 1);
    q += sprintf(q, " VMAX=");
    q = FmtF(q, Stepper_GetVmax((uint8_t)axis), 2);
    q += sprintf(q, " SLMIN=");
    q = FmtF(q, Stepper_GetSoftMin((uint8_t)axis), 3);
    q += sprintf(q, " SLMAX=");
    q = FmtF(q, Stepper_GetSoftMax((uint8_t)axis), 3);
    q += sprintf(q, " HF=");
    q = FmtF(q, Stepper_GetHomeFast((uint8_t)axis), 2);
    q += sprintf(q, " HS=");
    q = FmtF(q, Stepper_GetHomeSlow((uint8_t)axis), 2);
    q += sprintf(q, " DELTA=");
    q = FmtF(q, Stepper_GetDelta((uint8_t)axis), 6);
    q += sprintf(q, "\r\n");

    Send(out);
}

/* SET X LEAD=5 PPR=1600 DIRINV=0 ACC=200 VMAX=50 SLMIN=0 SLMAX=240 HF=10 HS=1 */
static void CmdSet(char *args)
{
    char   *p = args;
    int8_t  axis;
    uint8_t changed = 0;
    uint8_t bad = 0;
    float   fval;

    axis = ParseAxis(&p);
    if (axis < 0)
    {
        ReplyErr(PROTO_ERR_SYNTAX);
        return;
    }

    if (Stepper_IsAlarm((uint8_t)axis))
    {
        ReplyErr(PROTO_ERR_ALARM);
        return;
    }

    if (Stepper_IsBusy((uint8_t)axis))
    {
        ReplyErr(PROTO_ERR_BUSY);
        return;
    }

    while (1)
    {
        char key[12];
        uint8_t klen = 0;
        char *q = SkipSpace(p);

        if (*q == '\0')
            break;

        /* 取键名（到 '=' 为止） */
        while (*q && *q != '=' && klen < sizeof(key) - 1)
            key[klen++] = *q++;
        key[klen] = '\0';

        if (*q != '=')
        {
            bad = 1;
            break;
        }
        q++;

        fval = strtof(q, &p);
        if (p == q)
        {
            bad = 1;
            break;
        }

        if (strcmp(key, "LEAD") == 0)
        {
            if (!Stepper_SetLead((uint8_t)axis, fval)) bad = 1;
            else changed = 1;
        }
        else if (strcmp(key, "PPR") == 0)
        {
            if (!Stepper_SetPPR((uint8_t)axis, (uint16_t)fval)) bad = 1;
            else changed = 1;
        }
        else if (strcmp(key, "DIRINV") == 0)
        {
            if (!Stepper_SetDirInv((uint8_t)axis, (uint8_t)fval)) bad = 1;
            else changed = 1;
        }
        else if (strcmp(key, "ACC") == 0)
        {
            if (!Stepper_SetAccel((uint8_t)axis, fval)) bad = 1;
            else changed = 1;
        }
        else if (strcmp(key, "VMAX") == 0)
        {
            if (!Stepper_SetVmax((uint8_t)axis, fval)) bad = 1;
            else changed = 1;
        }
        else if (strcmp(key, "SLMIN") == 0)
        {
            float mx = Stepper_GetSoftMax((uint8_t)axis);
            if (fval < 0.0f || fval >= mx) bad = 1;
            else
            {
                Stepper_SetSoftLimit((uint8_t)axis, fval, mx);
                changed = 1;
            }
        }
        else if (strcmp(key, "SLMAX") == 0)
        {
            float mn = Stepper_GetSoftMin((uint8_t)axis);
            if (fval > 250.0f || fval <= mn) bad = 1;
            else
            {
                Stepper_SetSoftLimit((uint8_t)axis, mn, fval);
                changed = 1;
            }
        }
        else if (strcmp(key, "HF") == 0)
        {
            if (!Stepper_SetHomeFast((uint8_t)axis, fval)) bad = 1;
            else changed = 1;
        }
        else if (strcmp(key, "HS") == 0)
        {
            if (!Stepper_SetHomeSlow((uint8_t)axis, fval)) bad = 1;
            else changed = 1;
        }
        else
        {
            bad = 1;
        }

        if (bad)
            break;
    }

    if (bad)
    {
        ReplyErr(PROTO_ERR_RANGE);
        return;
    }

    if (!changed)
    {
        ReplyErr(PROTO_ERR_SYNTAX);
        return;
    }

    ReplyOK();
}

static void CmdHelp(char *args)
{
    (void)args;
    Send("PING JOG MOV MOVA LINE G02 G03 STOP ESTOP HOME RESET SET GET TRACE DUMP\r\n");
}

/* --------------------------------------------------------------------------
 *  分派表：按首个词查表（项目三加 G00/G01 只需在此加一行）
 * -------------------------------------------------------------------------- */
typedef struct
{
    const char *name;
    void (*fn)(char *args);
} CmdEntry;

static const CmdEntry s_cmds[] =
{
    { "PING",  CmdPing  },
    { "JOG",   CmdJog   },
    { "MOV",   CmdMov   },
    { "MOVA",  CmdMova  },
    { "LINE",  CmdLine  },
    { "G02",   CmdG02   },
    { "G03",   CmdG03   },
    { "STOP",  CmdStop  },
    { "ESTOP", CmdEstop },
    { "HOME",  CmdHome  },
    { "RESET", CmdReset },
    { "SET",   CmdSet   },
    { "GET",   CmdGet   },
    { "TRACE", CmdTrace },
    { "DUMP",  CmdDump  },
    { "HELP",  CmdHelp  },
};

#define CMD_COUNT (sizeof(s_cmds) / sizeof(s_cmds[0]))

static void Dispatch(char *line)
{
    char *p;
    uint8_t i;

    p = SkipSpace(line);
    if (*p == '\0')
        return;

    /* 取首个词 */
    {
        char  word[12];
        uint8_t n = 0;
        char *q = p;

        while (*q && *q != ' ' && *q != '\t' && n < sizeof(word) - 1)
            word[n++] = *q++;
        word[n] = '\0';

        ToUpper(word);

        for (i = 0; i < CMD_COUNT; i++)
        {
            if (strcmp(word, s_cmds[i].name) == 0)
            {
                s_cmds[i].fn(q);
                return;
            }
        }
    }

    ReplyErr(PROTO_ERR_UNKNOWN);
}

/* --------------------------------------------------------------------------
 *  初始化与主循环
 * -------------------------------------------------------------------------- */
void Protocol_Init(void)
{
    s_rx_head = 0;
    s_rx_tail = 0;
    s_line_len = 0;
    s_status_last = HAL_GetTick();

    /* 串口中断优先级低于限位/急停：限位必须在 2 µs 内封脉冲 */
    HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);

    HAL_UART_Receive_IT(&huart1, (uint8_t *)&s_rx_byte, 1);
}

/* 接收中断：只入队，不做解析 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t next;

    if (huart->Instance != USART1)
        return;

    next = (uint16_t)((s_rx_head + 1u) % PROTO_RX_SIZE);
    if (next != s_rx_tail)              /* 满了就丢新字节，不阻塞中断 */
    {
        s_rx_buf[s_rx_head] = s_rx_byte;
        s_rx_head = next;
    }

    HAL_UART_Receive_IT(&huart1, (uint8_t *)&s_rx_byte, 1);
}

/* 溢出/噪声错误后必须重新武装接收，否则串口"死掉" */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
        return;

    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&s_rx_byte, 1);
}

void Protocol_Poll(void)
{
    /* 1. 组行 */
    while (s_rx_head != s_rx_tail)
    {
        uint8_t c = s_rx_buf[s_rx_tail];

        s_rx_tail = (uint16_t)((s_rx_tail + 1u) % PROTO_RX_SIZE);

        if (c == '\n')
        {
            s_line[s_line_len] = '\0';
            if (s_line_len > 0)
            {
                Stepper_NoteHostMessage();      /* 心跳/通信超时保护（F8） */
                Dispatch(s_line);
            }
            s_line_len = 0;
        }
        else if (c != '\r')
        {
            if (s_line_len < PROTO_LINE_MAX - 1)
                s_line[s_line_len++] = (char)c;
            else
                s_line_len = 0;                 /* 超长行丢弃 */
        }
    }

    /* 2. 100 ms 状态上报（F10） */
    if (HAL_GetTick() - s_status_last >= PROTO_STATUS_MS)
    {
        s_status_last = HAL_GetTick();
        SendStatus();
    }
}
