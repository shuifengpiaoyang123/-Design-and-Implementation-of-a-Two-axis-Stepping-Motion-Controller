"""
selftest.py —— 上位机协议层自测（不需要连接硬件）

运行： python selftest.py
覆盖：指令打包格式、<ST> 解析、应答解析、GET 解析、运动记录换算与加速度拟合
用途：不接板子也能确认协议层没问题，便于"上位机先行开发"（任务书 4.3.5 建议）
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
import protocol as proto

ok_cnt = 0
fail_cnt = 0


def check(name, cond, extra=""):
    global ok_cnt, fail_cnt
    if cond:
        ok_cnt += 1
        print(f"  [OK]   {name}")
    else:
        fail_cnt += 1
        print(f"  [FAIL] {name} {extra}")


def test_builders():
    print("1) 指令打包")
    check("PING", proto.cmd_ping() == "PING")
    check("JOG X+ F20", proto.cmd_jog("X", +1, 20) == "JOG X+ F20", proto.cmd_jog("X", 1, 20))
    check("JOG Y- F10.5", proto.cmd_jog("Y", -1, 10.5) == "JOG Y- F10.5", proto.cmd_jog("Y", -1, 10.5))
    check("MOV X-12.5 F20", proto.cmd_mov("X", -12.5, 20) == "MOV X-12.5 F20", proto.cmd_mov("X", -12.5, 20))
    check("MOVA X100 F30", proto.cmd_mova("X", 100, 30) == "MOVA X100 F30", proto.cmd_mova("X", 100, 30))
    check("LINE X10 Y10 F20", proto.cmd_line(10, 10, 20) == "LINE X10 Y10 F20",
          proto.cmd_line(10, 10, 20))
    check("LINE X-20 Y-5 F12.5", proto.cmd_line(-20, -5, 12.5) == "LINE X-20 Y-5 F12.5",
          proto.cmd_line(-20, -5, 12.5))
    check("LINE 纯 X 轴", proto.cmd_line(15, 0, 20) == "LINE X15 Y0 F20",
          proto.cmd_line(15, 0, 20))
    check("STOP X", proto.cmd_stop("X") == "STOP X")
    check("STOP", proto.cmd_stop() == "STOP")
    check("SET 全参数",
          proto.cmd_set("X", lead=5, ppr=1600, dirinv=0, acc=200, vmax=50,
                        slmin=0, slmax=240, hf=10, hs=1)
          == "SET X LEAD=5 PPR=1600 DIRINV=0 ACC=200 VMAX=50 SLMIN=0 SLMAX=240 HF=10 HS=1",
          proto.cmd_set("X", lead=5, ppr=1600, dirinv=0, acc=200, vmax=50,
                        slmin=0, slmax=240, hf=10, hs=1))


def test_status():
    print("2) 状态上报解析")
    line = "<ST X=12.500,25.00,RUN,H1 Y=0.000,-10.00,IDLE,H0 LIM=0100 ALM=0>"
    st = proto.parse_status(line)
    check("解析成功", st is not None)
    if st is None:
        return
    check("X 位置 12.500", abs(st.x.pos - 12.5) < 1e-6, st.x.pos)
    check("X 速度 25.00", abs(st.x.speed - 25.0) < 1e-6, st.x.speed)
    check("X 状态 RUN", st.x.state == "RUN", st.x.state)
    check("X 已回零", st.x.homed)
    check("Y 速度 -10", abs(st.y.speed + 10.0) < 1e-6, st.y.speed)
    check("Y 未回零", not st.y.homed)
    check("LIM=0100 → X+ 触发", st.limit_of("X", True) and not st.limit_of("X", False))
    check("LIM=0100 → Y 未触发", not st.limit_of("Y", True) and not st.limit_of("Y", False))
    check("X 运动中 busy", st.x.busy)
    check("Y 空闲 not busy", not st.y.busy)
    check("非法行返回 None", proto.parse_status("OK") is None)


def test_reply():
    print("3) 应答解析")
    check("OK", proto.parse_reply("OK") == ("ok", None))
    check("ERR,4", proto.parse_reply("ERR,4") == ("err", 4))
    check("PONG", proto.parse_reply("PONG") == ("pong", None))
    kind, data = proto.parse_reply("OK X LEAD=5.000 PPR=1600")
    check("OK 带数据", kind == "ok_data" and data.startswith("X LEAD"), (kind, data))
    check("错误码文案", proto.err_text(4) == "超出软限位", proto.err_text(4))

    gp = proto.parse_get("X LEAD=5.000 PPR=2000 DIRINV=1 ACC=150.0 VMAX=40.00 "
                         "SLMIN=0.000 SLMAX=240.000 HF=10.00 HS=1.00 DELTA=0.002500")
    check("GET 解析 PPR", gp and gp.ppr == 2000)
    check("GET 解析 DIRINV", gp and gp.dirinv == 1)
    check("GET 解析 DELTA", gp and abs(gp.delta - 0.0025) < 1e-9)


def test_trace():
    print("4) 运动记录换算与加速度拟合")
    lead, ppr = 5.0, 1600
    delta = config.pulse_equivalent(lead, ppr)
    check("脉冲当量 = 0.003125", abs(delta - 0.003125) < 1e-12, delta)

    # 造一条 50 mm @ 50 mm/s、a=200 的梯形曲线（1 ms 采样）
    a, vcmd, dist = 200.0, 50.0, 50.0
    t_acc = vcmd / a
    s_acc = 0.5 * a * t_acc ** 2
    s_cruise = dist - 2 * s_acc
    t_cruise = s_cruise / vcmd

    deltas = []
    t = 0.0
    while t < t_acc:
        v = a * t
        deltas.append(round(v * 0.001 / delta))
        t += 0.001
    for _ in range(int(t_cruise / 0.001)):
        deltas.append(round(vcmd * 0.001 / delta))
    t = 0.0
    while t < t_acc:
        v = vcmd - a * t
        if v <= 0:
            break
        deltas.append(round(v * 0.001 / delta))
        t += 0.001

    tt, v, x = proto.compute_trace(deltas, delta, 1)
    check("采样点数合理", 1200 <= len(v) <= 1300, len(v))
    check("最大速度 ≈ 50", abs(max(v) - 50) < 1.0, max(v))
    check("总行程 ≈ 50 mm", abs(x[-1] - 50) < 0.1, x[-1])

    acc_fit = proto.fit_acceleration(v, 1)
    err = abs(acc_fit - a) / a * 100
    check(f"拟合加速度 {acc_fit:.1f} 误差 {err:.1f}% < 5%", err < 5.0, acc_fit)

    hdr = proto.parse_trace_header("<TR X N=1234 DT=1 STOP=800>")
    check("记录头解析", hdr and hdr.count == 1234 and hdr.stop_index == 800)


def test_arc():
    print("6) 圆弧插补几何（项目二）")
    import arc

    # —— 半径/角度 → I/J/Δ ——
    g = arc.from_r_angle(10, 0, 360, True)
    check("整圆 I=-10 J=0", abs(g.i + 10) < 1e-9 and abs(g.j) < 1e-9, (g.i, g.j))
    check("整圆 ΔX=ΔY=0", abs(g.dx) < 1e-9 and abs(g.dy) < 1e-9, (g.dx, g.dy))
    check("整圆弧长 = 2πR", abs(g.arc_len - 2 * math.pi * 10) < 1e-9, g.arc_len)

    q = arc.from_r_angle(10, 0, 90, True)
    check("1/4 弧终点 = (0,10) 圆心系",
          abs(q.end_pt[0]) < 1e-9 and abs(q.end_pt[1] - 10) < 1e-9, q.end_pt)
    check("1/4 弧 Δ=(-10,+10)", abs(q.dx + 10) < 1e-9 and abs(q.dy - 10) < 1e-9, (q.dx, q.dy))
    check("1/4 弧长 = πR/2", abs(q.arc_len - math.pi * 10 / 2) < 1e-9, q.arc_len)

    # —— I/J + Δ 反推 ——
    back = arc.from_ij(q.i, q.j, q.dx, q.dy, True)
    check("反推扫角 90°", abs(back.sweep - q.sweep) < 1e-9, back.sweep)
    check("反推半径 10", abs(back.r - 10) < 1e-9, back.r)

    full = arc.from_ij(-10, 0, 0, 0, False)
    check("I/J 整圆顺时针 sweep=-2π", abs(full.sweep + 2 * math.pi) < 1e-9, full.sweep)

    # —— 终点不在圆上必须报错 ——
    try:
        arc.from_ij(-10, 0, -11.5, 10, True)     # 距圆心 √(2.25+100)=10.112，偏差 0.112 > 1%R
        check("终点不在圆上 → 报错", False)
    except ValueError:
        check("终点不在圆上 → 报错", True)

    # —— 顺时针归一化：扫角永远落在 (0, 2π] ——
    for ang in (30, 90, 200, 300):
        cw = arc.from_r_angle(10, ang, 120, False)
        check(f"顺时针起始 {ang}° 反推一致",
              abs(arc.from_ij(cw.i, cw.j, cw.dx, cw.dy, False).sweep - cw.sweep) < 1e-9)

    # —— 半径/角度生成到指令串 ——
    check("G03 指令串", proto.cmd_arc(*q.to_cmd_args(), 10, True)
          == "G03 X-10 Y10 I-10 J0 F10", proto.cmd_arc(*q.to_cmd_args(), 10, True))
    check("G02 整圆指令串", proto.cmd_arc(*g.to_cmd_args(), 10, False)
          == "G02 X0 Y0 I-10 J0 F10", proto.cmd_arc(*g.to_cmd_args(), 10, False))

    # —— 误差统计与进度 ——
    pts = [g.point_at(g.a0 + g.sweep * k / 100) for k in range(101)]
    rel = [(x - g.r, y) for x, y in pts]          # 圆心系 → 相对起点
    errs = arc.radius_errors(rel, g.i, g.j, g.r)
    check("理想圆误差 ≈ 0", max(abs(e) for e in errs) < 1e-9,
          max(abs(e) for e in errs))
    errs2 = arc.radius_errors([(p[0] + 0.001, p[1]) for p in rel], g.i, g.j, g.r)
    check("外扩 1 µm → 误差 +1 µm", abs(max(errs2) - 0.001) < 1e-6, max(errs2))
    prog = arc.sweep_progress(rel, g.i, g.j, g.a0, g.sweep)
    check("进度单调且收尾 = 1", all(prog[k] <= prog[k + 1] + 1e-9 for k in range(100))
          and abs(prog[-1] - 1.0) < 1e-9, prog[-1])

    # —— 时间估算 ——
    t_full = g.est_time(10, 200)
    check("整圆用时 ≈ 6.48 s", abs(t_full - (2 * math.pi * 10 / 10 + 10 / 200)) < 0.1, t_full)
    short = arc.from_r_angle(10, 0, 1, True)      # 1° 短弧（0.17 mm）→ 三角形退化
    check("短弧峰值 < F", short.peak_feed(10, 200) < 10, short.peak_feed(10, 200))


def test_vmax_limit():
    print("5) 细分与速度上限")
    check("1600 细分上限 ≈ 78.1", abs(config.vmax_limit(5, 1600) - 78.125) < 0.01,
          config.vmax_limit(5, 1600))
    check("6400 细分上限 ≈ 19.5", abs(config.vmax_limit(5, 6400) - 19.531) < 0.01,
          config.vmax_limit(5, 6400))


if __name__ == "__main__":
    test_builders()
    test_status()
    test_reply()
    test_trace()
    test_vmax_limit()
    test_arc()
    print()
    print(f"结果：{ok_cnt} 项通过，{fail_cnt} 项失败")
    sys.exit(0 if fail_cnt == 0 else 1)
