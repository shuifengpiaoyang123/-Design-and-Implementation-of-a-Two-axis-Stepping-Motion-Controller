"""arc_sim.py —— 圆弧插补仿真（与固件同构，不接硬件）

目的：在动固件之前，先用纯 Python 验证 DDA 圆弧的
      轨迹精度、速度平稳性、终点误差，并与逐点比较法对照。

模型与固件严格一致：
  δ      = 导程 / 每转脉冲数          stepper.c: Stepper_Recompute
  K      = v · 2^32 / (δ · ft)         stepper.c: Stepper_SpeedToIncrement
  节拍   = 20 µs（50 kHz）             TIM2
  规划   = 1 ms                        SysTick
  出脉冲 = 相位累加溢出                Stepper_Axis_Tick
  K 上限 = 2^31-1 → 输出 ≤ 25 kHz      INCREMENT_MAX

两套算法：
  DDA        —— 切向速度分解到两轴，各自累加
  逐点比较法 —— 每步只走一个轴，按偏差函数决定走哪个

用法：
  python sim/arc_sim.py
  python sim/arc_sim.py --r 20 --feed 15 --acc 300
"""

import argparse
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------------------------------------------------------------- 机械参数
LEAD_MM = 5.0
PPR = 1600
DELTA = LEAD_MM / PPR                 # 0.003125 mm/脉冲
FT = 50000                            # 50 kHz 节拍
TP = 0.001                            # 1 ms 规划周期
K_GAIN = (1 << 32) / (DELTA * FT)     # 2.7488e7
INC_MAX = 0x7FFFFFFF
TICKS_PER_MS = 50
VMAX = 25000.0 * DELTA                # 78.125 mm/s


# ---------------------------------------------------------------- 轴对象
class Axis:
    """对应 stepper.c 的 StepperAxis（只保留仿真需要的字段）"""

    def __init__(self):
        self.phase = 0
        self.increment = 0
        self.position = 0          # 脉冲
        self.direction = 1
        self.start_wait = 0
        self.pulse_lost = 0        # 统计：因钳位丢的拍数（正常情况下为 0）

    def set_speed(self, v_mm_s):
        v = abs(v_mm_s)
        if v <= 0.0:
            self.increment = 0
            return
        k = int(v * K_GAIN)
        if k > INC_MAX:
            k = INC_MAX
            self.pulse_lost += 1
        self.increment = k

    def tick(self):
        """一个 50 kHz 节拍：只负责发脉冲"""
        if self.start_wait > 0:
            self.start_wait -= 1
            return 0
        if self.increment == 0:
            return 0
        old = self.phase
        self.phase = (self.phase + self.increment) & 0xFFFFFFFF
        if self.phase < old:
            self.position += self.direction
            return 1
        return 0

    def mm(self):
        return self.position * DELTA


def round_pulse(mm):
    x = mm / DELTA
    return int(x + 0.5) if x >= 0 else int(x - 0.5)


# ---------------------------------------------------------------- DDA 圆弧
def run_dda(radius, start_deg, sweep_deg, feed, accel, ccw=True, dir_wait=50):
    """圆心在原点，从 start_deg 扫 sweep_deg。返回 (采样列表, 统计字典)

    注意坐标约定：轴对象的位置是"相对起点的位移"，而半径误差要按"离圆心的
    绝对距离"算，所以必须加起点偏移 ox/oy。第一版忘了这个偏移，
    结果半径误差算出 10 mm —— 这是最容易犯的坐标原点错误。
    """
    ax, ay = Axis(), Axis()
    theta0 = math.radians(start_deg)
    theta = theta0
    sweep = math.radians(sweep_deg)
    done = 0.0
    direction = 1 if ccw else -1

    ox = radius * math.cos(theta0)      # 起点在圆心坐标系下的绝对位置
    oy = radius * math.sin(theta0)

    alpha = accel / radius              # 角加速度 rad/s²
    omega_cmd = feed / radius           # 目标角速度 rad/s
    omega = 0.0

    samples = []                        # (t_ms, x_mm, y_mm, v_arc)
    t_ms = 0
    max_ms = int(400000 / TICKS_PER_MS)  # 400 s 上限，防死循环
    arc_len = 0.0
    last_x, last_y = ox + ax.mm(), oy + ay.mm()
    v_min, v_max = 1e9, 0.0
    flip_count = 0

    while t_ms < max_ms:
        # ---- 1 ms 规划（对应 Stepper_Planner_1ms 的圆弧分支）
        remain = sweep - done
        if remain <= 0.0 and omega < alpha * TP:
            break

        omega_brake = math.sqrt(2.0 * alpha * max(remain, 0.0))
        omega_cmd_eff = min(omega_cmd, omega_brake)

        if omega < omega_cmd_eff:
            omega += alpha * TP
            if omega > omega_cmd_eff:
                omega = omega_cmd_eff
        else:
            omega = omega_cmd_eff

        dth = omega * TP
        if dth >= remain:
            dth = remain
        theta += direction * dth
        done += dth

        # 切向速度 = 角速度 × 半径（不能用指令 F，否则起步瞬间会跳速）
        # 角度取本毫秒的"中点"：用末端角（欧拉法）会让轨迹持续外扩，
        # 实测 R=10 走一圈能外扩 10 µm，改用中点角后降到量化噪声量级
        theta_mid = theta - direction * dth * 0.5
        v = omega * radius
        vx = -v * math.sin(theta_mid) * direction
        vy = v * math.cos(theta_mid) * direction

        # ---- 换向：该轴速度过零时才允许（圆弧上正切点）
        for a, vv in ((ax, vx), (ay, vy)):
            want = 1 if vv >= 0 else -1
            if want != a.direction and abs(vv) < 1.0:
                a.direction = want
                a.start_wait = dir_wait      # 方向建立时间（50 拍 = 1 ms）
                flip_count += 1

        ax.set_speed(vx)
        ay.set_speed(vy)

        # ---- 50 个节拍
        for _ in range(TICKS_PER_MS):
            ax.tick()
            ay.tick()
        t_ms += 1

        x, y = ox + ax.mm(), oy + ay.mm()
        d = math.hypot(x - last_x, y - last_y)
        arc_len += d
        last_x, last_y = x, y
        samples.append((t_ms, x, y, v))

    # ---- 收尾：把不足一个脉冲的残量补足（固件里等价于一次 MoveAbs 到终点）
    tx = round_pulse(radius * math.cos(theta) - ox)
    ty = round_pulse(radius * math.sin(theta) - oy)
    settle_ms = 0
    while (ax.position != tx or ay.position != ty) and settle_ms < 500:
        for a, t in ((ax, tx), (ay, ty)):
            rem = abs(t - a.position)
            if rem == 0:
                a.increment = 0
                continue
            a.direction = 1 if t > a.position else -1
            v = math.sqrt(2.0 * accel * rem * DELTA)
            a.set_speed(min(v, VMAX))
        for _ in range(TICKS_PER_MS):
            ax.tick()
            ay.tick()
        settle_ms += 1

    # ---- 统计
    errs = [math.hypot(x, y) - radius for _, x, y, _ in samples]
    # 速度波动只统计匀速段（去掉起停的 10%），否则会被斜坡淹没
    n = len(samples)
    mid = samples[int(n * 0.10):int(n * 0.90)]
    vmin = min(p[3] for p in mid) if mid else 0.0
    vmax = max(p[3] for p in mid) if mid else 0.0
    stats = {
        "name": "DDA",
        "ms": t_ms,
        "pulse_x": ax.position,
        "pulse_y": ay.position,
        "arc_len": arc_len,
        "err_max": max(abs(e) for e in errs),
        "err_mean": sum(abs(e) for e in errs) / len(errs),
        "v_min": vmin,
        "v_max": vmax,
        "flips": flip_count,
        "end_err": math.hypot(ox + ax.mm() - radius * math.cos(theta),
                              oy + ay.mm() - radius * math.sin(theta)),
        "settle_ms": settle_ms,
        "clamp": ax.pulse_lost + ay.pulse_lost,
        "samples": samples,
        "errs": errs,
    }
    return stats


# ---------------------------------------------------------------- 逐点比较法
def run_pole(radius, start_deg, sweep_deg, feed, ccw=True):
    """逐点比较法：每步只走一个轴，按偏差函数 F 决定走哪个方向。

    四象限规则（逆时针）：
        Q1  F>=0 → -x   F<0 → +y      Q2  F>=0 → -y   F<0 → -x
        Q3  F>=0 → +x   F<0 → -y      Q4  F>=0 → +y   F<0 → +x
    顺时针把"两个方向"整体取反即可。

    步进速率固定（对应固件里"不能动态改 50 kHz 节拍"的硬约束），
    所以弧长速度 = 步长在切线上的投影 × 步频，天然在 0.707F~F 之间波动。
    """
    rp = radius / DELTA                       # 半径，脉冲
    x = int(round(rp * math.cos(math.radians(start_deg))))
    y = int(round(rp * math.sin(math.radians(start_deg))))
    R2 = int(round(rp)) ** 2
    F = x * x + y * y - R2

    step_period = DELTA / feed                # s
    tick_per_step = step_period * FT          # 拍/步（允许小数）
    acc = tick_per_step

    CCW = {0: ((-1, 0), (0, 1)),
           1: ((0, -1), (-1, 0)),
           2: ((1, 0), (0, -1)),
           3: ((0, 1), (1, 0))}
    CW = {q: tuple((-dx, -dy) for (dx, dy) in v) for q, v in CCW.items()}
    table = CCW if ccw else CW

    samples = []
    steps = 0
    t_ticks = 0
    ms_adv = 0.0
    arc_len = 0.0
    swept = 0.0                               # 已扫过的圆心角（弧长/半径）
    sweep = abs(math.radians(sweep_deg))
    max_steps = int(sweep * rp * 3) + 4000     # 逐点比较要走 1/0.85 倍步数，留 3 倍余量
    v_series = []
    proj_list = []

    while steps < max_steps and swept < sweep:
        t_ticks += 1
        acc += 1.0

        if acc >= tick_per_step:
            acc -= tick_per_step
            q = 0 if (x > 0 and y >= 0) else 1 if (x <= 0 and y > 0) \
                else 2 if (x < 0 and y <= 0) else 3
            dx, dy = table[q][0 if F >= 0 else 1]

            if dx == -1:
                F = F - 2 * x + 1
                x -= 1
            elif dx == 1:
                F = F + 2 * x + 1
                x += 1
            elif dy == 1:
                F = F + 2 * y + 1
                y += 1
            else:
                F = F - 2 * y + 1
                y -= 1

            steps += 1
            ang = math.atan2(y, x)
            tan = (-math.sin(ang), math.cos(ang)) if ccw else (math.sin(ang), -math.cos(ang))
            proj = abs(dx * tan[0] + dy * tan[1])
            proj_list.append(proj)
            adv = proj * DELTA
            ms_adv += adv
            swept += adv / radius
            arc_len += DELTA                  # 刀具实走弦长

        if t_ticks % TICKS_PER_MS == 0:
            samples.append((t_ticks / FT * 1000.0, x * DELTA, y * DELTA, 0.0))
            v_series.append(ms_adv / TP)
            ms_adv = 0.0

    t_ms = t_ticks / FT * 1000.0
    errs = [math.hypot(sx, sy) - radius for _, sx, sy, _ in samples]

    # 终点误差：理论终点与实到点
    te = math.radians(start_deg + (sweep_deg if ccw else -sweep_deg))
    end_err = math.hypot(x * DELTA - radius * math.cos(te),
                         y * DELTA - radius * math.sin(te))

    # 1 ms 窗口只有 3~4 步，量化噪声会盖住真实波动 → 用 20 ms 滑窗看趋势
    W = 20
    wv = [sum(v_series[i:i + W]) / W for i in range(0, max(1, len(v_series) - W), W)]
    mid = wv[int(len(wv) * 0.10):int(len(wv) * 0.90)] if len(wv) > 10 else wv
    stats = {
        "name": "逐点比较",
        "ms": t_ms,
        "steps": steps,
        "pulse_x": x,
        "arc_len": arc_len,
        "err_max": max(abs(e) for e in errs) if errs else 0,
        "err_mean": sum(abs(e) for e in errs) / len(errs) if errs else 0,
        "v_min": min(mid) if mid else 0,
        "v_max": max(mid) if mid else 0,
        "proj_lo": sorted(proj_list)[max(0, int(len(proj_list) * 0.02))] if proj_list else 0,
        "proj_hi": sorted(proj_list)[min(len(proj_list) - 1, int(len(proj_list) * 0.98))]
        if proj_list else 0,
        "proj_mean": sum(proj_list) / len(proj_list) if proj_list else 0,
        "flips": 0,
        "end_err": end_err,
        "settle_ms": 0,
        "clamp": 0,
        "steps_per_mm": steps / arc_len if arc_len else 0,
        "samples": samples,
        "errs": errs,
    }
    return stats


# ---------------------------------------------------------------- 输出
def report(r, feed, accel, dda, pole, sweep_deg):
    print("=" * 68)
    print(f"圆弧插补仿真   R={r} mm  F={feed} mm/s  A={accel} mm/s^2  "
          f"δ={DELTA:.6f} mm  上限={VMAX:.2f} mm/s")
    print("=" * 68)
    theory = abs(sweep_deg) / 360.0 * 2 * math.pi * r   # 实际弧长
    for s in (dda, pole):
        print(f"\n【{s['name']}】")
        if s["name"] == "DDA":
            print(f"  用时          {s['ms']} ms   收尾 {s['settle_ms']} ms")
            print(f"  脉冲数        X={s['pulse_x']}  Y={s['pulse_y']}")
        else:
            print(f"  用时          {s['ms']:.1f} ms   步数 {s['steps']}"
                  f"  （{s['steps_per_mm']:.0f} 步/mm，DDA 为 {1/DELTA:.0f} 脉冲/mm）")
        print(f"  刀具路径长    {s['arc_len']:.3f} mm  (理论 {theory:.3f} mm)")
        print(f"  半径误差      max {s['err_max']*1000:.1f} µm   "
              f"mean {s['err_mean']*1000:.1f} µm")
        print(f"  弧长速度      {s['v_min']:.3f} ~ {s['v_max']:.3f} mm/s"
              f"   (匀速段波动 {_pct(s['v_min'], s['v_max'])})")
        print(f"  终点误差      {s['end_err']*1000:.1f} µm")
        if s["name"] == "DDA":
            print(f"  换向次数      {s['flips']}    K 钳位拍数 {s['clamp']}")
        else:
            print(f"  单步弧长推进  {s['proj_lo']:.3f}·δ ~ {s['proj_hi']:.3f}·δ"
                  f"   平均 {s['proj_mean']:.3f}·δ"
                  f"  （多数步走满一个当量，每隔几十步插一步纯径向修正）")

    print("\n结论：")
    print(f"  进给平稳性   DDA {_pct(dda['v_min'], dda['v_max']):>8}   "
          f"逐点比较 {_pct(pole['v_min'], pole['v_max']):>8}   ← 直接决定表面质量")
    print(f"  几何精度     DDA {dda['err_max']*1000:5.1f} µm    "
          f"逐点比较 {pole['err_max']*1000:5.1f} µm    ← 逐点比较更贴圆")
    print(f"  额外行程     逐点比较比圆弧多走 {(pole['arc_len']/theory-1)*100:5.1f}%"
          f"（阶梯路径，费时费丝杠）")
    print(f"  同脉冲率用时 逐点比较 {pole['ms']/1000:.2f} s  vs  DDA {dda['ms']/1000:.2f} s"
          f"（慢 {(pole['ms']/dda['ms']-1)*100:.0f}%）")
    print(f"  一个脉冲当量 = {DELTA*1000:.1f} µm，"
          f"DDA 最大误差 = {dda['err_max']/DELTA:.2f} 个当量")


def _pct(lo, hi):
    return f"{(hi/lo-1)*100:.2f}%" if lo > 1e-9 else "n/a"


def plot(r, feed, dda, pole, out):
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False

    fig, axes = plt.subplots(1, 3, figsize=(15.5, 4.4))

    # ① 轨迹（只画前 90°，否则看不出细节）
    a0 = axes[0]
    th = [i * math.pi / 1800 for i in range(0, 1900)]
    a0.plot([r * math.cos(t) for t in th], [r * math.sin(t) for t in th],
            color="#B4B2A9", lw=2.2, label="理论圆")
    n = len(dda["samples"]) * 90 // 360
    a0.plot([p[1] for p in dda["samples"][:n]], [p[2] for p in dda["samples"][:n]],
            color="#185FA5", lw=1.3, label="DDA")
    pn = len(pole["samples"]) * 90 // 360
    a0.plot([p[1] for p in pole["samples"][:pn]], [p[2] for p in pole["samples"][:pn]],
            color="#993C1D", lw=0.9, ls="--", label="逐点比较")
    a0.set_aspect("equal")
    a0.set_xlim(-0.5, r * 1.08)
    a0.set_ylim(-0.5, r * 1.08)
    a0.set_title("轨迹（前 1/4 圆）", fontsize=12)
    a0.legend(fontsize=9, loc="lower right")
    a0.grid(alpha=0.25, lw=0.5)

    # ② 半径误差
    a1 = axes[1]
    a1.plot([p[0] for p in dda["samples"]], [e * 1000 for e in dda["errs"]],
            color="#185FA5", lw=1.0, label="DDA")
    pe = pole["errs"]
    a1.plot([p[0] for p in pole["samples"]], [e * 1000 for e in pe[:len(pole["samples"])]],
            color="#993C1D", lw=0.9, label="逐点比较")
    a1.axhline(0, color="#888780", lw=0.6)
    a1.set_xlabel("时间 ms")
    a1.set_ylabel("半径误差 µm")
    a1.set_title("半径误差", fontsize=12)
    a1.legend(fontsize=9)
    a1.grid(alpha=0.25, lw=0.5)

    # ③ 弧长速度
    a2 = axes[2]
    a2.plot([p[0] for p in dda["samples"]], [p[3] for p in dda["samples"]],
            color="#185FA5", lw=1.1, label="DDA")
    a2.axhline(feed, color="#B4B2A9", lw=0.8, ls=":")
    a2.axhline(feed * 0.707, color="#F0997B", lw=0.8, ls=":")
    a2.set_xlabel("时间 ms")
    a2.set_ylabel("弧长速度 mm/s")
    a2.set_title("合成进给速度（虚线 = F 与 0.707F）", fontsize=12)
    a2.grid(alpha=0.25, lw=0.5)

    fig.suptitle(f"DDA 圆弧插补验证  R={r} mm  F={feed} mm/s  A=200 mm/s²",
                 fontsize=13)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f"\n图已保存：{out}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--r", type=float, default=10.0)
    ap.add_argument("--feed", type=float, default=10.0)
    ap.add_argument("--acc", type=float, default=200.0)
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--sweep", type=float, default=360.0)
    ap.add_argument("--out", default="docs/arc_sim_result.png")
    a = ap.parse_args()

    dda = run_dda(a.r, a.start, a.sweep, a.feed, a.acc, ccw=True)
    pole = run_pole(a.r, a.start, a.sweep, a.feed, ccw=True)
    report(a.r, a.feed, a.acc, dda, pole, a.sweep)

    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    plot(a.r, a.feed, dda, pole, a.out)


if __name__ == "__main__":
    main()
