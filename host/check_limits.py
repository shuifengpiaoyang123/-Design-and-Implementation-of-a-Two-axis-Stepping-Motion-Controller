"""
限位开关接线/极性检测（配合固件 100 ms 状态帧 <ST ... LIM=xxxx ...>）

用法：
    python check_limits.py            # 默认 COM7
    python check_limits.py COM5       # 指定端口

做三件事：
  1. 显示四个限位（X- X+ Y- Y+）和急停的实时电平
  2. 依次提示你遮挡每一个开关，记录遮挡前后的电平变化
  3. 根据结果判断：接线是否正确、开关是「高电平有效」还是「低电平有效」

判断依据：
  正常的光电开关，遮挡与不遮挡时该位一定不同。
  如果某一位在你遮挡/松开时都不变，就是那根线没接好（悬空会被上拉成 1）。
"""

import sys
import time
import queue
import threading

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM7"
BAUD = 115200

NAMES = [("X-", 0), ("X+", 1), ("Y-", 2), ("Y+", 3)]


class Link:
    def __init__(self, port, baud):
        self.s = serial.Serial(port, baud, timeout=0.05)
        self.q = queue.Queue()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        buf = b""
        while not self.stop.is_set():
            try:
                chunk = self.s.read(512)
            except Exception:
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.decode("utf-8", "replace").strip()
                if line:
                    self.q.put(line)

    def latest_status(self, wait=0.30):
        """丢掉积压，取最新一帧状态；返回 (lim_mask, estop) 或 None"""
        last = None
        t0 = time.time()
        while time.time() - t0 < wait:
            try:
                line = self.q.get(timeout=0.05)
            except queue.Empty:
                continue
            if line.startswith("<ST"):
                last = line
        return parse_status(last) if last else None

    def send(self, text):
        self.s.write((text + "\r\n").encode())

    def close(self):
        self.stop.set()
        time.sleep(0.1)
        self.s.close()


def parse_status(line):
    lim = 0
    estop = 0
    for field in line.strip("<>").split():
        if field.startswith("LIM="):
            try:
                lim = int(field[4:], 2)
            except ValueError:
                lim = 0
        elif field.startswith("ESTOP="):
            try:
                estop = int(field[6:])
            except ValueError:
                estop = 0
    return lim, estop


def bits(lim):
    return "".join(str((lim >> i) & 1) for i in range(4))


def main():
    print(f"打开 {PORT} @ {BAUD} ...")
    link = Link(PORT, BAUD)
    time.sleep(0.4)

    st = link.latest_status(0.5)
    if st is None:
        print("收不到状态帧：检查端口/波特率，或固件是否是协议版")
        link.close()
        return

    lim, estop = st
    print(f"\n当前电平：X-={lim & 1}  X+={(lim >> 1) & 1}  Y-={(lim >> 2) & 1}  "
          f"Y+={(lim >> 3) & 1}  ESTOP={estop}")
    print("（上面这 5 个值不在状态帧里会显示 0；正常待命时通常全是 0 或全是 1，"
          "取决于开关是常开还是常闭）\n")

    if lim == 0 and estop == 0:
        baseline_desc = "全 0"
    elif lim == 0b1111:
        baseline_desc = "全 1"
    else:
        baseline_desc = "混合"
    print(f"静止基线：{bits(lim)}（{baseline_desc}）")
    if baseline_desc == "混合":
        print("⚠ 混合状态说明至少有一个开关处于触发状态，先排除是否有东西挡着光\n")

    result = {}
    for name, idx in NAMES:
        input(f"请【用手或挡片遮住 {name} 限位开关】，遮好后按回车 ...")
        st = link.latest_status(0.45)
        lim_on = st[0]
        input(f"   -> 读到的电平：{bits(lim_on)}（{name} = {(lim_on >> idx) & 1}）\n"
              f"   现在【松开 {name}】，松开后按回车 ...")
        st = link.latest_status(0.45)
        lim_off = st[0]
        print(f"   -> 松开后电平：{bits(lim_off)}（{name} = {(lim_off >> idx) & 1}）\n")
        result[name] = ((lim_on >> idx) & 1, (lim_off >> idx) & 1)

    print("\n================ 结论 ================")
    ok = True
    for name, (on, off) in result.items():
        if on == off:
            ok = False
            print(f"  ✗ {name}：遮挡和松开都是 {on} → 这个开关的线没接好/没供电，"
                  f"或接错引脚")
        else:
            kind = "高电平有效（遮挡=1）" if on == 1 else "低电平有效（遮挡=0）"
            print(f"  ✓ {name}：遮挡={on} 松开={off} → {kind}")

    if ok:
        on_vals = [v[0] for v in result.values()]
        if all(v == 1 for v in on_vals):
            print("\n四个开关遮挡时都是 1 → 固件当前判定（高电平=触发）是对的，"
                  "可以直接开 LIMITS_ENABLE=1")
        elif all(v == 0 for v in on_vals):
            print("\n四个开关遮挡时都是 0 → 极性与固件相反，需要把限位判定改成低有效，"
                  "并把 EXTI 改成下降沿触发。告诉我，我改固件。")
        else:
            print("\n四个开关极性不一致，需要逐个确认（可能混用了常开/常闭型号）")
    else:
        print("\n先把显示 ✗ 的那几根线接好再看。检查：开关 V+/GND 是否供电、"
              "信号线是否接到对应引脚、共地是否连上。")

    link.close()


if __name__ == "__main__":
    main()
