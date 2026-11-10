"""
serial_link.py —— 串口通信层（独立线程、按行组帧）
任务书 4.3.5：串口通信要用独立线程或异步方式收发、按行组帧，不得阻塞界面。

设计要点：
  · 收发全在这个 QThread 里做，界面线程只发信号/塞队列
  · 每收到一行就带上毫秒时间戳抛给界面（U7 通信记录要用）
  · 发送走队列，避免界面线程直接碰串口对象
"""
import time
import queue
from typing import List, Tuple

import serial
from serial.tools import list_ports
from PySide6.QtCore import QThread, Signal


def list_serial_ports() -> List[Tuple[str, str]]:
    """返回 [(端口号, 描述), ...]"""
    out = []
    for p in list_ports.comports():
        desc = p.description or ""
        out.append((p.device, desc))
    out.sort(key=lambda x: x[0])
    return out


class SerialWorker(QThread):
    line_received = Signal(str, float)   # 一行文本, 毫秒时间戳
    opened = Signal()
    closed = Signal()
    failed = Signal(str)

    def __init__(self, port: str, baud: int, parent=None):
        super().__init__(parent)
        self._port = port
        self._baud = baud
        self._ser = None
        self._stop = False
        self._tx = queue.Queue()
        self._buf = bytearray()

    # ---------------- 对外接口（界面线程调用） ----------------
    def send(self, text: str):
        self._tx.put(text)

    def stop(self):
        self._stop = True

    # ---------------- 线程体 ----------------
    def run(self):
        try:
            self._ser = serial.Serial(
                self._port, self._baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.02,
                write_timeout=0.5,
            )
        except Exception as e:                      # noqa: BLE001
            self.failed.emit(f"打开 {self._port} 失败：{e}")
            return

        self.opened.emit()

        try:
            while not self._stop:
                # 1) 发送队列
                while True:
                    try:
                        text = self._tx.get_nowait()
                    except queue.Empty:
                        break
                    if not text.endswith("\n"):
                        text += "\r\n"
                    self._ser.write(text.encode("ascii", errors="ignore"))

                # 2) 接收
                n = self._ser.in_waiting
                data = self._ser.read(n if n > 0 else 1)
                if not data:
                    continue

                self._buf.extend(data)
                while b"\n" in self._buf:
                    raw, _, rest = bytes(self._buf).partition(b"\n")
                    self._buf = bytearray(rest)
                    line = raw.decode("ascii", errors="ignore").strip("\r\n ")
                    if line:
                        self.line_received.emit(line, time.time() * 1000.0)

                # 防止异常数据把缓冲撑爆
                if len(self._buf) > 4096:
                    self._buf.clear()
        except Exception as e:                      # noqa: BLE001
            self.failed.emit(f"串口异常：{e}")
        finally:
            try:
                if self._ser and self._ser.is_open:
                    self._ser.close()
            except Exception:                       # noqa: BLE001
                pass
            self.closed.emit()
