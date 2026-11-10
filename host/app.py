"""
app.py —— 上位机入口

运行：
    python app.py
或双击同目录下的 run.bat

依赖：
    pip install PySide6 pyserial
"""
import sys

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt

from ui_main import MainWindow


def main():
    # Qt6 已默认启用高 DPI 缩放，不需要再设 AA_EnableHighDpiScaling
    app = QApplication(sys.argv)
    app.setApplicationName("StepperHMI")

    # 浅色主题，保证与报告截图一致
    app.setStyle("Fusion")

    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
