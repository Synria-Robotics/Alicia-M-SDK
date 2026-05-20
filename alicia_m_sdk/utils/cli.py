"""命令行示例工具。"""

from __future__ import annotations

import sys


def add_port_argument(parser):
    """@brief 为示例脚本添加通用串口参数。

    @param parser argparse.ArgumentParser 实例。
    @return 原 parser，便于链式调用。
    """
    parser.add_argument(
        "--port",
        type=str,
        default="",
        help="Alicia-M serial port, for example COM37. Omit to auto-detect.",
    )
    return parser


def select_waypoint_mode():
    """@brief 选择路点输入模式。

    @return 模式编号字符串："1" 手动录点，"2" 自动生成，"3" 从文件加载。
    """
    from .beauty_logger import beauty_print

    beauty_print("请选择路点来源：", type="module")
    print("  1) 手动录点")
    print("  2) 自动生成")
    print("  3) 从文件加载")
    while True:
        mode = input("输入模式编号 [1/2/3]: ").strip()
        if mode in {"1", "2", "3"}:
            return mode
        beauty_print("输入无效，请重新输入。", type="warning")


try:
    import msvcrt
except ImportError:
    msvcrt = None

if msvcrt is None:
    import select
    import termios
    import tty
else:
    select = None
    termios = None
    tty = None


class NonBlockingKeyReader:
    """@brief 跨平台非阻塞单键读取器。"""

    def __init__(self):
        self._original_termios = None

    def __enter__(self):
        if msvcrt is None and sys.stdin.isatty():
            self._original_termios = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type, exc, traceback):
        if msvcrt is None and self._original_termios is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._original_termios)
        return False

    def read_key(self):
        """@brief 非阻塞读取一个按键；没有按键时返回 None。"""
        if msvcrt is not None:
            if not msvcrt.kbhit():
                return None
            key = msvcrt.getwch()
            if key in ("\x00", "\xe0"):
                if msvcrt.kbhit():
                    msvcrt.getwch()
                return None
            return key.lower()

        if not sys.stdin.isatty():
            return None
        readable, _, _ = select.select([sys.stdin], [], [], 0)
        if not readable:
            return None
        return sys.stdin.read(1).lower()
