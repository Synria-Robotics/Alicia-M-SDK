"""Shared helpers for example scripts."""

import sys


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


def add_port_argument(parser):
    """Add the common Alicia-M serial port option."""
    parser.add_argument(
        "--port",
        type=str,
        default="",
        help="Alicia-M serial port, for example COM37. Omit to auto-detect.",
    )
    return parser


class NonBlockingKeyReader:
    """跨平台非阻塞单键读取器。

    Windows 使用 msvcrt；Linux/macOS 临时切换终端模式，退出时恢复。
    """

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
        """非阻塞读取一个按键；没有按键时返回 None。"""
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
