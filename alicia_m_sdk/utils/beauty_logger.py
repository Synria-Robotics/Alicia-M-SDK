# Copyright (c) 2025 Synria Robotics Co., Ltd.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
#
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

import os
from datetime import datetime
from typing import List, Any

# 定义日志级别常量
class LogLevel:
    """日志级别定义"""
    DEBUG = 0
    INFO = 1
    MODULE = 2
    WARNING = 3
    ERROR = 4
    SUCCESS = 5


class BeautyLogger:
    """
    Lightweight logger for Alicia-M-SDK package.
    """

    def __init__(self, log_dir: str, log_name: str = 'rofunc.log', verbose: bool = True, min_level: int = LogLevel.INFO):
        """
        Alicia-M-SDK轻量级日志器

        :param log_dir: 日志文件保存路径
        :param log_name: 日志文件名
        :param verbose: 是否在控制台打印日志
        :param min_level: 最小日志级别
        """
        self.log_dir = log_dir
        self.log_name = log_name
        self.log_path = os.path.join(self.log_dir, self.log_name)
        self.verbose = verbose
        self.min_level = min_level

        os.makedirs(self.log_dir, exist_ok=True)
        
    def _write_log(self, content, type):
        with open(self.log_path, "a", encoding="utf-8") as f:
            f.write(" Alicia-M-SDK:{}] {}\n".format(type.upper(), content))

    def _should_print(self, level: int) -> bool:
        """
        检查是否应该打印日志

        :param level: 要检查的日志级别
        :return: 是否应该打印
        """
        return self.verbose and level >= self.min_level

    def set_min_level(self, level: int):
        """
        设置最小日志级别

        :param level: 最小日志级别
        """
        if level < LogLevel.DEBUG or level > LogLevel.SUCCESS:
            raise ValueError("Invalid log level. Must be between LogLevel.DEBUG and LogLevel.SUCCESS")
        self.min_level = level

    def warning(self, content, local_verbose=True):
        """
        打印警告消息

        :param content: 警告消息内容
        :param local_verbose: 是否在控制台打印
        """
        if self._should_print(LogLevel.WARNING) and local_verbose:
            beauty_print(content, type="warning")
        self._write_log(content, type="warning")

    def module(self, content, local_verbose=True):
        """
        打印模块消息

        :param content: 模块消息内容
        :param local_verbose: 是否在控制台打印
        """
        if self._should_print(LogLevel.MODULE) and local_verbose:
            beauty_print(content, type="module")
        self._write_log(content, type="module")

    def info(self, content, local_verbose=True):
        """
        打印信息消息

        :param content: 信息消息内容
        :param local_verbose: 是否在控制台打印
        """
        if self._should_print(LogLevel.INFO) and local_verbose:
            beauty_print(content, type="info")
        self._write_log(content, type="info")

    def debug(self, content, local_verbose=True):
        """
        打印调试消息
        
        :param content: 调试消息内容
        :param local_verbose: 是否在控制台打印
        """
        if self._should_print(LogLevel.DEBUG) and local_verbose:
            beauty_print(content, type="debug")
        self._write_log(content, type="debug")

    def error(self, content, local_verbose=True):
        """
        打印错误消息
        
        :param content: 错误消息内容
        :param local_verbose: 是否在控制台打印
        """
        if self._should_print(LogLevel.ERROR) and local_verbose:
            beauty_print(content, type="error")
        self._write_log(content, type="error")
        raise Exception(content)

    def success(self, content, local_verbose=True):
        """
        打印成功消息
        
        :param content: 成功消息内容
        :param local_verbose: 是否在控制台打印
        """
        if self._should_print(LogLevel.SUCCESS) and local_verbose:
            beauty_print(content, type="success")
        self._write_log(content, type="success")


def beauty_print(content, type: str = None):
    """
    使用不同颜色打印内容

    :param content: 要打印的内容
    :param type: 支持 "warning", "module", "info", "error", "debug", "success"
    """
    if type is None:
        type = "info"
    if type == "warning":
        print("\033[1;37m [Alicia-M-SDK:WARNING] {}\033[0m".format(content))  # For warning (gray)
    elif type == "module":
        print("\033[1;33m [Alicia-M-SDK:MODULE] {}\033[0m".format(content))  # For a new module (light yellow)
    elif type == "info":
        print("\033[1;35m [Alicia-M-SDK:INFO] {}\033[0m".format(content))  # For info (light purple)
    elif type == "debug":
        print("\033[1;34m [Alicia-M-SDK:DEBUG] {}\033[0m".format(content))  # For debug (light blue)
    elif type == "error":
        print("\033[1;31m [Alicia-M-SDK:ERROR] {}\033[0m".format(content))  # For error (red)
    elif type == "success":
        print("\033[1;32m [Alicia-M-SDK:SUCCESS] {}\033[0m".format(content))  # For success (green)
    else:
        raise ValueError("Invalid level")


def hex_print(logger: BeautyLogger, title: str, data: List[int]):
    """
    print the data in hex format
    :param logger: the logger
    :param title: the title of the data
    :param data: the data to print
    :return: None
    """
    hex_buf = ' '.join(f"{b:02X}" for b in data)
    logger.info(f"{title}: {hex_buf}")


def beauty_print_matrix(name: str, data: Any, precision: int = 4, max_batch_items: int = 1, indent: int = 2):
    """
    Pretty print a scalar / vector / matrix / batch of matrices with RoboCore style.

    Automatically handles:
    - torch tensors (moved to cpu and converted to numpy)
    - numpy arrays / Python lists / scalars
    - Batch data (N, m, n) where m,n <= 6 treated as matrices batch

    :param name: label of the value
    :param data: value (scalar / 1D / 2D / 3D)
    :param precision: number of decimal places
    :param max_batch_items: number of batch entries to preview
    :param indent: left indentation (spaces)
    """
    # Lazy imports to avoid hard dependency if user does not need them
    try:
        import numpy as _np  # type: ignore
    except Exception:  # pragma: no cover
        _np = None  # type: ignore
    try:
        import torch as _torch  # type: ignore
    except Exception:  # pragma: no cover
        _torch = None  # type: ignore

    # Normalize input
    arr = data
    if _torch is not None and isinstance(arr, _torch.Tensor):
        arr = arr.detach().cpu().numpy()
    elif _np is not None and not isinstance(arr, (int, float)) and not isinstance(arr, str):
        if not isinstance(arr, _np.ndarray):
            try:
                arr = _np.array(arr)
            except Exception:
                pass

    # Simple scalar
    if isinstance(arr, (int, float)) or (hasattr(arr, "ndim") and getattr(arr, "ndim") == 0):
        print(" " * indent + f"{name} = {float(arr):.{precision}f}")

    # If still something unexpected, just print raw
    if not hasattr(arr, "ndim"):
        print(" " * indent + f"{name} = {arr}")

    ndim = arr.ndim  # type: ignore
    fmt = f"{{:>{precision + 6}.{precision}f}}"
    pad = " " * indent

    if ndim == 1:
        # Vector
        try:
            line = "  ".join(fmt.format(float(v)) for v in arr)
            print(pad + f"{name} = [{line}]")
        except Exception:
            print(pad + f"{name} = {arr}")
    elif ndim == 2:
        # Single matrix
        print(pad + f"{name} =")
        for row in arr:
            try:
                row_str = "  ".join(fmt.format(float(v)) for v in row)
            except Exception:
                row_str = "  ".join(str(v) for v in row)
            print(pad + "  [" + row_str + "]")
    elif ndim == 3:
        n = arr.shape[0]
        print(pad + f"{name} (batch size={n})")
        preview = min(max_batch_items, n)
        for bi in range(preview):
            if preview > 1:
                print(pad + f"  [item {bi}]")
            for row in arr[bi]:
                try:
                    row_str = "  ".join(fmt.format(float(v)) for v in row)
                except Exception:
                    row_str = "  ".join(str(v) for v in row)
                print(pad + "    [" + row_str + "]")
        if preview < n:
            print(pad + f"  ... ({n - preview} more)")
    else:
        # Higher dimension – fallback summary
        print(pad + f"{name} shape={getattr(arr, 'shape', '?')}")


def beauty_print_array(arr: Any, precision: int = 5, sign: bool = True) -> str:
    """Return a formatted string for 1D / 2D numeric arrays (numpy / torch / list).

    Behavior:
    - Scalars -> formatted with specified precision
    - 1D -> [ +0.12345, -0.12345, ... ]
    - 2D -> multi-line matrix style
    - Other shapes -> falls back to str(arr)

    :param arr: input data
    :param precision: decimal places
    :param sign: whether to always show sign
    :return: string
    """
    try:
        import numpy as _np  # type: ignore
    except Exception:  # pragma: no cover
        _np = None  # type: ignore
    try:
        import torch as _torch  # type: ignore
    except Exception:  # pragma: no cover
        _torch = None  # type: ignore

    # Normalize to numpy array when possible
    if _torch is not None and isinstance(arr, _torch.Tensor):
        arr = arr.detach().cpu().numpy()
    elif _np is not None:
        if not isinstance(arr, (int, float)) and not isinstance(arr, str):
            if not isinstance(arr, _np.ndarray):
                try:
                    arr = _np.array(arr)
                except Exception:
                    pass

    # Scalars
    if isinstance(arr, (int, float)):
        fmt = f"%{'+' if sign else ''}.{precision}f"
        return fmt % float(arr)

    if _np is None or not hasattr(arr, 'ndim'):
        return str(arr)

    if arr.ndim == 0:
        fmt = f"%{'+' if sign else ''}.{precision}f"
        return fmt % float(arr)

    number_fmt = f"{{:{'+' if sign else ''}.{precision}f}}"

    if arr.ndim == 1:
        values = ', '.join(number_fmt.format(float(x)) for x in arr)
        return f"[{values}]"
    elif arr.ndim == 2:
        lines = []
        for row in arr:
            row_str = '  '.join(number_fmt.format(float(x)) for x in row)
            lines.append(f"  [{row_str}]")
        return "[\n" + "\n".join(lines) + "\n]"
    else:
        return str(arr)


def print_info(msg: str) -> None:
    """Print info message.

    :param msg, str: Message content
    :return: None
    """
    beauty_print(msg, type="info")


def print_success(msg: str) -> None:
    """Print success message.

    :param msg, str: Message content
    :return: None
    """
    beauty_print(msg, type="success")


def print_warning(msg: str) -> None:
    """Print warning message.

    :param msg, str: Message content
    :return: None
    """
    beauty_print(msg, type="warning")


def print_error(msg: str) -> None:
    """Print error message.

    :param msg, str: Message content
    :return: None
    """
    beauty_print(msg, type="error")


def format_array(arr: Any, precision: int = 5, sign: bool = True) -> str:
    """Format numeric array with sign and precision.

    :param arr, Any: Numeric array-like object
    :param precision, int: Decimal places
    :param sign, bool: Whether to always show sign
    :return: Formatted string
    """
    return beauty_print_array(arr, precision=precision, sign=sign)


def get_logger(name: str = "") -> BeautyLogger:
    """Get sdk singleton logger instance.

    :param name, str: Reserved for compatibility
    :return: BeautyLogger singleton
    """
    return logger


def get_sdk_log_file_path() -> str:
    """Get current sdk log file path.

    :return: Log file path
    """
    return logger.log_path


_log_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
logger = BeautyLogger(
    log_dir="./logs",
    log_name=f"alicia_m_sdk_{_log_timestamp}.log",
    verbose=True,
    min_level=LogLevel.INFO,
)
