"""日志系统

SDK 内部使用标准 logging 模块分级输出，
面向用户的状态打印使用 robocore.utils.beauty_logger 提供格式化彩色输出。

日志级别约定：
    - DEBUG:   通信收发原始数据（默认不输出）
    - INFO:    一般操作信息
    - WARNING: 参数超限裁剪、非致命异常
    - ERROR:   操作失败、连接断开
"""

import logging
from typing import Union, List

# beauty_logger 延迟导入标记
_beauty_logger_available = None


def _ensure_beauty_logger():
    """延迟导入 beauty_logger，避免导入时立即报错

    Returns:
        (beauty_print, beauty_print_array) 函数元组，导入失败时返回 (None, None)
    """
    global _beauty_logger_available
    if _beauty_logger_available is not None:
        return _beauty_logger_available

    try:
        from robocore.utils.beauty_logger import beauty_print, beauty_print_array
        _beauty_logger_available = (beauty_print, beauty_print_array)
    except ImportError:
        _beauty_logger_available = (None, None)
    return _beauty_logger_available


def get_logger(name: str, level: int = logging.INFO) -> logging.Logger:
    """获取带格式化的 SDK 内部日志实例

    日志格式: [时间] [模块名] [级别] 消息

    如果该 logger 尚未配置 handler，则自动添加一个 StreamHandler
    并设置统一的格式化器。避免重复添加 handler。

    Args:
        name: 模块名称，通常传入 __name__
        level: 日志级别，默认 INFO

    Returns:
        配置好的 logging.Logger 实例
    """
    log = logging.getLogger(name)
    log.setLevel(level)

    # 避免重复添加 handler
    if not log.handlers:
        handler = logging.StreamHandler()
        handler.setLevel(level)
        formatter = logging.Formatter(
            fmt="[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s",
            datefmt="%H:%M:%S",
        )
        handler.setFormatter(formatter)
        log.addHandler(handler)
        # 阻止日志向上传播到 root logger，避免重复输出
        log.propagate = False

    return log


# ============================================================
# beauty_logger 转发函数（供 API 层和示例统一调用）
# ============================================================

def print_info(msg: str) -> None:
    """输出信息提示（紫色）

    转发到 beauty_print(msg, type="info")。
    beauty_logger 不可用时回退到 print。

    Args:
        msg: 信息内容
    """
    bp, _ = _ensure_beauty_logger()
    if bp is not None:
        bp(msg, type="info")
    else:
        print("[INFO]", msg)


def print_success(msg: str) -> None:
    """输出成功提示（绿色）

    转发到 beauty_print(msg, type="success")。
    beauty_logger 不可用时回退到 print。

    Args:
        msg: 信息内容
    """
    bp, _ = _ensure_beauty_logger()
    if bp is not None:
        bp(msg, type="success")
    else:
        print("[SUCCESS]", msg)


def print_warning(msg: str) -> None:
    """输出警告提示（灰色）

    转发到 beauty_print(msg, type="warning")。
    beauty_logger 不可用时回退到 print。

    Args:
        msg: 信息内容
    """
    bp, _ = _ensure_beauty_logger()
    if bp is not None:
        bp(msg, type="warning")
    else:
        print("[WARNING]", msg)


def print_error(msg: str) -> None:
    """输出错误提示（红色）

    转发到 beauty_print(msg, type="error")。
    beauty_logger 不可用时回退到 print。

    Args:
        msg: 信息内容
    """
    bp, _ = _ensure_beauty_logger()
    if bp is not None:
        bp(msg, type="error")
    else:
        print("[ERROR]", msg)


def format_array(
    arr: Union[List[float], tuple],
    precision: int = 5,
    sign: bool = True,
) -> str:
    """格式化数组输出

    转发到 beauty_print_array(arr, precision, sign)。
    beauty_logger 不可用时使用内置格式化。

    Args:
        arr: 待格式化的数组
        precision: 小数位数，默认 5
        sign: 是否显示正号，默认 True

    Returns:
        格式化后的字符串
    """
    _, bpa = _ensure_beauty_logger()
    if bpa is not None:
        return bpa(arr, precision=precision, sign=sign)

    # 回退：内置简易格式化
    fmt = "{:+.%df}" % precision if sign else "{:.%df}" % precision
    items = [fmt.format(float(v)) for v in arr]
    return "[" + ", ".join(items) + "]"
