"""Apex SDK 异常类定义"""


class ApexError(Exception):
    """SDK 基础异常"""

    def __init__(self, message: str, code: int = 0):
        super().__init__(message)
        self.code = code
        self.message = message


class ConnectionError(ApexError):
    """连接异常（TCP 连接断开或超时）"""
    pass


class CommandError(ApexError):
    """命令执行失败（服务端返回 ok=false）"""
    pass


class TimeoutError(ApexError):
    """操作超时"""
    pass


class RealtimeError(ApexError):
    """实时通道异常"""
    pass
