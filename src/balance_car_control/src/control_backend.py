#!/usr/bin/env python3
"""平衡控制策略抽象接口（对应 ros2_control 的 ControllerInterface 思路）。

节点只依赖 BalanceControlBackend，具体算法由 pid / lqr 等实现类提供。
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional

from balance_types import BalanceState, MotionCommand


def declare_param(node, name, default_value):
    """声明参数；若 node 已声明则跳过（避免 backend 与 node 重复 declare）。"""
    if not node.has_parameter(name):
        node.declare_parameter(name, default_value)


@dataclass
class BalanceControlOutput:
    """单次控制周期 backend 的输出。"""

    balance_tau: float
    pitch_target: float = 0.0
    velocity_setpoint: float = 0.0
    target_wheel_velocity: float = 0.0
    x_target: float = 0.0
    extra: dict = field(default_factory=dict)


class BalanceControlBackend(ABC):
    """平衡控制策略接口。"""

    @property
    @abstractmethod
    def mode_name(self) -> str:
        """策略名称，如 'pid' / 'lqr'。"""

    @abstractmethod
    def reset(self, state: Optional[BalanceState] = None) -> None:
        """安全停车 / 失能 / 状态失效时复位内部状态。"""

    @abstractmethod
    def compute(
        self,
        state: BalanceState,
        target_wheel_velocity: float,
        dt: float,
        enabled: bool,
    ) -> BalanceControlOutput:
        """根据当前状态与速度指令，返回单轮纵向力矩 [N·m]。"""

    def log_config(self, logger) -> None:
        """启动时打印本策略相关配置（可选）。"""


def create_balance_backend(mode: str, node) -> BalanceControlBackend:
    """按 control_mode 实例化对应 backend。"""
    from cascade_pid_backend import CascadePidBackend
    from lqr_backend import LqrBackend

    normalized = mode.strip().lower()
    if normalized == "pid":
        return CascadePidBackend.from_node(node)
    if normalized == "lqr":
        return LqrBackend.from_node(node)
    raise ValueError(
        "unknown control_mode: {!r} (expected 'pid' or 'lqr')".format(mode)
    )
