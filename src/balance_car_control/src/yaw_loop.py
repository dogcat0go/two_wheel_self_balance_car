#!/usr/bin/env python3
"""偏航（方向）环：把 target_yaw_rate / 航向保持 变成差动力矩 turn_tau。

与平衡环的关系：
    - 平衡环输出 balance_tau（两轮同向，保 pitch）
    - 偏航环输出 turn_tau（左右反向，改 heading）
    - WheelMixer: left = balance - turn, right = balance + turn

结构：
    - 有转向指令：Yaw 角速度 PD（跟踪 cmd_vel.angular.z）
    - 无转向指令：锁定进入直行瞬间的 yaw，外环航向 → 内环角速度 PD
"""

import math
from dataclasses import dataclass


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_pi(angle: float) -> float:
    """把角度折到 (-π, π]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class YawLoopConfig:
    """偏航环参数（effort 模式，输出单轮差动力矩 [N·m]）。

    kp_yaw [N·m / (rad/s)] —— 角速度误差 → 差动力矩
    kd_yaw [N·m / (rad/s²)] —— 角速度变化阻尼（可选，框架预留）
    max_turn_tau [N·m]    —— 差动力矩饱和，应小于 max_wheel_effort
    output_sign ±1.0      —— 符号与 URDF/IMU 约定不一致时翻转
    enabled               —— False 时 turn_tau 恒为 0
    yaw_cmd_activate_threshold [rad/s] —— |target_yaw_rate| 低于此值视为直行
    yaw_rate_deadband [rad/s] —— 测量角速度死区，抑制陀螺噪声
    heading_hold_enabled  —— 直行时锁定航向角，抑制长时漂移
    kp_heading [1/s]      —— 航向外环：yaw 误差 → 期望 yaw_rate
    max_heading_rate [rad/s] —— 航向外环输出的角速度指令限幅
    """

    kp_yaw: float = 0.02
    kd_yaw: float = 0.0
    max_turn_tau: float = 0.06
    output_sign: float = 1.0
    enabled: bool = True
    yaw_cmd_activate_threshold: float = 0.08
    yaw_rate_deadband: float = 0.05
    heading_hold_enabled: bool = True
    kp_heading: float = 1.5
    max_heading_rate: float = 0.5


class YawLoop:
    def __init__(self, config: YawLoopConfig):
        self.config = config
        self.last_turn_tau = 0.0
        self._last_yaw_rate = 0.0
        self._last_yaw_rate_initialized = False
        self._yaw_ref = None

    def reset(self):
        self.last_turn_tau = 0.0
        self._last_yaw_rate = 0.0
        self._last_yaw_rate_initialized = False
        self._yaw_ref = None

    def _apply_rate_deadband(self, yaw_rate: float) -> float:
        deadband = abs(self.config.yaw_rate_deadband)
        if abs(yaw_rate) <= deadband:
            return 0.0
        if yaw_rate > 0.0:
            return yaw_rate - deadband
        return yaw_rate + deadband

    def _rate_to_turn_tau(
        self,
        target_yaw_rate: float,
        yaw_rate: float,
        dt: float,
    ) -> float:
        cfg = self.config
        measured_rate = self._apply_rate_deadband(yaw_rate)
        rate_error = target_yaw_rate - measured_rate

        d_yaw_rate = 0.0
        if self._last_yaw_rate_initialized:
            d_yaw_rate = (yaw_rate - self._last_yaw_rate) / dt
        self._last_yaw_rate = yaw_rate
        self._last_yaw_rate_initialized = True

        raw_turn = cfg.kp_yaw * rate_error - cfg.kd_yaw * d_yaw_rate
        limit = abs(cfg.max_turn_tau)
        turn_tau = clamp(cfg.output_sign * raw_turn, -limit, limit)
        self.last_turn_tau = turn_tau
        return turn_tau

    def compute(
        self,
        target_yaw_rate: float,
        yaw_rate: float,
        dt: float,
        yaw: float = 0.0,
    ) -> float:
        cfg = self.config
        if not cfg.enabled or dt <= 0.0:
            self.last_turn_tau = 0.0
            self._yaw_ref = None
            return 0.0

        turning = abs(target_yaw_rate) >= cfg.yaw_cmd_activate_threshold
        if turning:
            # 主动转向：松开航向锁，跟踪角速度指令
            self._yaw_ref = None
            return self._rate_to_turn_tau(target_yaw_rate, yaw_rate, dt)

        if not cfg.heading_hold_enabled:
            self.last_turn_tau = 0.0
            self._yaw_ref = None
            self._last_yaw_rate = yaw_rate
            self._last_yaw_rate_initialized = True
            return 0.0

        # 直行：进入瞬间锁定当前 yaw，外环生成期望角速度再走内环
        if self._yaw_ref is None:
            self._yaw_ref = yaw
        yaw_error = wrap_pi(self._yaw_ref - yaw)
        rate_cmd = clamp(
            cfg.kp_heading * yaw_error,
            -abs(cfg.max_heading_rate),
            abs(cfg.max_heading_rate),
        )
        return self._rate_to_turn_tau(rate_cmd, yaw_rate, dt)
