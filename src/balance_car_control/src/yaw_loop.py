#!/usr/bin/env python3
"""偏航（方向）环：把 target_yaw_rate 变成差动力矩 turn_tau。

与平衡环的关系：
    - 平衡环输出 balance_tau（两轮同向，保 pitch）
    - 偏航环输出 turn_tau（左右反向，改 heading）
    - WheelMixer: left = balance - turn, right = balance + turn

当前实现：Yaw 角速度 PD（框架版，后续可换为串级位置环）。
"""

from dataclasses import dataclass


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


@dataclass
class YawLoopConfig:
    """偏航环参数（effort 模式，输出单轮差动力矩 [N·m]）。

    kp_yaw [N·m / (rad/s)] —— 角速度误差 → 差动力矩
    kd_yaw [N·m / (rad/s²)] —— 角速度变化阻尼（可选，框架预留）
    max_turn_tau [N·m]    —— 差动力矩饱和，应小于 max_wheel_effort
    output_sign ±1.0      —— 符号与 URDF/IMU 约定不一致时翻转
    enabled               —— False 时 turn_tau 恒为 0
    yaw_cmd_activate_threshold [rad/s] —— |target_yaw_rate| 低于此值时不输出 turn_tau
        （站立/只调平衡时必须关断，否则 IMU 零偏会被当成“要转向”）
    yaw_rate_deadband [rad/s] —— 测量角速度死区，抑制陀螺噪声
    """

    kp_yaw: float = 0.02
    kd_yaw: float = 0.0
    max_turn_tau: float = 0.06
    output_sign: float = 1.0
    enabled: bool = True
    yaw_cmd_activate_threshold: float = 0.08
    yaw_rate_deadband: float = 0.05


class YawLoop:
    def __init__(self, config: YawLoopConfig):
        self.config = config
        self.last_turn_tau = 0.0
        self._last_yaw_rate = 0.0
        self._last_yaw_rate_initialized = False

    def reset(self):
        self.last_turn_tau = 0.0
        self._last_yaw_rate = 0.0
        self._last_yaw_rate_initialized = False

    def compute(
        self,
        target_yaw_rate: float,
        yaw_rate: float,
        dt: float,
    ) -> float:
        cfg = self.config
        if not cfg.enabled or dt <= 0.0:
            self.last_turn_tau = 0.0
            return 0.0

        # 无转向指令时完全关闭差动力矩，避免破坏左右对称平衡。
        if abs(target_yaw_rate) < cfg.yaw_cmd_activate_threshold:
            self.last_turn_tau = 0.0
            self._last_yaw_rate = yaw_rate
            self._last_yaw_rate_initialized = True
            return 0.0

        measured_rate = yaw_rate
        deadband = abs(cfg.yaw_rate_deadband)
        if abs(measured_rate) <= deadband:
            measured_rate = 0.0
        elif measured_rate > 0.0:
            measured_rate -= deadband
        else:
            measured_rate += deadband

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
