#!/usr/bin/env python3
"""运动指令适配：把 ROS /cmd_vel 转成控制器内部用的 MotionCommand。

职责边界（框架层）：
    - 订阅 geometry_msgs/Twist，做单位换算与限幅
    - 超时后自动归零（安全默认）
    - 不直接参与控制律计算

单位约定：
    cmd_vel.linear.x   [m/s]   → target_wheel_velocity [rad/s] = v / wheel_radius
    cmd_vel.angular.z  [rad/s] → target_yaw_rate       [rad/s]（直接使用）
"""

from dataclasses import dataclass

from balance_types import MotionCommand


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


@dataclass
class MotionCommandConfig:
    wheel_radius: float = 0.0325          # [m] 与 URDF 一致
    max_linear_velocity: float = 0.5      # [m/s] cmd_vel.linear.x 限幅
    max_angular_velocity: float = 1.5     # [rad/s] cmd_vel.angular.z 限幅
    max_wheel_velocity: float = 15.0        # [rad/s] 换算后轮速硬限幅
    cmd_vel_timeout_sec: float = 0.5        # [s] 无新指令则归零
    enabled: bool = True                    # False 时始终输出零指令


class MotionCommandAdapter:
    """维护最近一次有效 /cmd_vel，并在控制周期查询。"""

    def __init__(self, config: MotionCommandConfig):
        self.config = config
        self._command = MotionCommand()

    def reset(self):
        self._command = MotionCommand()

    def update_from_twist(self, linear_x: float, angular_z: float, stamp_sec: float):
        cfg = self.config
        if not cfg.enabled:
            self.reset()
            return

        v = clamp(linear_x, -cfg.max_linear_velocity, cfg.max_linear_velocity)
        w = clamp(angular_z, -cfg.max_angular_velocity, cfg.max_angular_velocity)

        wheel_v = v / cfg.wheel_radius if cfg.wheel_radius > 0.0 else 0.0
        wheel_v = clamp(wheel_v, -cfg.max_wheel_velocity, cfg.max_wheel_velocity)

        self._command = MotionCommand(
            target_wheel_velocity=wheel_v,
            target_yaw_rate=w,
            stamp_sec=stamp_sec,
        )

    def current_command(self, now_sec: float) -> MotionCommand:
        cfg = self.config
        if not cfg.enabled:
            return MotionCommand()

        age = now_sec - self._command.stamp_sec
        if self._command.stamp_sec <= 0.0 or age > cfg.cmd_vel_timeout_sec:
            return MotionCommand()

        return MotionCommand(
            target_wheel_velocity=self._command.target_wheel_velocity,
            target_yaw_rate=self._command.target_yaw_rate,
            stamp_sec=self._command.stamp_sec,
        )
