'''
Author: LCOIT dogcat.let@gmail.com
Date: 2026-05-27 22:27:05
LastEditors: LCOIT dogcat.let@gmail.com
LastEditTime: 2026-05-27 22:27:11
FilePath: /two_wheel_self_balance_car/src/balance_car_control/src/balance_types.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
#!/usr/bin/env python3

from dataclasses import dataclass


@dataclass
class BalanceState:
    stamp_sec: float
    pitch: float
    pitch_rate: float
    yaw: float = 0.0
    yaw_rate: float = 0.0
    wheel_velocity: float = 0.0
    # 左右轮平均角位置 [rad]，由 joint_states 的 position 求平均。
    # 位置环用它估计累计行程（≈ wheel_position × wheel_radius）。
    wheel_position: float = 0.0


@dataclass
class MotionCommand:
    """运动指令（由 /cmd_vel 或参数源解析而来）。

    target_wheel_velocity [rad/s] —— 左右轮平均角速度期望，喂给速度外环
    target_yaw_rate       [rad/s] —— 车体绕竖直轴角速度期望，喂给偏航环
    stamp_sec             [s]     —— 指令更新时间（用于超时判据）
    """

    target_wheel_velocity: float = 0.0
    target_yaw_rate: float = 0.0
    stamp_sec: float = 0.0


@dataclass
class WheelCommand:
    left: float
    right: float


@dataclass
class BalanceDebug:
    pitch: float
    pitch_rate: float
    pitch_target: float
    balance_output: float
    final_left: float
    final_right: float
    enabled: bool