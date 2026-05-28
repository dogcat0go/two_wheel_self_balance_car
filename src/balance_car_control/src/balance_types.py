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
    wheel_velocity: float = 0.0


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