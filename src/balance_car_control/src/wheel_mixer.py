'''
Author: LCOIT dogcat.let@gmail.com
Date: 2026-05-27 22:58:27
LastEditors: LCOIT dogcat.let@gmail.com
LastEditTime: 2026-05-27 22:58:31
FilePath: /two_wheel_self_balance_car/src/balance_car_control/src/wheel_mixer.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
#!/usr/bin/env python3

from balance_types import WheelCommand


class WheelMixer:
    """将"前后平衡"和"差动转向"两个标量混合成左右轮的指令。

    当前下游是 effort_controllers/JointGroupEffortController，
    所以两个入参和输出都是单轮力矩 [N·m]：
        balance_tau —— 让车体保持平衡的纵向力矩（两轮同向）
        turn_tau    —— 让车体转向的差动力矩（左右反向）
    若以后换回 velocity 接口，单位会改成 [rad/s]，公式不变。
    """

    def mix(self, balance_tau: float, turn_tau: float = 0.0) -> WheelCommand:
        return WheelCommand(
            left=balance_tau - turn_tau,
            right=balance_tau + turn_tau,
        )