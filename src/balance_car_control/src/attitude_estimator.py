'''
Author: LCOIT dogcat.let@gmail.com
Date: 2026-05-27 22:58:06
LastEditors: LCOIT dogcat.let@gmail.com
LastEditTime: 2026-05-27 22:58:09
FilePath: /two_wheel_self_balance_car/src/balance_car_control/src/attitude_estimator.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
#!/usr/bin/env python3

import math

from balance_types import BalanceState


def quaternion_to_euler(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class AttitudeEstimator:
    def __init__(self):
        self._latest_state = None
        self._wheel_velocity = 0.0

    def update_from_imu(self, imu_msg, stamp_sec):
        q = imu_msg.orientation
        _, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)

        self._latest_state = BalanceState(
            stamp_sec=stamp_sec,
            pitch=pitch,
            pitch_rate=imu_msg.angular_velocity.y,
            yaw=yaw,
            wheel_velocity=self._wheel_velocity,
        )

    def update_wheel_velocity(self, joint_state_msg):
        velocities = {}
        for index, name in enumerate(joint_state_msg.name):
            if index < len(joint_state_msg.velocity):
                velocities[name] = joint_state_msg.velocity[index]

        left = velocities.get("left_wheel_joint")
        right = velocities.get("right_wheel_joint")
        if left is None or right is None:
            return

        self._wheel_velocity = 0.5 * (left + right)
        if self._latest_state is not None:
            self._latest_state.wheel_velocity = self._wheel_velocity

    def latest_state(self):
        return self._latest_state