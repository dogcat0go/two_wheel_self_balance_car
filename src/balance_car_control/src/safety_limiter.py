'''
Author: LCOIT dogcat.let@gmail.com
Date: 2026-05-27 22:59:17
LastEditors: LCOIT dogcat.let@gmail.com
LastEditTime: 2026-05-27 22:59:26
FilePath: /two_wheel_self_balance_car/src/balance_car_control/src/safety_limiter.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
#!/usr/bin/env python3

from dataclasses import dataclass

from balance_types import WheelCommand


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


@dataclass
class SafetyConfig:
    """安全限幅参数。下游是 effort_controllers，故单位按力矩约定：

    max_wheel_effort  [N·m]    —— 单轮力矩饱和上限，应不超过 URDF 的 wheel_effort_limit
    max_effort_slew   [N·m/s]  —— 单轮力矩变化率上限，避免阶跃指令冲击
    fall_angle_rad    [rad]    —— pitch 超过这个角度就判摔，立刻清零输出
    imu_timeout_sec   [s]      —— IMU 数据陈旧到这个时间也判失效
    """

    max_wheel_effort: float = 2.5
    max_effort_slew: float = 60.0
    fall_angle_rad: float = 0.61
    imu_timeout_sec: float = 0.05
    # [s] 失效条件要连续持续这么久才判定为真失效；单拍抖动（IMU 偶发延迟、
    # 仿真掉帧）滤掉，不打断控制去做硬复位；真摔倒会持续超阈值，几乎不受影响。
    trip_debounce_sec: float = 0.1


class SafetyLimiter:
    def __init__(self, config: SafetyConfig):
        self.config = config
        self._last_left = 0.0
        self._last_right = 0.0
        self._invalid_since_sec = None

    def reset(self):
        self._last_left = 0.0
        self._last_right = 0.0
        self._invalid_since_sec = None

    def is_state_valid(self, state, now_sec):
        is_valid, _ = self.check_state_debounced(state, now_sec)
        return is_valid

    def check_state_debounced(self, state, now_sec):
        """带去抖的失效判定：原始条件要连续满足 trip_debounce_sec 才判失效。

        单拍抖动（未过门槛）仍返回有效，避免安全复位被误触发；一旦真的
        持续失效，行为与原始 check_state 一致（只是多等一小段确认时间）。

        "no_state"（还没收到过状态，如启动瞬间）不去抖：没有数据就是没有
        数据，下游拿不到 state 会直接崩，必须立即判失效。
        """
        if state is None:
            self._invalid_since_sec = now_sec
            return False, "no_state"

        raw_valid, reason = self.check_state(state, now_sec)
        if raw_valid:
            self._invalid_since_sec = None
            return True, "ok"

        if self._invalid_since_sec is None:
            self._invalid_since_sec = now_sec

        if now_sec - self._invalid_since_sec >= self.config.trip_debounce_sec:
            return False, reason
        return True, reason

    def check_state(self, state, now_sec):
        if state is None:
            return False, "no_state"

        age = now_sec - state.stamp_sec
        if age > self.config.imu_timeout_sec:
            return False, "imu_timeout age={:.3f}s limit={:.3f}s".format(
                age,
                self.config.imu_timeout_sec,
            )

        if abs(state.pitch) > self.config.fall_angle_rad:
            return False, "fall_angle pitch={:.3f}rad limit={:.3f}rad".format(
                state.pitch,
                self.config.fall_angle_rad,
            )

        return True, "ok"

    def _limit_one(self, target, previous, dt):
        effort_limit = abs(self.config.max_wheel_effort)
        saturated = clamp(target, -effort_limit, effort_limit)

        max_delta = abs(self.config.max_effort_slew) * max(dt, 0.0)
        delta = clamp(saturated - previous, -max_delta, max_delta)
        return previous + delta

    def limit_command(self, command: WheelCommand, dt):
        left = self._limit_one(command.left, self._last_left, dt)
        right = self._limit_one(command.right, self._last_right, dt)

        self._last_left = left
        self._last_right = right

        return WheelCommand(left=left, right=right)