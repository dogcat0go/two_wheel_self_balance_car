#!/usr/bin/env python3
"""LQR 参考轨迹 x_ref = [θ_ref, θ̇_ref, x_ref, ẋ_ref]。"""

from dataclasses import dataclass

from balance_types import LQRReference

_STANDSTILL_WHEEL_VEL = 1e-3  # [rad/s] 停止指令判据
_STOPPED_WHEEL_VEL = 0.05  # [rad/s] 实车真正静止判据：低于此才锚定 x_ref


@dataclass
class LQRReferenceConfig:
    wheel_radius: float = 0.0325
    pitch_bias: float = -0.011
    enabled: bool = True
    ref_accel_limit: float = 0.3  # [m/s²] x_dot_ref 斜坡限幅；<=0 关闭（阶跃）
    max_lead: float = 0.06  # [m] |实际位置-x_ref| 全程限幅，防止误差累积成大跳变


class LQRReferenceTracker:
    """维护 LQR 参考状态。"""

    def __init__(self, config: LQRReferenceConfig):
        self.config = config
        self._reference = LQRReference(theta_ref=config.pitch_bias)
        self._standstill_latched = False

    def reset(self, wheel_position: float = 0.0) -> None:
        x_ref_m = wheel_position * self.config.wheel_radius
        self._reference = LQRReference(
            theta_ref=self.config.pitch_bias,
            theta_dot_ref=0.0,
            x_ref=x_ref_m,
            x_dot_ref=0.0,
        )
        self._standstill_latched = True

    def update(
        self,
        commanded_velocity: float,
        wheel_position: float,
        wheel_velocity: float,
        dt: float,
        theta_eq: float = 0.0,
    ) -> LQRReference:
        """theta_eq: 坡上平衡倾角（叠在 pitch_bias 上），平地为 0。"""
        cfg = self.config
        stop_cmd = abs(commanded_velocity) < _STANDSTILL_WHEEL_VEL

        if cfg.enabled and not stop_cmd:
            target = commanded_velocity * cfg.wheel_radius
            self._standstill_latched = False
        else:
            target = 0.0

        # x_dot_ref 斜坡限幅：停止指令时参考速度滑到 0，k_x 全程在线负责减速；
        # 只在实车真正静止那一刻，把 x_ref 顺势对齐到实际位置一次（消掉刹车
        # 过冲残差），避免早锚（车还在滑）被回拉，也避免全程关掉 k_x 导致
        # 姿态环把车稳在非零巡航速度上（TWIP 无位置反馈时速度是自由积分）。
        prev = self._reference.x_dot_ref
        if cfg.ref_accel_limit > 0.0 and dt > 0.0:
            step = cfg.ref_accel_limit * dt
            x_dot_ref = min(max(target, prev - step), prev + step)
        else:
            x_dot_ref = target

        just_stopped = (
            stop_cmd
            and not self._standstill_latched
            and abs(wheel_velocity) <= _STOPPED_WHEEL_VEL
        )
        if just_stopped:
            self._standstill_latched = True

        if just_stopped:
            x_dot_ref = 0.0
            x_ref = wheel_position * cfg.wheel_radius
        elif stop_cmd and self._standstill_latched:
            x_dot_ref = 0.0
            x_ref = self._reference.x_ref
        else:
            x_ref = self._reference.x_ref + x_dot_ref * dt
            # max_lead 限幅只在正常行驶（未收到停止指令）时生效：防止巡航
            # 跟踪误差无限累积。停止指令已发但车还没真停这段，绝不能限幅——
            # 这段恰恰要让 e_x 自由增长，靠 k_x 把残余速度真正拉停；限幅会
            # 把回拉力矩封顶，车速压不到位就永远到不了锚定阈值，变成低速
            # 长距离蠕动（停不下来）。
            if cfg.max_lead > 0.0 and not stop_cmd:
                x_pos = wheel_position * cfg.wheel_radius
                x_ref = min(max(x_ref, x_pos - cfg.max_lead), x_pos + cfg.max_lead)

        self._reference = LQRReference(
            theta_ref=cfg.pitch_bias + theta_eq,
            theta_dot_ref=0.0,
            x_ref=x_ref,
            x_dot_ref=x_dot_ref,
        )
        return self._reference

    @property
    def last_reference(self) -> LQRReference:
        return self._reference
