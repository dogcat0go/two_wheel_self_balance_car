'''
Author: LCOIT dogcat.let@gmail.com
Date: 2026-05-27 22:37:42
LastEditors: LCOIT dogcat.let@gmail.com
LastEditTime: 2026-05-29 03:59:22
FilePath: /two_wheel_self_balance_car/src/balance_car_control/src/balance_pd.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
#!/usr/bin/env python3

from dataclasses import dataclass

from balance_types import BalanceState


@dataclass
class BalancePDConfig:
    """平衡 PD 增益。

    下游已经换成 effort_controllers，因此 compute() 的输出单位是
    单轮力矩 [N·m]，对应增益的物理单位：
        kp_pitch  [N·m / rad]
        kd_pitch  [N·m / (rad/s)]
        kv_wheel  [N·m / (rad/s)]  —— 用轮速做软阻尼，抑制飞车
        kx_position [m ]     —— 位置环增益,用于修正pitch_target
        min_balance_effort [N·m]   —— 死区外的最小力矩（克服静摩擦用）
    """

    kp_pitch: float = 1.0
    kd_pitch: float = 0.05
    kv_wheel: float = 0.01           # 之前 0.002 太小,提到 0.01
    kx_position: float = 0.05        # 新增:位置环增益,先用这个起步
    pitch_target: float = 0.0        # 配合 URDF 修正,先改回 0
    pitch_deadband: float = 0.0
    min_balance_effort: float = 0.0
    output_sign: float = 1.0
    # 滤波/限幅
    wheel_v_filter_tau: float = 0.025   # 一阶低通 ~6 Hz
    max_position: float = 5.0           # 位置积分限幅 [rad],约 16 cm 位移

class BalancePDController:
    def __init__(self, config: BalancePDConfig):
        self.config = config

    def compute(self, state: BalanceState) -> float:
        """根据 pitch / pitch_rate / 轮速，返回单轮目标力矩 [N·m]。"""
        pitch_error = state.pitch - self.config.pitch_target
        if abs(pitch_error) <= abs(self.config.pitch_deadband):
            return 0.0

        # 用 + kd*pitch_rate 而不是 - kd*pitch_rate：在当前坐标系下，
        # pitch_rate 与 pitch 同号意味着继续向同一方向倒，输出应当沿"倒下方向"
        # 给轮子施力矩——其在 base_link 上的反作用力矩恰好朝纠正方向，
        # 把车体往上扶。这点和经典 cart-pole 教科书相反，是 URDF 关节符号约定决定的。
        raw_effort = (
            self.config.kp_pitch * pitch_error
            + self.config.kd_pitch * state.pitch_rate
            - self.config.kv_wheel * state.wheel_velocity
        )
        min_effort = abs(self.config.min_balance_effort)
        if min_effort > 0.0 and abs(raw_effort) < min_effort:
            raw_effort = min_effort if raw_effort >= 0.0 else -min_effort

        return self.config.output_sign * raw_effort