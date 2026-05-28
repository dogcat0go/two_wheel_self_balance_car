#!/usr/bin/env python3
"""串级 PID 的外环：速度环。

平衡车的串级结构：
    外环(本文件)：输入 = 轮速误差，输出 = 目标倾角 pitch_target
    内环(balance_pd.py)：输入 = pitch 误差，输出 = 单轮力矩 [N·m]

物理直觉：
    要让车减速/不前冲，就让车体往"后"仰一点，用倾角去换加减速；
    内环再把车体稳定到这个目标倾角。这样外环只管"车速"，
    内环只管"姿态"，两个环解耦、各自好调。

关键的符号约定（本车实测推导）：
    本车质心/IMU 存在零偏，pitch_target=0 时车会朝某一方向前冲。
    实测：轮速朝正方向前冲时，需要把 pitch_target 抬正才能把车刹住。
    因此这里把"误差"定义为 (wheel_v - target_velocity)（被控量 - 设定值，
    即反作用方向），而不是教科书的 (设定值 - 被控量)。
    这样轮速为正时 pitch_target 输出为正，正好对得上。
    如果启用后发现"漂移反而被加速"，把 output_sign 改成 -1.0 翻转即可。

积分项的额外好处：
    它会自动累积到抵消质心/IMU 零偏的那个角度（≈θ_natural），
    所以内环的 pitch_target 不再需要手工标定，外环积分自动找平。
"""

from dataclasses import dataclass


@dataclass
class VelocityLoopConfig:
    """速度外环参数。

    单位说明（pitch_target 单位是 rad，wheel_v 单位是 rad/s）：
        kp_v               [rad / (rad/s)]      速度误差 -> 目标倾角，比例项
        ki_v               [rad / (rad/s · s)]  积分项，消除稳态漂移 + 自动找平零偏
        target_velocity    [rad/s]              期望轮速；站立平衡=0，前进/后退给非零
        pitch_target_limit [rad]                外环最多能要求的倾角，保护内环不被拉爆
        integral_limit     [rad]                积分项 anti-windup 限幅
        output_sign        ±1.0                 符号翻转开关（见模块顶部说明）
    """

    kp_v: float = 0.005
    ki_v: float = 0.002
    target_velocity: float = 0.0
    pitch_target_limit: float = 0.12   # ~6.9°，外环要求的最大倾角
    integral_limit: float = 0.12       # 积分贡献限幅，单位同 pitch_target
    output_sign: float = 1.0
    enabled: bool = True


class VelocityLoop:
    """速度外环 PI 控制器，输出给内环的 pitch_target。"""

    def __init__(self, config: VelocityLoopConfig):
        self.config = config
        self._integral = 0.0
        # 缓存最近一次输出，方便日志/调试查看
        self.last_pitch_target = 0.0
        self.last_p_term = 0.0
        self.last_i_term = 0.0

    def reset(self):
        """清空积分。倒车/安全停车/失能时调用，避免积分饱和带来上电甩头。"""
        self._integral = 0.0
        self.last_pitch_target = 0.0
        self.last_p_term = 0.0
        self.last_i_term = 0.0

    def compute(self, wheel_velocity: float, dt: float) -> float:
        cfg = self.config
        if not cfg.enabled or dt <= 0.0:
            return 0.0

        # 反作用误差：被控量 - 设定值（符号见模块顶部说明）
        v_error = wheel_velocity - cfg.target_velocity

        p_term = cfg.kp_v * v_error

        # 积分 + anti-windup：先累加，再按"积分贡献"限幅，
        # 并把积分状态回算回去，防止积分量持续堆积（conditional clamp）。
        self._integral += v_error * dt
        i_term = cfg.ki_v * self._integral
        if i_term > cfg.integral_limit:
            i_term = cfg.integral_limit
            if cfg.ki_v != 0.0:
                self._integral = i_term / cfg.ki_v
        elif i_term < -cfg.integral_limit:
            i_term = -cfg.integral_limit
            if cfg.ki_v != 0.0:
                self._integral = i_term / cfg.ki_v

        pitch_target = cfg.output_sign * (p_term + i_term)

        # 输出限幅：外环最多只能要求 ±pitch_target_limit 的倾角
        limit = abs(cfg.pitch_target_limit)
        if pitch_target > limit:
            pitch_target = limit
        elif pitch_target < -limit:
            pitch_target = -limit

        self.last_p_term = cfg.output_sign * p_term
        self.last_i_term = cfg.output_sign * i_term
        self.last_pitch_target = pitch_target
        return pitch_target
