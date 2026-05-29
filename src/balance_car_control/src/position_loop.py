#!/usr/bin/env python3
"""串级最外环：位置环。

三环串级结构：
    位置环(本文件)：输入 = 位置误差，输出 = 速度设定 v_setpoint [rad/s]
    速度环(velocity_loop.py)：输入 = 速度误差，输出 = 目标倾角 pitch_target
    倾角环(balance_pd.py)：输入 = pitch 误差，输出 = 单轮力矩 [N·m]

要解决的问题：
    纯“速度环”只保证 wheel_v → 0，不保证“停回原位”。停车要先后仰刹车，
    轮子会先倒退一点把质心追回竖直，速度归零后就停在倒退后的新位置。
    位置环记住一个“目标位置 x_target”，把这点倒退量再补回来。

设定值如何生成（关键）：
    x_target 持续积分“被命令的轮速” v_cmd：x_target += v_cmd * dt
      - v_cmd = 0（站立/停车）：x_target 冻结 → 位置环把倒退量拉回 → 不留位移。
      - v_cmd ≠ 0（前进/后退）：x_target 随指令移动 → 位置环帮助跟踪行程。
    这样同一套逻辑既能“停回原位”，又不会和正常行驶打架。

输出与符号：
    输出是“速度修正 v_corr”，叠加在 v_cmd 上作为速度环的 target_velocity：
        v_setpoint = v_cmd + v_corr
    v_corr 与 wheel_velocity 同单位、同符号约定（正 = 前进）。
    若启用后“越跑越远 / 发散”，把 output_sign 改成 -1.0。
"""

from dataclasses import dataclass


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


@dataclass
class PositionLoopConfig:
    """位置外环参数。

    单位（wheel_position 单位 rad，v_setpoint 单位 rad/s）：
        kp_x [ (rad/s) / rad ]      位置误差 → 速度修正，比例项
        kd_x [ (rad/s) / (rad/s) ]  位置误差变化率（≈ v_cmd − wheel_v）阻尼，常留 0
        max_velocity_correction [rad/s] 位置环最多能要求的速度修正，保护速度环
        max_position_error [rad]    位置误差 anti-windup 限幅（限制“记多远”）
        output_sign ±1.0            符号翻转开关
        enabled                     False 时直通：v_setpoint = v_cmd（退回两环）
    """

    kp_x: float = 0.5
    kd_x: float = 0.0
    max_velocity_correction: float = 4.0
    max_position_error: float = 20.0   # ~0.65 m 行程（×wheel_radius 0.0325）
    output_sign: float = 1.0
    enabled: bool = True


class PositionLoop:
    """位置外环 PD 控制器，输出速度设定给速度环。"""

    def __init__(self, config: PositionLoopConfig):
        self.config = config
        self._x_target = 0.0
        self._initialized = False
        # 缓存供日志/调试
        self.last_x_target = 0.0
        self.last_v_correction = 0.0
        self.last_v_setpoint = 0.0

    def reset(self, wheel_position=0.0):
        """复位：把目标位置对齐到当前轮位置，避免使能瞬间产生大误差甩车。"""
        self._x_target = wheel_position
        self._initialized = True
        self.last_x_target = wheel_position
        self.last_v_correction = 0.0
        self.last_v_setpoint = 0.0

    def compute(self, wheel_position, wheel_velocity, commanded_velocity, dt):
        """返回速度环应跟踪的 v_setpoint [rad/s]。"""
        cfg = self.config

        # 关闭或异常：直通，并把目标位置锁到当前，保证下次使能不跳变。
        if not cfg.enabled or dt <= 0.0:
            self._x_target = wheel_position
            self._initialized = True
            self.last_x_target = wheel_position
            self.last_v_correction = 0.0
            self.last_v_setpoint = commanded_velocity
            return commanded_velocity

        if not self._initialized:
            self._x_target = wheel_position
            self._initialized = True

        # 设定值积分被命令的速度：v_cmd=0 时冻结 → 位置保持。
        self._x_target += commanded_velocity * dt

        pos_error = self._x_target - wheel_position

        # anti-windup：限制“记多远”，并把目标位置回算，避免误差无限堆积。
        max_err = abs(cfg.max_position_error)
        if max_err > 0.0:
            if pos_error > max_err:
                pos_error = max_err
                self._x_target = wheel_position + max_err
            elif pos_error < -max_err:
                pos_error = -max_err
                self._x_target = wheel_position - max_err

        # 位置 PD：D 用误差变化率 (v_cmd − wheel_v)，默认 kd_x=0 交给速度环阻尼。
        d_error = commanded_velocity - wheel_velocity
        v_corr = cfg.output_sign * (cfg.kp_x * pos_error + cfg.kd_x * d_error)
        v_corr = clamp(
            v_corr,
            -abs(cfg.max_velocity_correction),
            abs(cfg.max_velocity_correction),
        )

        v_setpoint = commanded_velocity + v_corr

        self.last_x_target = self._x_target
        self.last_v_correction = v_corr
        self.last_v_setpoint = v_setpoint
        return v_setpoint
