#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

from attitude_estimator import AttitudeEstimator
from balance_pd import BalancePDConfig, BalancePDController
from safety_limiter import SafetyConfig, SafetyLimiter
from wheel_mixer import WheelMixer


class BalanceControllerNode(Node):
    def __init__(self):
        super().__init__("balance_controller_node")

        self._count_timer = 0
        self._debug_div = 0

        self.declare_parameter("enabled", False)
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("command_topic", "/wheel_effort_controller/commands")
        self.declare_parameter("debug_topic", "/balance_controller/debug")
        self.declare_parameter("control_rate", 200.0)
        self.declare_parameter("log_period_sec", 0.5)

        self.declare_parameter("pitch_target", 0.0)
        self.declare_parameter("kp_pitch", 2.0)
        self.declare_parameter("kd_pitch", 0.12)
        self.declare_parameter("kv_wheel", 0.0)
        self.declare_parameter("pitch_deadband", 0.0)
        # 注意：单位是 N·m（effort 内环）
        self.declare_parameter("min_balance_effort", 0.0)
        self.declare_parameter("output_sign", 1.0)

        # 注意：单位是 N·m 和 N·m/s（effort 内环）
        self.declare_parameter("max_wheel_effort", 2.5)
        self.declare_parameter("max_effort_slew", 60.0)
        self.declare_parameter("fall_angle_rad", 0.61)
        self.declare_parameter("imu_timeout_sec", 0.05)

        # 轮速一阶低通系数 (0, 1]。1.0=不滤波；越小越平滑但滞后越大。
        self.declare_parameter("wheel_velocity_lpf_alpha", 1.0)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.log_period_sec = float(self.get_parameter("log_period_sec").value)

        pd_config = BalancePDConfig(
            kp_pitch=float(self.get_parameter("kp_pitch").value),
            kd_pitch=float(self.get_parameter("kd_pitch").value),
            kv_wheel=float(self.get_parameter("kv_wheel").value),
            pitch_target=float(self.get_parameter("pitch_target").value),
            pitch_deadband=float(self.get_parameter("pitch_deadband").value),
            min_balance_effort=float(self.get_parameter("min_balance_effort").value),
            output_sign=float(self.get_parameter("output_sign").value),
        )
        safety_config = SafetyConfig(
            max_wheel_effort=float(self.get_parameter("max_wheel_effort").value),
            max_effort_slew=float(self.get_parameter("max_effort_slew").value),
            fall_angle_rad=float(self.get_parameter("fall_angle_rad").value),
            imu_timeout_sec=float(self.get_parameter("imu_timeout_sec").value),
        )

        self.estimator = AttitudeEstimator(
            wheel_velocity_lpf_alpha=float(
                self.get_parameter("wheel_velocity_lpf_alpha").value
            )
        )
        self.balance_pd = BalancePDController(pd_config)
        self.safety = SafetyLimiter(safety_config)
        self.wheel_mixer = WheelMixer()

        self.command_pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter("command_topic").value,
            10,
        )
        self.debug_pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter("debug_topic").value,
            10,
        )
        self.create_subscription(
            Imu,
            self.get_parameter("imu_topic").value,
            self.on_imu,
            20,
        )
        self.create_subscription(
            JointState,
            self.get_parameter("joint_states_topic").value,
            self.on_joint_states,
            20,
        )

        self.last_control_time = self.get_clock().now()
        self.last_log_time = self.get_clock().now()
        self.last_invalid_log_time = self.get_clock().now()
        control_rate = float(self.get_parameter("control_rate").value)
        self.create_timer(1.0 / control_rate, self.on_timer)

        self.get_logger().warn(
            "Balance controller started (effort mode). enabled={} "
            "kp={:.3f} Nm/rad kd={:.3f} Nm/(rad/s) kv={:.3f} Nm/(rad/s) "
            "deadband={:.3f}deg min_tau={:.3f} Nm".format(
                self.enabled,
                pd_config.kp_pitch,
                pd_config.kd_pitch,
                pd_config.kv_wheel,
                math.degrees(pd_config.pitch_deadband),
                pd_config.min_balance_effort,
            )
        )
        if not self.enabled:
            self.get_logger().warn(
                "Controller is in observe-only mode. It will publish [0.0, 0.0] "
                "until enabled:=true is provided."
            )
        self.get_logger().info(
            "Topics: imu={} joint_states={} command={} debug={} control_rate={:.1f}Hz".format(
                self.get_parameter("imu_topic").value,
                self.get_parameter("joint_states_topic").value,
                self.get_parameter("command_topic").value,
                self.get_parameter("debug_topic").value,
                control_rate,
            )
        )
        self.get_logger().info(
            "Safety: max_wheel_effort={:.3f} Nm max_effort_slew={:.3f} Nm/s "
            "fall_angle={:.2f}deg imu_timeout={:.3f}s".format(
                safety_config.max_wheel_effort,
                safety_config.max_effort_slew,
                math.degrees(safety_config.fall_angle_rad),
                safety_config.imu_timeout_sec,
            )
        )
        if abs(pd_config.output_sign) != 1.0:
            self.get_logger().warn(
                "output_sign is {:.3f}. It is usually expected to be 1.0 or -1.0; "
                "use kp/kd for gain size.".format(pd_config.output_sign)
            )

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_imu(self, msg):
        self.estimator.update_from_imu(msg, self.now_sec())

    def on_joint_states(self, msg):
        self.estimator.update_wheel_velocity(msg)

    def publish_wheels(self, left, right):
        self.command_pub.publish(Float64MultiArray(data=[left, right]))

    def stop_wheels(self):
        self.safety.reset()
        self.publish_wheels(0.0, 0.0)

    def publish_debug(self, state, balance_tau, left_tau, right_tau):
        """调试话题数据排布（effort 模式，单位 N·m / rad / rad·s⁻¹）：
            [pitch, pitch_rate, pitch_target,
             balance_tau, left_tau, right_tau,
             enabled, yaw, wheel_velocity]
        """
        if state is None:
            return

        self.debug_pub.publish(Float64MultiArray(data=[
            state.pitch,
            state.pitch_rate,
            self.balance_pd.config.pitch_target,
            balance_tau,
            left_tau,
            right_tau,
            1.0 if self.enabled else 0.0,
            state.yaw,
            state.wheel_velocity,
        ]))

    def maybe_log_control(self, state, balance_tau, left_tau, right_tau, final_published):
        now = self.get_clock().now()
        elapsed = (now - self.last_log_time).nanoseconds / 1e9
        if elapsed < self.log_period_sec:
            return

        self.last_log_time = now
        self.get_logger().info(
            "pitch={:.2f}deg pitch_rate={:.3f}rad/s wheel_v={:.3f}rad/s "
            "balance_tau={:.3f}Nm cmd=[{:.3f}, {:.3f}]Nm enabled={} published={} count_timer={}".format(
                math.degrees(state.pitch),
                state.pitch_rate,
                state.wheel_velocity,
                balance_tau,
                left_tau,
                right_tau,
                self.enabled,
                final_published,
                self._count_timer,
            )
        )

    def maybe_log_invalid_state(self, reason):
        now = self.get_clock().now()
        elapsed = (now - self.last_invalid_log_time).nanoseconds / 1e9
        if elapsed < self.log_period_sec:
            return

        self.last_invalid_log_time = now
        self.get_logger().warn(
            "Safety stop: {}. Publishing [0.0, 0.0]. count_timer={}".format(reason, self._count_timer)
        )

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = now
    # # dt clamp:抗 timer 抖动和 OS 卡顿
    # # 下限 1ms 防止除零或异常小;上限 4×名义 防止积分跳变
    # dt = max(1e-3, min(dt, 4 * self.control_period_nominal))
        state = self.estimator.latest_state()
        state_valid, invalid_reason = self.safety.check_state(state, self.now_sec())
        if not state_valid:
            self.maybe_log_invalid_state(invalid_reason)
            self.stop_wheels()
            return

        # PD 给出的是"让车体保持平衡所需的单轮力矩" [N·m]
        balance_tau = self.balance_pd.compute(state)

        # 第一阶段不做转向。未来 yaw 环或 cmd_vel 可以在这里生成 turn_tau（差动力矩）。
        turn_tau = 0.0
        mixed_command = self.wheel_mixer.mix(balance_tau, turn_tau)
        safe_command = self.safety.limit_command(mixed_command, dt)
        self._count_timer += 1
        if self.enabled:
            self.publish_wheels(safe_command.left, safe_command.right)
        else:
            self.stop_wheels()
            safe_command.left = 0.0
            safe_command.right = 0.0
        self._debug_div = (self._debug_div + 1) % 2
        if self._debug_div == 0:
            self.publish_debug(
                state=state,
                balance_tau=balance_tau,
                left_tau=safe_command.left,
                right_tau=safe_command.right,
            )
            self.maybe_log_control(
                state=state,
                balance_tau=balance_tau,
                left_tau=safe_command.left,
                right_tau=safe_command.right,
                final_published=self.enabled,
            )


def main():
    rclpy.init()
    node = BalanceControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.stop_wheels()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()