#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

from attitude_estimator import AttitudeEstimator
from control_backend import create_balance_backend
from motion_command import MotionCommandAdapter, MotionCommandConfig
from safety_limiter import SafetyConfig, SafetyLimiter
from wheel_mixer import WheelMixer
from yaw_loop import YawLoop, YawLoopConfig


class BalanceControllerNode(Node):
    def __init__(self):
        super().__init__("balance_controller_node")

        self._count_timer = 0
        self._debug_div = 0

        self.declare_parameter("control_mode", "pid")
        self.declare_parameter("enabled", False)
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("command_topic", "/wheel_effort_controller/commands")
        self.declare_parameter("debug_topic", "/balance_controller/debug")
        self.declare_parameter("control_rate", 200.0)
        self.declare_parameter("log_period_sec", 0.5)

        self.declare_parameter("max_wheel_effort", 2.5)
        self.declare_parameter("max_effort_slew", 60.0)
        self.declare_parameter("fall_angle_rad", 0.61)
        self.declare_parameter("imu_timeout_sec", 0.05)
        self.declare_parameter("recovery_ramp_sec", 1.0)

        self.declare_parameter("wheel_velocity_lpf_alpha", 1.0)

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("motion_control_enabled", True)
        self.declare_parameter("wheel_radius", 0.0325)
        self.declare_parameter("max_linear_velocity", 0.5)
        self.declare_parameter("max_angular_velocity", 1.5)
        self.declare_parameter("max_wheel_velocity_cmd", 15.0)
        self.declare_parameter("cmd_vel_timeout_sec", 0.5)
        self.declare_parameter("target_velocity", 0.0)

        self.declare_parameter("yaw_loop_enabled", True)
        self.declare_parameter("kp_yaw", 0.02)
        self.declare_parameter("kd_yaw", 0.0)
        self.declare_parameter("max_turn_tau", 0.06)
        self.declare_parameter("yaw_output_sign", 1.0)
        self.declare_parameter("yaw_cmd_activate_threshold", 0.08)
        self.declare_parameter("yaw_rate_deadband", 0.05)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.log_period_sec = float(self.get_parameter("log_period_sec").value)
        self.motion_control_enabled = bool(
            self.get_parameter("motion_control_enabled").value
        )
        self.static_target_velocity = float(self.get_parameter("target_velocity").value)
        control_mode = str(self.get_parameter("control_mode").value)

        safety_config = SafetyConfig(
            max_wheel_effort=float(self.get_parameter("max_wheel_effort").value),
            max_effort_slew=float(self.get_parameter("max_effort_slew").value),
            fall_angle_rad=float(self.get_parameter("fall_angle_rad").value),
            imu_timeout_sec=float(self.get_parameter("imu_timeout_sec").value),
        )
        motion_config = MotionCommandConfig(
            wheel_radius=float(self.get_parameter("wheel_radius").value),
            max_linear_velocity=float(self.get_parameter("max_linear_velocity").value),
            max_angular_velocity=float(self.get_parameter("max_angular_velocity").value),
            max_wheel_velocity=float(self.get_parameter("max_wheel_velocity_cmd").value),
            cmd_vel_timeout_sec=float(self.get_parameter("cmd_vel_timeout_sec").value),
            enabled=self.motion_control_enabled,
        )
        yaw_config = YawLoopConfig(
            kp_yaw=float(self.get_parameter("kp_yaw").value),
            kd_yaw=float(self.get_parameter("kd_yaw").value),
            max_turn_tau=float(self.get_parameter("max_turn_tau").value),
            output_sign=float(self.get_parameter("yaw_output_sign").value),
            enabled=bool(self.get_parameter("yaw_loop_enabled").value),
            yaw_cmd_activate_threshold=float(
                self.get_parameter("yaw_cmd_activate_threshold").value
            ),
            yaw_rate_deadband=float(self.get_parameter("yaw_rate_deadband").value),
        )

        self.estimator = AttitudeEstimator(
            wheel_velocity_lpf_alpha=float(
                self.get_parameter("wheel_velocity_lpf_alpha").value
            )
        )
        self.balance_backend = create_balance_backend(control_mode, self)
        self.motion_adapter = MotionCommandAdapter(motion_config)
        self.yaw_loop = YawLoop(yaw_config)
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
        self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            self.on_cmd_vel,
            10,
        )

        self.last_control_time = self.get_clock().now()
        self.last_log_time = self.get_clock().now()
        self.last_invalid_log_time = self.get_clock().now()
        self._safety_invalid = False
        self._recovery_end_sec = 0.0
        self.recovery_ramp_sec = float(self.get_parameter("recovery_ramp_sec").value)
        control_rate = float(self.get_parameter("control_rate").value)
        self.create_timer(1.0 / control_rate, self.on_timer)

        self.get_logger().warn(
            "Balance controller started (effort mode). control_mode={} enabled={}".format(
                self.balance_backend.mode_name,
                self.enabled,
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
        self.get_logger().info(
            "Motion: enabled={} cmd_vel={} wheel_r={:.4f}m "
            "v_max={:.2f}m/s w_max={:.2f}rad/s timeout={:.2f}s".format(
                motion_config.enabled,
                self.get_parameter("cmd_vel_topic").value,
                motion_config.wheel_radius,
                motion_config.max_linear_velocity,
                motion_config.max_angular_velocity,
                motion_config.cmd_vel_timeout_sec,
            )
        )
        self.get_logger().info(
            "Yaw loop: enabled={} kp_yaw={:.4f} kd_yaw={:.4f} "
            "max_turn_tau={:.3f}Nm sign={:.0f}".format(
                yaw_config.enabled,
                yaw_config.kp_yaw,
                yaw_config.kd_yaw,
                yaw_config.max_turn_tau,
                yaw_config.output_sign,
            )
        )
        self.balance_backend.log_config(self.get_logger())

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_imu(self, msg):
        self.estimator.update_from_imu(msg, self.now_sec())

    def on_joint_states(self, msg):
        self.estimator.update_wheel_velocity(msg)

    def on_cmd_vel(self, msg):
        self.motion_adapter.update_from_twist(
            linear_x=float(msg.linear.x),
            angular_z=float(msg.angular.z),
            stamp_sec=self.now_sec(),
        )

    def publish_wheels(self, left, right):
        self.command_pub.publish(Float64MultiArray(data=[left, right]))

    def stop_wheels(self):
        self.safety.reset()
        self.publish_wheels(0.0, 0.0)

    def publish_debug(
        self,
        state,
        control_output,
        left_tau,
        right_tau,
        target_yaw_rate,
        turn_tau,
    ):
        """调试话题数据排布（effort 模式）：
            [0] pitch [rad]
            [1] pitch_rate [rad/s]
            [2] pitch_target [rad]
            [3] balance_tau [N·m]
            [4] left_tau [N·m]
            [5] right_tau [N·m]
            [6] enabled [0/1]
            [7] yaw [rad]
            [8] wheel_velocity [rad/s]
            [9] target_wheel_velocity [rad/s]
            [10] target_yaw_rate [rad/s]
            [11] turn_tau [N·m]
            [12] yaw_rate [rad/s]
            [13] wheel_position [rad]
            [14] x_target [rad]
            [15] velocity_setpoint [rad/s]
        """
        if state is None:
            return

        self.debug_pub.publish(Float64MultiArray(data=[
            state.pitch,
            state.pitch_rate,
            control_output.pitch_target,
            control_output.balance_tau,
            left_tau,
            right_tau,
            1.0 if self.enabled else 0.0,
            state.yaw,
            state.wheel_velocity,
            control_output.target_wheel_velocity,
            target_yaw_rate,
            turn_tau,
            state.yaw_rate,
            state.wheel_position,
            control_output.x_target,
            control_output.velocity_setpoint,
        ]))

    def maybe_log_control(
        self,
        state,
        control_output,
        left_tau,
        right_tau,
        turn_tau,
        target_yaw_rate,
        final_published,
    ):
        now = self.get_clock().now()
        elapsed = (now - self.last_log_time).nanoseconds / 1e9
        if elapsed < self.log_period_sec:
            return

        self.last_log_time = now
        self.get_logger().info(
            "mode={} pitch={:.2f}deg pitch_tgt={:.2f}deg wheel_v={:.3f} v_set={:.3f} "
            "tgt_v={:.3f} x={:.3f} x_tgt={:.3f} turn_tau={:.3f}Nm balance_tau={:.3f}Nm "
            "cmd=[{:.3f}, {:.3f}]Nm enabled={} count_timer={}".format(
                self.balance_backend.mode_name,
                math.degrees(state.pitch),
                math.degrees(control_output.pitch_target),
                state.wheel_velocity,
                control_output.velocity_setpoint,
                control_output.target_wheel_velocity,
                state.wheel_position,
                control_output.x_target,
                turn_tau,
                control_output.balance_tau,
                left_tau,
                right_tau,
                self.enabled,
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
            "Safety stop: {}. Publishing [0.0, 0.0]. count_timer={}".format(
                reason, self._count_timer
            )
        )

    def resolve_target_wheel_velocity(self, motion_cmd):
        if self.motion_control_enabled:
            return motion_cmd.target_wheel_velocity
        return self.static_target_velocity

    def resolve_target_yaw_rate(self, motion_cmd):
        if self.motion_control_enabled:
            return motion_cmd.target_yaw_rate
        return 0.0

    def _recovery_gain_scale(self, now_sec: float) -> float:
        remaining = self._recovery_end_sec - now_sec
        if remaining <= 0.0:
            return 1.0
        ramp = max(self.recovery_ramp_sec, 1e-3)
        return 0.25 + 0.75 * (1.0 - remaining / ramp)

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = now
        now_sec = self.now_sec()

        state = self.estimator.latest_state()
        state_valid, invalid_reason = self.safety.check_state(state, now_sec)
        if not state_valid:
            self.maybe_log_invalid_state(invalid_reason)
            if not self._safety_invalid:
                self._safety_invalid = True
                self.balance_backend.reset(state)
                self.yaw_loop.reset()
            self.stop_wheels()
            return

        if self._safety_invalid:
            self._safety_invalid = False
            self.balance_backend.reset(state)
            self.yaw_loop.reset()
            self._recovery_end_sec = now_sec + self.recovery_ramp_sec

        motion_cmd = self.motion_adapter.current_command(now_sec)
        target_wheel_velocity = self.resolve_target_wheel_velocity(motion_cmd)
        target_yaw_rate = self.resolve_target_yaw_rate(motion_cmd)

        control_output = self.balance_backend.compute(
            state=state,
            target_wheel_velocity=target_wheel_velocity,
            dt=dt,
            enabled=self.enabled,
        )

        recovery_scale = self._recovery_gain_scale(now_sec)
        if recovery_scale < 1.0:
            control_output.balance_tau *= recovery_scale

        if not self.enabled:
            self.yaw_loop.reset()

        turn_tau = self.yaw_loop.compute(
            target_yaw_rate=target_yaw_rate,
            yaw_rate=state.yaw_rate,
            dt=dt,
        )
        mixed_command = self.wheel_mixer.mix(control_output.balance_tau, turn_tau)
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
                control_output=control_output,
                left_tau=safe_command.left,
                right_tau=safe_command.right,
                target_yaw_rate=target_yaw_rate,
                turn_tau=turn_tau,
            )
            self.maybe_log_control(
                state=state,
                control_output=control_output,
                left_tau=safe_command.left,
                right_tau=safe_command.right,
                turn_tau=turn_tau,
                target_yaw_rate=target_yaw_rate,
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
