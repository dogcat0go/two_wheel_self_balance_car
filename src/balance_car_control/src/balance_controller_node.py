#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

from attitude_estimator import AttitudeEstimator
from balance_pd import BalancePDConfig, BalancePDController
from motion_command import MotionCommandAdapter, MotionCommandConfig
from safety_limiter import SafetyConfig, SafetyLimiter
from velocity_loop import VelocityLoop, VelocityLoopConfig
from wheel_mixer import WheelMixer
from yaw_loop import YawLoop, YawLoopConfig


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

        # ---- 串级外环：速度环 ----
        # 外环输出"目标倾角 pitch_target"喂给内环 PD，自动消除前冲/漂移。
        # 启用外环后，内环的 kv_wheel 应设 0（速度由外环负责），pitch_target
        # 当作"额外偏置"叠加在外环输出之上（一般留 0，让外环积分自动找平）。
        self.declare_parameter("velocity_loop_enabled", True)
        self.declare_parameter("kp_v", 0.005)
        self.declare_parameter("ki_v", 0.002)
        self.declare_parameter("target_velocity", 0.0)
        self.declare_parameter("pitch_target_limit", 0.12)
        self.declare_parameter("velocity_integral_limit", 0.12)
        self.declare_parameter("velocity_output_sign", 1.0)

        # ---- 运动指令：/cmd_vel → 速度 + 方向 ----
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("motion_control_enabled", True)
        self.declare_parameter("wheel_radius", 0.0325)
        self.declare_parameter("max_linear_velocity", 0.5)
        self.declare_parameter("max_angular_velocity", 1.5)
        self.declare_parameter("max_wheel_velocity_cmd", 15.0)
        self.declare_parameter("cmd_vel_timeout_sec", 0.5)

        # ---- 偏航环（方向 / 差动力矩）----
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
        # 内环 pitch_target 偏置（叠加在外环输出上）。外环开时一般留 0。
        self.pitch_target_bias = float(self.get_parameter("pitch_target").value)
        # motion_control 关闭时，速度外环仍可用 yaml 里的静态 target_velocity。
        self.static_target_velocity = float(self.get_parameter("target_velocity").value)

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
        velocity_config = VelocityLoopConfig(
            kp_v=float(self.get_parameter("kp_v").value),
            ki_v=float(self.get_parameter("ki_v").value),
            target_velocity=self.static_target_velocity,
            pitch_target_limit=float(self.get_parameter("pitch_target_limit").value),
            integral_limit=float(self.get_parameter("velocity_integral_limit").value),
            output_sign=float(self.get_parameter("velocity_output_sign").value),
            enabled=bool(self.get_parameter("velocity_loop_enabled").value),
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
        self.balance_pd = BalancePDController(pd_config)
        self.velocity_loop = VelocityLoop(velocity_config)
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
        self.get_logger().info(
            "Velocity loop (cascade outer): enabled={} kp_v={:.4f} ki_v={:.4f} "
            "target_v={:.3f}rad/s pitch_target_limit={:.2f}deg sign={:.0f}".format(
                velocity_config.enabled,
                velocity_config.kp_v,
                velocity_config.ki_v,
                velocity_config.target_velocity,
                math.degrees(velocity_config.pitch_target_limit),
                velocity_config.output_sign,
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
        balance_tau,
        left_tau,
        right_tau,
        target_wheel_velocity,
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
            target_wheel_velocity,
            target_yaw_rate,
            turn_tau,
            state.yaw_rate,
        ]))

    def maybe_log_control(
        self,
        state,
        balance_tau,
        left_tau,
        right_tau,
        turn_tau,
        target_wheel_velocity,
        target_yaw_rate,
        final_published,
    ):
        now = self.get_clock().now()
        elapsed = (now - self.last_log_time).nanoseconds / 1e9
        if elapsed < self.log_period_sec:
            return

        self.last_log_time = now
        self.get_logger().info(
            "pitch={:.2f}deg pitch_tgt={:.2f}deg wheel_v={:.3f} tgt_v={:.3f} "
            "yaw_rate={:.3f} tgt_w={:.3f} turn_tau={:.3f}Nm balance_tau={:.3f}Nm "
            "cmd=[{:.3f}, {:.3f}]Nm enabled={} count_timer={}".format(
                math.degrees(state.pitch),
                math.degrees(self.balance_pd.config.pitch_target),
                state.wheel_velocity,
                target_wheel_velocity,
                state.yaw_rate,
                target_yaw_rate,
                turn_tau,
                balance_tau,
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
            # 车已倒/数据失效：复位外环积分，避免积分饱和导致复位后甩头
            self.velocity_loop.reset()
            self.yaw_loop.reset()
            self.stop_wheels()
            return

        now_sec = self.now_sec()
        motion_cmd = self.motion_adapter.current_command(now_sec)
        if self.motion_control_enabled:
            target_wheel_velocity = motion_cmd.target_wheel_velocity
            target_yaw_rate = motion_cmd.target_yaw_rate
        else:
            target_wheel_velocity = self.static_target_velocity
            target_yaw_rate = 0.0

        self.velocity_loop.config.target_velocity = target_wheel_velocity

        # 串级外环（速度环）：根据轮速误差算出"目标倾角"喂给内环。
        if self.enabled:
            pitch_target = self.velocity_loop.compute(state.wheel_velocity, dt)
        else:
            self.velocity_loop.reset()
            self.yaw_loop.reset()
            pitch_target = 0.0
        self.balance_pd.config.pitch_target = self.pitch_target_bias + pitch_target

        # 内环 PD 给出的是"让车体保持平衡所需的单轮力矩" [N·m]
        balance_tau = self.balance_pd.compute(state)

        # 偏航环：角速度 PD → 差动力矩 turn_tau
        turn_tau = self.yaw_loop.compute(
            target_yaw_rate=target_yaw_rate,
            yaw_rate=state.yaw_rate,
            dt=dt,
        )
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
                target_wheel_velocity=target_wheel_velocity,
                target_yaw_rate=target_yaw_rate,
                turn_tau=turn_tau,
            )
            self.maybe_log_control(
                state=state,
                balance_tau=balance_tau,
                left_tau=safe_command.left,
                right_tau=safe_command.right,
                turn_tau=turn_tau,
                target_wheel_velocity=target_wheel_velocity,
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