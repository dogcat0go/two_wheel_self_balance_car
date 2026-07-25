#!/usr/bin/env python3
"""一维 KF 斜坡估计（slope_kf_simple.md §2–3）。

观测：
  sin z_dyn = (τ/r − m̃·v̇ − c·v) / (M·g)          # 简化，无耦合
  sin z_kin = (f_Wz − g) / v̇ ,  R_kin = σ_f²/v̇² + σ_ε²

KF：预测 P += q·dt；再对 (z_dyn,R_dyn)、(z_kin,R_kin) 串行标量更新。

发布：
  /slope/alpha   Float32  —— 主输出 α̂ [rad]
  /slope/dyn_debug Float64MultiArray
    [0] sin_dyn  [1] z_dyn  [2] sin_kin  [3] z_kin
    [4] v  [5] vdot  [6] tau_f  [7] s  [8] R_kin
    [9] f_wz  [10] z_kin_held  [11] kin_gate
    [12] alpha_hat  [13] P  [14] dyn_mismatch=(τ_f/r−m̃v̇)/(Mg)
    [15] tau_f/r  [16] m̃·v̇   —— 平地对比这两条：同形差比例→m̃；差常偏→c；只边沿错位→延迟
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32, Float64MultiArray


class SlopeDynObsNode(Node):
    def __init__(self):
        super().__init__("slope_dyn_obs")
        self.g = 9.8
        self.r = 0.0325
        self.M = 0.756
        self.Iw = 2.0e-5
        self.ell = 0.0345
        # 平地回归标定（τ_f/r − m_eff·v̇ = c·v，固定 m_eff 只拟合 c）：
        # 前/后半段独立拟合 3.908 / 3.921，一致性很好，取整段拟合值
        self.c = 3.916
        # 延迟拧到 0.2 仍压不矮鼓包 → 主因多半不是延迟，先回到 0.1
        self.tau_d = 0.1
        self.tau_lpf = 0.1
        self.sig_f = 0.15
        self.sig_eps = 0.01
        self.vdot_gate = 0.3
        self.q = 3.0e-4
        self.R_dyn = 0.4
        self.m_eff = self.M + self.Iw / self.r**2
        # 启动按平地先验：α=0、P 小；前 hold_sec 对外发布 α=0，躲开起立瞬态
        self.assume_flat_start = True
        self.flat_hold_sec = 3.0
        self._t0 = None

        self.alpha = 0.0
        self.P = 1.0e-4 if self.assume_flat_start else 1.0e-3
        self.v = self.v_prev = self.vdot = 0.0
        self.tau_raw = 0.0
        self.tau_f = 0.0
        self._s = 0.0
        self.t_js = self.t_imu = None
        self.gy = self.gy_prev = 0.0
        self.z_kin_held = 0.0

        self.create_subscription(JointState, "/joint_states", self.js_cb, 50)
        self.create_subscription(Imu, "/imu/data", self.imu_cb, 100)
        self.create_subscription(
            Float64MultiArray,
            "/wheel_effort_controller/commands",
            lambda m: setattr(self, "tau_raw", float(sum(m.data))),
            50,
        )
        self.pub_alpha = self.create_publisher(Float32, "/slope/alpha", 10)
        self.dbg = self.create_publisher(Float64MultiArray, "/slope/dyn_debug", 10)
        self.get_logger().info(
            "KF: assume_flat_start={} hold={:.1f}s P0={:.1e}".format(
                self.assume_flat_start, self.flat_hold_sec, self.P
            )
        )

    def js_cb(self, msg: JointState):
        try:
            iL = msg.name.index("left_wheel_joint")
            iR = msg.name.index("right_wheel_joint")
        except ValueError:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t_js is None:
            self.t_js = t
            return
        dt = t - self.t_js
        self.t_js = t
        if not (0.0 < dt < 0.1):
            return

        self.v_prev, self.v = self.v, self.r * 0.5 * (
            msg.velocity[iL] + msg.velocity[iR]
        )
        self.vdot += (dt / self.tau_d) * (
            (self.v - self.v_prev) / dt - self.vdot
        )
        self.tau_f += (dt / self.tau_lpf) * (self.tau_raw - self.tau_f)
        self._s = self.r * 0.5 * (msg.position[iL] + msg.position[iR])

    def imu_cb(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t_imu is None:
            self.t_imu = t
            return
        dt = t - self.t_imu
        self.t_imu = t
        if not (0.0 < dt < 0.05):
            return

        q = msg.orientation
        theta = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        self.gy_prev, self.gy = self.gy, msg.angular_velocity.y
        theta_ddot = (self.gy - self.gy_prev) / dt
        fx = msg.linear_acceleration.x
        fz = msg.linear_acceleration.z

        sin_dyn = (
            self.tau_f / self.r - self.m_eff * self.vdot - self.c * self.v
        ) / (self.M * self.g)
        z_dyn = math.asin(max(-0.6, min(0.6, sin_dyn)))
        tau_over_r = self.tau_f / self.r
        m_vdot = self.m_eff * self.vdot
        dyn_mismatch = (tau_over_r - m_vdot) / (self.M * self.g)

        fxc = fx - self.ell * theta_ddot
        fzc = fz + self.ell * self.gy**2
        f_wz = -fxc * math.sin(theta) + fzc * math.cos(theta)
        vd = self.vdot if abs(self.vdot) > 1e-4 else 1e-4
        sin_kin = (f_wz - self.g) / vd
        z_kin = math.asin(max(-0.6, min(0.6, sin_kin)))
        r_kin = min((self.sig_f / vd) ** 2 + self.sig_eps**2, 1e6)

        kin_gate = 1.0 if abs(self.vdot) > self.vdot_gate else 0.0
        if kin_gate:
            self.z_kin_held = z_kin

        # §3 一维 KF：预测 + dyn → kin 串行更新
        self.P += self.q * dt
        for z, R in ((z_dyn, self.R_dyn), (z_kin, r_kin)):
            K = self.P / (self.P + R)
            self.alpha += K * (z - self.alpha)
            self.P *= 1.0 - K

        if self._t0 is None:
            self._t0 = t
        # 控制用：起步 hold 内报平地，hold 后 1s 渐入避免阶跃；debug[12] 仍是真实 KF
        alpha_pub = self.alpha
        if self.assume_flat_start:
            t_run = t - self._t0
            if t_run < self.flat_hold_sec:
                alpha_pub = 0.0
            else:
                alpha_pub = self.alpha * min(
                    1.0, (t_run - self.flat_hold_sec) / 1.0
                )

        self.pub_alpha.publish(Float32(data=float(alpha_pub)))
        out = Float64MultiArray()
        out.data = [
            float(sin_dyn),
            float(z_dyn),
            float(sin_kin),
            float(z_kin),
            float(self.v),#4 速度
            float(self.vdot),#5 加速度
            float(self.tau_f),#6 力矩
            float(self._s),#7 位置
            float(r_kin),#8
            float(f_wz),#9
            float(self.z_kin_held),#10
            float(kin_gate),
            float(self.alpha), #12 KF 真值
            float(self.P),
            float(dyn_mismatch),  #14
            float(tau_over_r),    #15
            float(m_vdot),        #16
        ]
        self.dbg.publish(out)


def main():
    rclpy.init()
    node = SlopeDynObsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
