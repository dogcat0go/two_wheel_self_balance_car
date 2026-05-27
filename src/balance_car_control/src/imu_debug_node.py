#!/usr/bin/env python3
'''
Author: LCOIT dogcat.let@gmail.com
Date: 2026-05-27 02:30:15
LastEditors: LCOIT dogcat.let@gmail.com
LastEditTime: 2026-05-27 16:34:30
FilePath: /two_wheel_self_balance_car/src/balance_car_control/src/imu_debug_node.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray

# TODO: 理解 四元数转换为欧拉角 的数学原理
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


class ImuDebugNode(Node):
    def __init__(self):
        super().__init__("imu_debug_node")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("print_degrees", True)
        self.declare_parameter("log_period_sec", 0.25)

        imu_topic = self.get_parameter("imu_topic").value
        self.print_degrees = self.get_parameter("print_degrees").value
        self.log_period_sec = float(self.get_parameter("log_period_sec").value)

        self.last_log_time = self.get_clock().now()
        self.debug_pub = self.create_publisher(
            Float64MultiArray,
            "/balance_controller/imu_debug",
            10,
        )
        self.create_subscription(Imu, imu_topic, self.on_imu, 10)

        self.get_logger().info(f"Listening IMU topic: {imu_topic}")

    def on_imu(self, msg):
        q = msg.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)

        av = msg.angular_velocity
        self.debug_pub.publish(Float64MultiArray(data=[
            roll,
            pitch,
            yaw,
            av.x,
            av.y,
            av.z,
        ]))

        now = self.get_clock().now()
        elapsed = (now - self.last_log_time).nanoseconds / 1e9
        if elapsed < self.log_period_sec:
            return

        self.last_log_time = now
        if self.print_degrees:
            self.get_logger().info(
                "roll={:.2f} deg, pitch={:.2f} deg, yaw={:.2f} deg, "
                "gyro_y={:.4f} rad/s".format(
                    math.degrees(roll),
                    math.degrees(pitch),
                    math.degrees(yaw),
                    av.y,
                )
            )
        else:
            self.get_logger().info(
                "roll={:.4f}, pitch={:.4f}, yaw={:.4f}, gyro_y={:.4f}".format(
                    roll,
                    pitch,
                    yaw,
                    av.y,
                )
            )


def main():
    rclpy.init()
    node = ImuDebugNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()