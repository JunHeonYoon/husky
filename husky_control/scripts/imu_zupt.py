#!/usr/bin/env python3
"""
Software ZUPT node.
When IMU angular_velocity.z is below threshold, it's gyro bias, not real rotation.
Zero it out to prevent yaw drift. Same logic as SensorConnect ZUPT (angular rate based).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuZupt(Node):
    def __init__(self):
        super().__init__('imu_zupt')
        self.declare_parameter('input_topic',      'imu/data')
        self.declare_parameter('output_topic',     'imu/data_zupt')
        self.declare_parameter('gyro_threshold',   0.0174)  # rad/s

        input_topic  = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._threshold = self.get_parameter('gyro_threshold').value

        self.create_subscription(Imu, input_topic, self._imu_cb, 10)
        self._pub = self.create_publisher(Imu, output_topic, 10)

    def _imu_cb(self, msg):
        if abs(msg.angular_velocity.z) < self._threshold:
            msg.angular_velocity.z = 0.0
            cov = list(msg.angular_velocity_covariance)
            cov[8] = 1e-9
            msg.angular_velocity_covariance = tuple(cov)
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = ImuZupt()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
