#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy

class ImuRemapNode(Node):
    def __init__(self):
        super().__init__('imu_remap_node')
        # Publisher for transformed IMU
        self.imu_pub = self.create_publisher(Imu, '/remapped_imu', 10)
        # Subscriber to original IMU
        qos = QoSProfile(depth=0, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.imu_sub = self.create_subscription(
            Imu,
            '/camera/camera/imu',
            self.imu_callback,
            
        qos)

    def imu_callback(self, msg):
        imu_msg = Imu()
        # Copy header
        imu_msg.header = msg.header

        # Remap angular velocity
        imu_msg.angular_velocity.x = msg.angular_velocity.z
        imu_msg.angular_velocity.y = msg.angular_velocity.x
        imu_msg.angular_velocity.z = msg.angular_velocity.y
        imu_msg.angular_velocity_covariance = msg.angular_velocity_covariance

        # Remap linear acceleration
        imu_msg.linear_acceleration.x = msg.linear_acceleration.z
        imu_msg.linear_acceleration.y = msg.linear_acceleration.x
        imu_msg.linear_acceleration.z = msg.linear_acceleration.y
        imu_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

        # Keep orientation as is
        imu_msg.orientation = msg.orientation
        imu_msg.orientation_covariance = msg.orientation_covariance

        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuRemapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
