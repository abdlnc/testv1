#!/usr/bin/env python3
# Copyright 2025 Miguel Caluag
# Licensed under the Apache License, Version 2.0
# http://www.apache.org/licenses/LICENSE-2.0

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import math

MPU6050_ADDR = 0x68

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(MPU6050_ADDR, 0x6B, 0)
        self.timer = self.create_timer(0.1, self.publish_imu_data)
        self.get_logger().info('MPU6050 IMU node started.')

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(MPU6050_ADDR, addr)
        low = self.bus.read_byte_data(MPU6050_ADDR, addr + 1)
        value = (high << 8) | low
        if value > 32768:
            value -= 65536
        return value

    def publish_imu_data(self):
        imu = Imu()

        acc_x = self.read_raw_data(0x3B) / 16384.0
        acc_y = self.read_raw_data(0x3D) / 16384.0
        acc_z = self.read_raw_data(0x3F) / 16384.0

        gyro_x = self.read_raw_data(0x43) / 131.0
        gyro_y = self.read_raw_data(0x45) / 131.0
        gyro_z = self.read_raw_data(0x47) / 131.0

        imu.linear_acceleration.x = acc_x * 9.81
        imu.linear_acceleration.y = acc_y * 9.81
        imu.linear_acceleration.z = acc_z * 9.81

        imu.angular_velocity.x = math.radians(gyro_x)
        imu.angular_velocity.y = math.radians(gyro_y)
        imu.angular_velocity.z = math.radians(gyro_z)

        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'

        self.publisher_.publish(imu)

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
