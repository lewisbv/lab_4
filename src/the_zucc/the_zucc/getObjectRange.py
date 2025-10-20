# Lewis Busch-Vogel and Akash Iyer
# Lab 4 - Get object range, sort through lidar data and publish nearest distance and angle

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray

class GetObjectRange(Node):
    def __init__(self):	
        super().__init__('getObjectRange')
        self.get_logger().info("getObjectRange node started, waiting for lidar messages...")

        self.subscription = self.create_subscription(LaserScan, "/scan", self.lidar_callback, qos_profile_sensor_data)
        self.publisher_ = self.create_publisher(Float32MultiArray, "avoid_obj", 1)

    def lidar_callback(self, msg: LaserScan):
        if not msg.ranges:
            msg = Float32MultiArray()
            msg.data = []
            self.publisher_.publish(msg)
            return

        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        ranges_new = []
        angles_new = []
        i = 0

        for range in ranges:
            if not np.isnan(range) and not np.isinf(range):
                ranges_new.append(range)
                angles_new.append(angles[i])

            i += 1

        ranges = np.array(ranges_new)
        angles = np.array(angles_new)
        
        # self.get_logger().info(f"nearest object: {min(ranges)} at angle {angles[np.argmin(ranges)]}")
        
        q = len(ranges) // 7
        ranges = np.concatenate((ranges[:q], ranges[-q:]))
        angles = np.concatenate((angles[:q], angles[-q:]))

        avoid_msg = Float32MultiArray()
        avoid_msg.data = [float(min(ranges)), float(angles[np.argmin(ranges)])]
    
        self.publisher_.publish(avoid_msg)


def main():
    rclpy.init()
    node = GetObjectRange()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
