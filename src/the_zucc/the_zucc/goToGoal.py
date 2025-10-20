# Lewis Busch-Vogel and Akash Iyer
# Lab 4 - Go to goal, process error information from processWaypoints and getObjectRange to drive robot

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class GoToGoal(Node):
    def __init__(self):	
        super().__init__('goToGoal')
        self.get_logger().info("goToGoal node started, waiting for DetectObject messages...")

        self.subscription = self.create_subscription(Float32MultiArray, "go_to_obj", self.go_callback, 1)
        self.subscription = self.create_subscription(Float32MultiArray, "avoid_obj", self.avoid_callback, 1)

        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 5)

        self.state = 0 # 0 = go to goal, 1 = avoid object, 2 = stuck in concave corner
        self.stay = False

        self.kp_go_x = 0.8
        self.kp_go_w = 0.4

        self.kp_avoid_x = 0.1
        self.kp_avoid_w = 0.1

        self.w = 0.0
        self.x = 0.0

    def post_twist(self):
        twist = Twist()
        twist.linear.x = self.x
        twist.angular.z = self.w
        self.publisher_.publish(twist)

    def go_callback(self, go_msg: Float32MultiArray):
        if len(go_msg.data) == 0:
            self.w = 0.0
            self.x = 0.0
            self.post_twist()

            return

        distance = go_msg.data[0]
        angle = go_msg.data[1]
        global_angle = go_msg.data[2]
        switch_waypoint = go_msg.data[3]

        if switch_waypoint == 0.0:
            self.stay = True

        if self.state == 0:
            e_angle = angle
            w = self.kp_go_w * e_angle

            if w > 2:
                self.w = 2
            elif w < -2:
                self.w = -2
            else:
                self.w = w
            
            if self.stay:
                self.x = 0.0

                if abs(angle) < 0.2:
                    self.stay = False
                
            else:
                e_distance = distance
                x = self.kp_avoid_x * e_distance + 0.05

                if x > .1:
                    self.x = .1
                elif x < -.1:
                    self.x = -.1
                else:
                    self.x = x

        self.post_twist()
        
    def avoid_callback(self, avoid_msg: Float32MultiArray):
        if avoid_msg.data == []:
            self.w = 0.0
            self.x = 0.0
            self.post_twist()
            return
        
        distance = avoid_msg.data[0]
        angle = avoid_msg.data[1]
        
        if distance < 0.4:
            self.state = 1

            e_angle = angle - np.pi
            w = self.kp_avoid_w * e_angle

            if w > 1.5:
                self.w = 1.5
            elif w < -1.5:
                self.w = -1.5
            else:
                self.w = w

            e_distance = distance - 0.2
            x = self.kp_avoid_x * e_distance

            if x > .15:
                self.x = .15
            elif x < -.15:
                self.x = -.15
            else:
                self.x = x

        else:
            self.state = 0
        
        self.post_twist()

        # self.get_logger().info(f"linear speed: {self.x}, angular speed: {self.w}, distance: {distance}, angle: {angle}")


def main():
    rclpy.init()
    node = GoToGoal()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
