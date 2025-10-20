# Lewis Busch-Vogel and Akash Iyer
# Lab 4 - Process waypoint data, publish distance and angle to text goal

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point

class ProcessWaypoints(Node):
    def __init__(self):	
        super().__init__('processWaypoints')
        # self.subscription = self.create_subscription(Odometry, '/odom', self.waypoints_callback, 1)
        self.publisher_ = self.create_publisher(Float32MultiArray, "go_to_obj", 1)

        self.get_logger().info("processWaypoints node started, waiting for image messages...")

        self.waypoints = [(1.5, 0.0), (1.5, 1.4), (0.0, 1.4)]
        # self.waypoints = [(1.5, 0.0)]
        self.switch_waypoint = True
        self.curr_waypoint_index = 0

        # State (for the update_Odometry code)
        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0
        self.globalPos = Point()

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            1)
        self.odom_sub  # prevent unused variable warning

    def odom_callback(self, data):
        self.update_Odometry(data)

    def update_Odometry(self,Odom):
        position = Odom.pose.pose.position
        
        #Orientation uses the quaternion aprametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z
        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

        #We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang

        if self.curr_waypoint_index >= len(self.waypoints):
            go_msg = Float32MultiArray()
            go_msg.data = []
            self.publisher_.publish(go_msg)
            return

        curr_waypoint = self.waypoints[self.curr_waypoint_index]

        e_x = curr_waypoint[0] - self.globalPos.x
        e_y = curr_waypoint[1] - self.globalPos.y

        if abs(e_x) < 0.05 and abs(e_y) < 0.05:
            if self.switch_waypoint:
                self.switch_waypoint = False
                self.curr_waypoint_index += 1

            if self.curr_waypoint_index >= len(self.waypoints):
                go_msg = Float32MultiArray()
                go_msg.data = []
                self.publisher_.publish(go_msg)
                return

            curr_waypoint = self.waypoints[self.curr_waypoint_index]

            e_x = curr_waypoint[0] - self.globalPos.x
            e_y = curr_waypoint[1] - self.globalPos.y

        else:
            self.switch_waypoint = True

        distance = np.sqrt(e_x**2 + e_y**2)
        angle = np.arctan2(e_y, e_x)

        def wrap(a):
            return (a + np.pi) % (2*np.pi) - np.pi

        angle = wrap(angle - self.globalAng)

        self.get_logger().info(f"error angle: {angle}, global angle: {self.globalAng}, pos: ({self.globalPos.x}, {self.globalPos.y}), waypoint: {curr_waypoint}")
        
        switch_val = 1.0

        if (not self.switch_waypoint):
            switch_val = 0.0

        go_msg = Float32MultiArray()
        go_msg.data = [float(distance), float(angle), float(self.globalAng), float(self.switch_waypoint), switch_val]
        self.publisher_.publish(go_msg)    


def main():
    rclpy.init()
    node = ProcessWaypoints()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
