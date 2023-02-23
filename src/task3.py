#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
 
class MazeSolver(): 
    def __init__(self):
        self.node_name = "maze_solver"

        #distance from walls
        self.safe_d = 0.5
 
        rospy.init_node(self.node_name, anonymous=True)
 
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(20) # hz
        self.vel_cmd = Twist()
        self.move = ""
        self.pmove = ""
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_hook)

 
    def callback_lidar(self, lidar_data):
        data = lidar_data.ranges
        front_arc = np.array(data[0:30] + data[-30:]).min()
        left_arc = np.array(data[45 - 15:45 + 15]).min()

        if front_arc > self.safe_d and left_arc > self.safe_d:
            self.vel_cmd.linear.x = 0.15
            self.vel_cmd.angular.z = 0.5
            self.move = "forward_left"

        if front_arc > self.safe_d and left_arc <= self.safe_d:
            self.vel_cmd.linear.x = 0.3
            self.vel_cmd.angular.z = 0
            self.move = "forward"

        if front_arc <= self.safe_d and left_arc > self.safe_d:
            self.vel_cmd.linear.x = 0.01
            self.vel_cmd.angular.z = 0.5
            self.move = "left_turn"

        if front_arc <= self.safe_d and left_arc <= self.safe_d:
            if self.pmove == "left_turn":
                return
            else:
                self.move = "right turn"
                self.vel_cmd.linear.x = 0
                self.vel_cmd.angular.z = -0.5
        
        print(self.move)

        self.vel_pub.publish(self.vel_cmd)
        self.pmove = self.move
 
    def main_loop(self):
        rospy.spin()
 
    def shutdown_hook(self):
        self.vel_cmd.linear.x = 0.0 # m/s
        self.vel_cmd.angular.z = 0.0 # rad/s
 
        print("stopping the robot")
 
        # publish to the /cmd_vel topic to make the robot stop
        self.vel_pub.publish(self.vel_cmd)
        self.ctrl_c = True
 
 
 
if __name__ == '__main__':
    subscriber_instance = MazeSolver()
    subscriber_instance.main_loop()
