#!/usr/bin/env python3

from concurrent.futures import thread
import threading

from click import command
import rospy
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

 
class Task2(): 
    def __init__(self):
        self.node_name = "task2_node"

        #distance from walls
        self.approach_distance = 0.7
        
        rospy.init_node(self.node_name, anonymous=True)
 
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_feedback)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(20) # hz
        self.vel_cmd = Twist()
        self.move = ""
        self.pmove = ""
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_hook)
        
    
    def callback_feedback(self, lidar_data):
        data = lidar_data.ranges
        front_arc = np.array(data[0:25] + data[-25:]).min()
        left_arc = np.array(data[45 - 5:45 + 25]).min()
        
        if front_arc > self.approach_distance and left_arc > self.approach_distance:
            self.vel_cmd.linear.x = 0.5
            self.vel_cmd.angular.z = 0.0
            self.move = "forward"

        if front_arc > self.approach_distance and left_arc <= self.approach_distance:
            self.vel_cmd.linear.x = 0.5
            self.vel_cmd.angular.z = 0.15
            self.move = "forward_right"

        if front_arc <= self.approach_distance and left_arc > self.approach_distance:
            self.vel_cmd.linear.x = 0.01
            self.vel_cmd.angular.z = 0.5
            self.move = "left_turn"

        if front_arc <= self.approach_distance and left_arc <= self.approach_distance:    
            self.vel_cmd.linear.x = 0.01
            self.vel_cmd.angular.z = -0.5
            self.move = "right_turn"

        print(self.move)

        self.vel_pub.publish(self.vel_cmd)
        self.pmove = self.move
        
        ## add_thread = threading.Thread(target = self.main_loop)
        ## add_thread.start()
       
        ## StartTime = rospy.get_rostime()
        ## while (rospy.get_rostime().secs - StartTime.secs) < 2:
            ## continue
        ## self.vel_cmd.linear.x = 0.0 
        ## self.vel_cmd.angular.z = 0.0 
        ## self.vel_pub.publish(self.vel_cmd)
        ## print("stopping the robot")

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
    subscriber_instance = Task2()
    subscriber_instance.main_loop()
