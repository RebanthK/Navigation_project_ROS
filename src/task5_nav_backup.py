#!/usr/bin/env python3

from re import search
import rospy
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
from std_msgs.msg import String
import roslaunch
 
# Import some image processing modules:
import cv2
import pathlib
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import argparse

class Explorer():

    def __init__(self):
        self.node_name = "explorer"
        cli = argparse.ArgumentParser(description=f"Command-line interface for the '{self.node_name}' node.")
        cli.add_argument("-colour", metavar="COL", type=String,
            default="Blue", 
            help="The name of a colour (for example)")
        self.args = cli.parse_args(rospy.myargv()[1:])

        self.color_index = 0
        self.color_ranges = [
            ((95, 150, 100),(105, 255, 255),"blue"),
            ((0, 185, 100),(10, 255, 255),"red"),
            ((80, 185, 100),(90, 255, 255),"green"),
            ((25,150,100),(33, 255, 255),"yellow"),
        ]
        self.beacon_found = False
        

        for i in range(4):
            if self.color_ranges[i][2] == self.args.colour.data.lower(): 
                self.color_index = i

        #distance from walls
        self.safe_d = 0.4
 
        rospy.init_node(self.node_name, anonymous=True)
 
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.camera_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.rate = rospy.Rate(1) # hz
        self.vel_cmd = Twist()
        self.move = ""
        self.pmove = ""
        self.ctrl_c = False

        self.starttime = 0
        self.first_call = True

        self.front_arc = 0
        self.left_arc = 0

        self.img_capture = False
        self.m00 = 0
        self.m00_min = 100000
        
        
        rospy.on_shutdown(self.shutdown_hook)

    def show_and_save_image(self, img):
        img_name = "the_beacon"
        #cv2.imshow(img_name, img)
        #cv2.waitKey(0)
        base_path = pathlib.Path.home().joinpath("catkin_ws/src/team19/snaps/the_beacon.jpg")
        cv2.imwrite(str(base_path), img)
        print(f"Saved an image to '{base_path}'\n"
            f"image dims = {img.shape[0]}x{img.shape[1]}px\n"
            f"file size = {base_path.stat().st_size} bytes") 

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, _ = cv_img.shape
        crop_width = width
        crop_height = 50
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        if not self.img_capture:
            self.show_and_save_image(cv_img)
            print("SAVING image")
            self.img_capture = True

        mask_real = cv2.inRange(hsv_img, self.color_ranges[self.color_index][0], self.color_ranges[self.color_index][1])
        m_real = cv2.moments(mask_real)
            
        self.m00 = m_real["m00"]
        self.cy = m_real["m10"] / (m_real["m00"] + 1e-5)


        if self.m00 > self.m00_min:
            self.beacon_found = True
            self.show_and_save_image(cv_img)

        if not self.beacon_found:
            for i in range(4):
                if i == 0:
                    mask = cv2.inRange(hsv_img, self.color_ranges[i][0], self.color_ranges[i][1])
                else:
                    mask = mask + cv2.inRange(hsv_img, self.color_ranges[i][0], self.color_ranges[i][1]) 

            m = cv2.moments(mask)
                
            self.m00 = m["m00"]
            self.cy = m["m10"] / (m["m00"] + 1e-5)

            if self.m00 > self.m00_min:
                #cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
                self.show_and_save_image(cv_img)

    def callback_lidar(self, lidar_data):
        
        data = lidar_data.ranges
        self.front_arc = np.array(data[:32] + data[-32:])
        self.left_arc = np.array(data[45 - 20:45 + 20])
        self.right_arc = np.array(data[270 - 20:270 + 20])

        self.front_arc = self.front_arc[self.front_arc != 0.0].min()
        self.left_arc = self.left_arc[self.left_arc != 0.0].min()
        self.right_arc = self.right_arc[self.right_arc != 0.0].min()
        if rospy.get_rostime().secs < self.starttime + 180:
            if self.front_arc > self.safe_d and self.left_arc > self.safe_d:
                self.vel_cmd.linear.x = 0.13
                self.vel_cmd.angular.z = 0.4
                self.move = "forward_left"

            if self.front_arc > self.safe_d and self.left_arc <= self.safe_d:
                self.vel_cmd.linear.x = 0.26
                self.vel_cmd.angular.z = 0
                self.move = "forward"

            if self.front_arc <= self.safe_d and self.left_arc > self.safe_d:
                self.vel_cmd.linear.x = 0
                self.vel_cmd.angular.z = 0.4
                self.move = "left_turn"

            if self.front_arc <= self.safe_d and self.left_arc <= self.safe_d:
                if self.pmove == "left_turn":
                    self.vel_cmd.linear.x = 0
                    self.vel_cmd.angular.z = 0.4
                else:
                    self.move = "right_turn"
                    self.vel_cmd.linear.x = 0
                    self.vel_cmd.angular.z = -0.4
            

        else:
            self.ctrl_c = True

        self.vel_pub.publish(self.vel_cmd)
        self.pmove = self.move
 
    def main_loop(self):  
        self.starttime = rospy.get_rostime().secs
        map_saved = False
        while not self.ctrl_c:           
            if ((rospy.get_rostime().secs - self.starttime) % 30 == 0) and (not map_saved):
                self.map_path = pathlib.Path.home().joinpath("catkin_ws/src/team19/maps/task5_map")
                print(f"Saving map at time: {rospy.get_time()}...")
                self.launch = roslaunch.scriptapi.ROSLaunch()
                self.launch.start()
                node = roslaunch.core.Node(package="map_server", node_type="map_saver", args=f"-f {self.map_path}")
                process = self.launch.launch(node)
                map_saved = True
            
            if (rospy.get_rostime().secs - self.starttime) % 30 != 0:
                map_saved = False
            continue
            
        

    def shutdown_hook(self):
        self.vel_cmd.linear.x = 0.0 # m/s
        self.vel_cmd.angular.z = 0.0 # rad/s
 
        print("stopping the robot")
 
        # publish to the /cmd_vel topic to make the robot stop
        self.vel_pub.publish(self.vel_cmd)

if __name__ == '__main__':
    subscriber_instance = Explorer()
    subscriber_instance.main_loop()