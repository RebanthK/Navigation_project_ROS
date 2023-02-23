#! /usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
from doctest import FAIL_FAST
from time import sleep
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image
from math import sqrt, pow, pi, atan, degrees, radians
# import the Odometry message for subscribing to the odom topic:
from nav_msgs.msg import Odometry
# import the function to convert orientation from quaternions to angles:
from tf.transformations import euler_from_quaternion
# Import the tb3 modules (which needs to exist within the "week6_vision" package)
from tb3 import Tb3Move
#import for goal node
from geometry_msgs.msg import PoseStamped

from sensor_msgs.msg import LaserScan

import numpy as np

class colour_search(object):

    def odom_callback_function(self, odom_data):
        # obtain the orientation co-ords:
        or_x = odom_data.pose.pose.orientation.x
        or_y = odom_data.pose.pose.orientation.y
        or_z = odom_data.pose.pose.orientation.z
        or_w = odom_data.pose.pose.orientation.w

        # obtain the position co-ords:
        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y

        # convert orientation co-ords to roll, pitch & yaw (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion([or_x, or_y, or_z, or_w], 'sxyz')

        # We are only interested in the x, y and theta_z odometry data for this
        # robot, so we only assign these to class variables (so that we can 
        # access them elsewhere within our Square() class):
        self.x = round(pos_x, 4)
        self.y = round(pos_y, 4)
        self.theta_z = round(degrees(yaw) + 180,4)

        # If this is the first time that this callback_function has run, then 
        # obtain a "reference position" (used to determine how far the robot has moved
        # during its current operation)
        if self.startup:
            # don't initialise again:
            self.startup = False
            # set the reference position:
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

    def callback_lidar(self, lidar_data):
        data = lidar_data.ranges
        self.front_arc = np.array(data[0:45] + data[-45:]).min()

    def __init__(self):
        node_name = "turn_and_face"
        self.startup = True
        rospy.init_node(node_name)

        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()
        self.odom_subscriber = rospy.Subscriber("odom", Odometry, self.odom_callback_function)
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
       
        self.goal_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
        self.goal = PoseStamped()
        self.robot_controller = Tb3Move()
        self.turn_vel_fast = 0.5
        self.turn_vel_slow = 0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        # define the robot pose variables and set them all to zero to start with:
        # variables to use for the "current position":
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        # variables to use for the "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0
        
        self.initialise = True
        self.stage = "Initialize"
        self.output = ""
        self.color_index = 0
        self.color_ranges = [
            ((115, 224, 100),(130, 255, 255),"blue"),
            ((0, 185, 100),(10, 255, 255),"red"),
            ((33, 150, 100),(70, 255, 255),"green"),
            ((75, 150, 100),(100, 255, 255),"turquoise"),
            ((25,150,100),(33, 255, 255),"yellow"),
            ((135,174,100),(170,255,255),"purple")
        ]
        self.route = ""
        self.routes = [
            ("a",
            [
                #rotation points to next point 
                (-2.0680, -1.9736, False, 0,1),
                (-2.04, -1.5252, False, 0,-1),
                (-1, -1.3, False, 0,-1),
                (0.2, -0.8, False, 0,1),
                (0.1, -0.8, True, 290,1),
                (-0.4, -0.8, False, 0,1),
                (-1.2, 0, False, 0,1),
                (-1.9, 1.3, False, 0,-1),
                (-0.3, 1.53, False, 0,-1),
                (0.15, 1.9, True, 320,1),
            ],
            90
            ),
            ("b",
            [
                (-1.2407, 2.0662, False, 0,1),
                (-1.7870, 2.0672, False, 0,-1),
                (-1.7, 1.25, False, 0,1),
                (-0.3, 1.2, False, 0,1),
                (0.15, 1.9, True, 290,1),
                (1.62, 1.05, False, 0,1),
                (1.4, -0.6, False, 0,1),
                (1.5, -1.3, False, 0,-1),
                (-0.39, -0.62, True, 355,-1)
            ],
            90
            ),
            ("c",
            [
                (2.06995, 1.97302, False, 0,1),
                (1.98, 1.06, True, 150, -1),
                (1.4, -0.6, False, 0,1),
                (1.5, -1.3, False, 0,-1),
                (-0.39, -0.62, True, 359,-1)
            ],
            180
            )
        ]
        self.coords = []

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(50)
        
        self.m00 = 0
        self.m00_min = 100000
        self.move_rate = "" # fast, slow or stop
        self.stop_counter = 0

    def shutdown_ops(self):
        self.robot_controller.set_move_cmd(0.0, 0)
        self.robot_controller.publish()
        cv2.destroyAllWindows()
        self.ctrl_c = True
    
    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 50
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((2*height/4) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        if self.stage == "Searching for color":
            for i in range(4):
                mask = cv2.inRange(hsv_img, self.color_ranges[i][0], self.color_ranges[i][1])
                m = cv2.moments(mask)                
                self.m00 = m["m00"]
                self.cy = m["m10"] / (m["m00"] + 1e-5)

                if self.m00 > self.m00_min:
                    self.stage = "SEARCH INITIATED"
                    self.output = f"SEARCH INITIATED: The target beacon colour is {self.color_ranges[i][2]}."
                    self.color_index = i
                    print(self.output)
                    break
                     
        if self.stage == "SEARCH INITIATED":
            mask = cv2.inRange(hsv_img, self.color_ranges[self.color_index][0], self.color_ranges[self.color_index][1])
            m = cv2.moments(mask)                
            self.m00 = m["m00"]
            self.cy = m["m10"] / (m["m00"] + 1e-5)
            
            if self.m00 > self.m00_min:
                self.output = "TARGET DETECTED: Beaconing initiated."
                cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)


    
    def spin_check(self, target_yaw, rotation):
        starttime = rospy.get_rostime().secs

        while rospy.get_rostime().secs <= starttime + ((2*(radians(target_yaw)))):
            if self.m00 > self.m00_min:
                # blob detected
                if self.cy >= 560-100 and self.cy <= 560+100:
                    if self.move_rate == "slow":
                        self.move_rate = "stop"
                        self.stop_counter = 20
                else:
                    self.move_rate = "slow"
            else:
                self.move_rate = "fast"
            if self.move_rate == "fast":
                self.robot_controller.set_move_cmd(0.0, (rotation * (self.turn_vel_fast)))
            elif self.move_rate == "slow":
                self.robot_controller.set_move_cmd(0.0, (rotation * (0.2)))
            elif self.move_rate == "stop":
                self.robot_controller.set_move_cmd(0.0, 0)
                self.robot_controller.publish()
                self.stage = "BEACON FOUND"
                print("TARGET DETECTED: Beaconing initiated.")
                break
            else:
                self.robot_controller.set_move_cmd(0.0, (rotation * (self.turn_vel_fast)))
            self.robot_controller.publish()
        self.robot_controller.set_move_cmd(0.0, 0)
        self.robot_controller.publish()

    def findroute(self):
        for route in self.routes:
            if (self.x > (route[1][0][0] - 0.1) and self.x < (route[1][0][0] + 0.1)) and (self.y > (route[1][0][1] - 0.1) and self.y < (route[1][0][1] + 0.1)):
                self.route = route[0]

    def move_to(self, target_x, target_y, rotation):
        if target_x > self.x and target_y > self.y:
            angle = degrees(atan(((target_y - self.y)/(target_x - self.x))))
            self.turn_to((180 + angle), rotation)
            self.move_straight_to(target_x, target_y)
            return
        if target_x < self.x and target_y > self.y:
            angle = degrees(atan(((target_y - self.y)/(self.x - target_x))))
            self.turn_to((360 - angle), rotation)
            self.move_straight_to(target_x, target_y)
            return
        if target_x > self.x and target_y < self.y:
            angle = degrees(atan(((target_y - self.y)/(self.x - target_x))))
            self.turn_to((180 - angle), rotation)
            self.move_straight_to(target_x, target_y)
            return
        if target_x < self.x and target_y < self.y:
            angle = degrees(atan(((self.y - target_y)/(self.x - target_x))))
            self.turn_to(angle, rotation)
            self.move_straight_to(target_x, target_y)
            return

    def move_straight_to(self, target_x, target_y):
        prev_dist = 10000
        current_dist = sqrt(pow((target_x - self.x), 2) + pow((target_y - self.y), 2))

        while (prev_dist + 0.01 >= current_dist):
            current_dist = sqrt(pow((target_x - self.x), 2) + pow((target_y - self.y), 2))
            self.robot_controller.set_move_cmd(0.25, 0.0)
            self.robot_controller.publish()
            if current_dist < prev_dist:
                prev_dist = current_dist
        self.robot_controller.set_move_cmd(0.0, 0.0)
        self.robot_controller.publish()  

    def turn_to(self, target_yaw, rotation):
        while not (((target_yaw - 2) < self.theta_z) and ((target_yaw + 2) > (self.theta_z))):
            self.robot_controller.set_move_cmd(0.0, (rotation * (0.7)))
            self.robot_controller.publish()
        self.robot_controller.set_move_cmd(0.0, 0)
        self.robot_controller.publish()

    def main(self):
        run = False
        while not self.ctrl_c:
            if (not self.startup) and (self.stage == "Initialize") :
                self.findroute()
                self.stage = "Startup Finished"
            if self.stage == "Startup Finished":
                # rotate robot 180 deg
                for route in self.routes:
                    if route[0] == self.route:
                        self.turn_to(route[2], 1)
                        self.stage = "Searching for color"                
            if self.stage == "SEARCH INITIATED":
                for route in self.routes:
                    if route[0] == self.route:
                        self.coords = route[1]
                for coord in self.coords:
                    if coord == self.coords[0]:
                        continue
                    self.move_to(coord[0], coord[1], coord[4])
                    if coord[2]:
                        self.spin_check(coord[3], coord[4])
                        if self.stage == "BEACON FOUND":
                            break
                self.shutdown_ops()
            if self.stage == "BEACON FOUND":
                while self.front_arc > 0.35:
                    self.robot_controller.set_move_cmd(0.3, 0)
                    self.robot_controller.publish()
                self.robot_controller.set_move_cmd(0.0, 0)
                self.robot_controller.publish()
                print("BEACONING COMPLETE: The robot has now stopped.")
                self.shutdown_ops()


if __name__ == "__main__":
    search_instance = colour_search()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass