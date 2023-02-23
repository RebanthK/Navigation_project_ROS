#!/usr/bin/env python3
# a template for the move_square exercise

from logging import shutdown
import rospy
# import the Twist message for publishing velocity commands:
from geometry_msgs.msg import Twist
# import the Odometry message for subscribing to the odom topic:
from nav_msgs.msg import Odometry, OccupancyGrid
# import the function to convert orientation from quaternions to angles:
from tf.transformations import euler_from_quaternion
# import some useful mathematical operations (and pi), which you may find useful:
from math import sqrt, pow, pi, floor
import numpy as np

class Task5:

    def callback_odom(self, odom_data):
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
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw

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
    
    def callback_function(self, map_data: OccupancyGrid):

        self.map_data = map_data
        self.map_grid = np.array(map_data.data).reshape((map_data.info.height, map_data.info.width))
        print(self.map_grid.shape)
        print(np.where(self.map_grid==-1)[0][0])
        self.height_odom = self.map_grid.shape[0] * map_data.info.resolution
        self.width_odom = self.map_grid.shape[1] * map_data.info.resolution
        self.origin_index = ((self.height_odom/2) * (1/map_data.info.resolution), (self.width_odom/2) * (1/map_data.info.resolution))


    def odom_pos_to_map_index(self):
        map_x = self.x - self.map_data.info.origin.position.x
        map_y = self.y - self.map_data.info.origin.position.y
        
        index_x = floor(map_x * 1/self.map_data.info.resolution)
        index_y = floor(map_y * 1/self.map_data.info.resolution)

        return (index_y, index_x)

    def map_index_to_odom_pos(self, indices):
        index_y, index_x = indices
        map_y = (index_y * self.map_data.info.resolution) + self.map_data.info.origin.position.y
        map_x = (index_x * self.map_data.info.resolution) + self.map_data.info.origin.position.x

        return (map_x, map_y)

    def __init__(self):
        node_name = "task5navigation"
        # a flag if this node has just been launched
        self.startup = True

        # This might be useful in the main_loop() to switch between turning and moving forwards...?
        self.turn = False

        # setup a cmd_vel publisher and an odom subscriber:
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/map", OccupancyGrid, self.callback_function)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback_odom)

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        # variables to use for the "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0
        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(10) # hz

        self.vel = Twist()
        self.check_stop = False

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {node_name} node has been initialised...")

    def shutdownhook(self):
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):
        self.pub.publish(Twist())
        self.ctrl_c = True
    

    def main_loop(self):
       
        while not self.ctrl_c:
            print(self.real_world_position_to_map_index())


if __name__ == '__main__':
    movesquare_instance = Task5()
    try:
        movesquare_instance.main_loop()
    except rospy.ROSInterruptException:
        pass