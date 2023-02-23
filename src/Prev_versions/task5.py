#!/usr/bin/env python3

import rospy
from pathlib import Path

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

class Task5():

    #save img (use with cvbridge_interface.imgmsg_to_cv2)
    #from week 6
    def show_and_save_image(img):
        image_path = "/home/student/myrosdata/beacon.jpg"

        cv2.imshow("beacon", img)
        cv2.waitKey(0)

        cv2.imwrite(str(image_path), img)
        print(f"Saved an image to '{image_path}'\n"
            f"image dims = {img.shape[0]}x{img.shape[1]}px\n"
            f"file size = {image_path.stat().st_size} bytes")


    #To be called at the start with the colour taken from launch input
    #Note - need to change mask values
    def maskSelect(self, colour, hsv_img):     
        if colour == "blue":
            lower = (115, 224, 100)
            upper = (130, 255, 255)
            return cv2.inRange(hsv_img, lower, upper)
        elif colour == "red":
            lower = (-1, 240, 100)
            upper= (180, 255, 255)
            return cv2.inRange(hsv_img, lower, upper)
        elif colour == "green":
            lower = (45, 140, 100)
            upper = (63, 256, 255)
            return cv2.inRange(hsv_img, lower, upper)
        elif colour == "yellow":
            lower = (75, 112, 100)
            upper= (93, 255, 255)
            return cv2.inRange(hsv_img, lower, upper)
        




# if __name__ == '__main__':
#     subscriber_instance = Task5()
#     subscriber_instance.main_loop()
