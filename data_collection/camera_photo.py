#!/usr/bin/env python3
import os
import cv2                                          # to display the images
from cv_bridge import CvBridge, CvBridgeError       # to convert ros image messages to OpenCV images
import rospy                                        # ROS Python Interface
from sensor_msgs.msg import Image
import time
import numpy as np

# Global Parameters
RATE = 5 # images per second
NO_OF_IMAGES = 100 # total number of images to be taken

class CamPhoto(object):
    
    """
        The following code will provide the Camera data of the MiRO
        and use the camera data for taking photos
        
        Attributes:
            default: 'both' is set as default where it uses both camera to take photo
                     'left' is used to take photo from left camera
                     'right' is used to take photo from right camera
    """

    def __init__(self, default = "both"):
        # rospy.init_node("cam_subscriber")
        # cv bridge is used for converting ros img data to data that can be used by open cv
        self.bridge = CvBridge()
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        # take photo from right camera
        if default == "right":
            cam_topic = "/sensors/camr"
            self.cam_sub = rospy.Subscriber(topic_base_name + cam_topic, Image, self.callback_right)
        
        # take photo from left camera
        elif default == "left":
            cam_topic = "/sensors/caml"
            self.cam_sub = rospy.Subscriber(topic_base_name + cam_topic, Image, self.callback_left)
        
        # take photo from both camera
        else:
            cam_topic = "/sensors/camr"
            self.cam_sub = rospy.Subscriber(topic_base_name + cam_topic, Image, self.callback_right)
            cam_topic = "/sensors/caml"
            self.cam_extra_sub = rospy.Subscriber(topic_base_name + cam_topic, Image, self.callback_left)
        self.cam_data = {'left': None, 'right': None}
        rospy.sleep(0.5)
        self.rate = rospy.Rate(RATE)
         
    # callback function camr
    def callback_left(self, data):
        """
            callback_left function is used to update the dictionary camaera data
            for left camera

            :param data: messages from left camera
        """
        try:
            # convert ros img data to open cv images
            self.cam_data['left'] = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
           print(e)
    
    
    def callback_right(self, data):
        """
            callback_right function is used to update the dictionary camaera data
            for right camera

            :param data: messages from right camera
        """
        try:
            # convert ros img data to open cv images
            self.cam_data['right'] = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
           print(e)

    def save_data(self):
        """
            The following function save images for you.
        """
        counter = 0
        while counter < NO_OF_IMAGES:
            camera_left = self.cam_data['left']
            camera_right = self.cam_data['right']
            if not camera_left is None:      
                cv2.imwrite('camera_left/image_' + str(counter) + '.png', camera_left)
                cv2.waitKey(3) 
            if not camera_right is None:
                cv2.imwrite('camera_right/image_' + str(counter) + '.png', camera_right)
                cv2.waitKey(3)
            self.rate.sleep()
            counter += 1
            print(counter)

    def read_data(self):
        camera_left = np.array(self.cam_data['left'])
        camera_right = np.array(self.cam_data['right'])
        return camera_left, camera_right

#camera = CamPhoto()
#camera.save_data()