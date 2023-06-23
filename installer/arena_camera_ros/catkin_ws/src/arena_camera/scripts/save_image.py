#!/usr/bin/env python

import rospy
import rospkg
import cv2
from cv_bridge import CvBridge, CvBridgeError
from actionlib import SimpleActionClient, SimpleActionServer
from camera_control_msgs.msg import GrabImagesAction, GrabImagesGoal
from camera_control_msgs.msg import GrabAndSaveImageAction, GrabAndSaveImageResult
from sensor_msgs.msg import Image
from os.path import isfile, join

class SaveImage():
    """
    This nodes saves a single depth image for now
    """

    def __init__(self):
        camera_name = rospy.get_param('~camera_name', '')
        if not camera_name:
            rospy.logwarn("No camera name given! Assuming 'arena_camera_node' as"
                          " camera name")
            camera_name = '/arena_camera_node'
        else:
            rospy.loginfo('Camera name is: ' + camera_name)

        self.cam_image = None

        rospack = rospkg.RosPack()
        self._fpd_path = rospack.get_path('arena_camera')
        rospy.loginfo(f"Path: {self._fpd_path}")
        #rospy.Subscriber("/arena_camera_node/image_raw", Image, self.callback, queue_size=1)
        rospy.Subscriber("/arena_camera_node/image_raw/compressedDepth", Image, self.callback, queue_size=1)
        
    def callback(self, img):
        self.cam_image = CvBridge().imgmsg_to_cv2(img, desired_encoding="32FC1")
        #self.cam_image = CvBridge().imgmsg_to_cv2(img, desired_encoding='passthrough')
        cv2.imwrite(join("/temp", "CapturedImage.png"), self.cam_image)
        rospy.loginfo("Image Saved!")

    def spin(self):
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("save_image")
    grab_and_save_img_as_node = SaveImage()
    grab_and_save_img_as_node.spin()
