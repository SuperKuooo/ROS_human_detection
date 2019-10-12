#!/usr/bin/env python

import numpy as np
import sys
import cv2

import rospy
import pyrealsense2 as rs2
import message_filters
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError


class obj_dist:
    def __init__(self):
        # Initializing ROS Topics
        self.bridge = CvBridge()
        dist_pub = rospy.Publisher('/obj_to_dist', Image, queue_size=10)
        self.human_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        self.depth_sub = message_filters.Subscriber('/camera/depth_registered/points', PointCloud2)

        ts = message_filters.TimeSynchronizer([self.human_sub, self.depth_sub], 10)
        ts.registerCallback(self.callback)

    def callback(self, Image, Depth):
        print('wtf')

def main(args):
    _ = obj_dist()
    rospy.init_node('obj_dist', anonymous=True)
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

