#!/usr/bin/env python

import numpy as np
import sys
import cv2
import rospy
import message_filters

sys.path.append('/home/jerry/Documents/workspaces/ROS_human_detection/src/human_detection')

from sensor_msgs import point_cloud2 as pcl
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from human_detection.msg import bounding_box, box_list

text_position = (10, 50)
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 0.75
font_color = (255, 255, 255)
line_type = 2

counter = 0

class obj_dist:
    def __init__(self):
        # Initializing ROS Topics
        self.bridge = CvBridge()
        self.dist_pub = rospy.Publisher('/obj_to_dist/human_distance', Image, queue_size=10)

        # self.bbx_sub = rospy.Subscriber('/human_detected_image/bounding_box', box_list, self.yea)
        # self.human_image_sub = rospy.Subscriber('/human_detected_image/image', Image, self.yea1)
        # self.depth_image_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.yea2)

        self.bbx_sub = message_filters.Subscriber('/human_detected_image/bounding_box', box_list)
        self.human_image_sub = message_filters.Subscriber('/human_detected_image/image', Image)
        self.depth_image_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)

        ts = message_filters.ApproximateTimeSynchronizer([self.bbx_sub, self.human_image_sub, self.depth_image_sub],
                                                         queue_size=10,
                                                         slop=1)
        ts.registerCallback(self.callback)

    def callback(self, bbx, image, depth):
        print('Called!')
        global counter
        if bbx.length:
            cv_depth = self.bridge.imgmsg_to_cv2(depth, 'passthrough')
            cv_image = self.bridge.imgmsg_to_cv2(image, 'bgr8')
            box = bbx.people_list[0]
            roi_depth = cv_depth[box.xmin:box.xmax, box.ymin:box.ymax]
            x = box.xmax - box.xmin
            y = box.ymax - box.ymin

            avg_distance = roi_depth[roi_depth <= 2500].sum() / (x*y) / 1000
            cv2.putText(cv_image, '{} meters'.format(avg_distance),
                        (box.xmin, box.ymax-100),
                        font,
                        font_scale,
                        font_color,
                        line_type)
            self.dist_pub.publish(self.bridge.cv2_to_imgmsg(cv_image))
            counter += 1


def main(args):
    rospy.init_node('obj_dist', anonymous=True)
    _ = obj_dist()
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    # rospy.set_param("use_sim_time", 'true')
    main(sys.argv)
