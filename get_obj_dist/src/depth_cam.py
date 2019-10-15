#!/usr/bin/env python

import sys
import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

text_position = (10, 50)
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 0.75
font_color = (255, 255, 255)
line_type = 2


class obj_dist:
    def __init__(self):
        # Initializing ROS Topics
        self.bridge = CvBridge()
        self.measure_pub = rospy.Publisher('/obj_to_dist/measuring', Image, queue_size=10)

        self.d_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.fixed_distance)


    def fixed_distance(self, data):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(data, 'passthrough')
        except CvBridgeError as e:
            print(e)
            return

        cv2.rectangle(cv_depth, (250, 190), (350, 290), (255, 0, 0), 5)
        roi_depth = cv_depth[250:350, 190:290]
        # roi_depth = cv_depth[250:255, 190:195]

        # Ten thousand pixels, divide by 1000 for meters
        # avg_distance = np.sum(roi_depth[:, :])/10000/1000
        filtered_depth = roi_depth[roi_depth <= 2900]
        _size = len(filtered_depth)
        avg_distance = filtered_depth.sum() / _size / 1000

        cv2.putText(cv_depth, 'Object Distance: {} meters'.format(avg_distance),
                    text_position,
                    font,
                    font_scale,
                    font_color,
                    line_type)

        try:
            ret_img = self.bridge.cv2_to_imgmsg(cv_depth, 'passthrough')
        except CvBridgeError as e:
            print(e)
            return

        self.measure_pub.publish(ret_img)


def main(args):
    rospy.init_node('obj_dist', anonymous=True)
    _ = obj_dist()
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.set_param("use_sim_time", 'true')
    main(sys.argv)
