#!/usr/bin/env python

import numpy as np
import sys
import cv2
import rospy
import message_filters

from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from human_detection.msg import box_list
from depth_filter import *

text_position = (10, 50)
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 0.75
font_color = (255, 255, 255)
line_type = 2


class obj_dist:
    def __init__(self):
        # Initializing ROS Topics
        self.bridge = CvBridge()
        self.dist_pub = rospy.Publisher('/obj_to_dist/human_distance', Image, queue_size=10)
        self.render_pub = rospy.Publisher('/obj_to_dist/render_human', Marker, queue_size=10)
        self.roi_pub = rospy.Publisher('/obj_to_dist/disappera', Image, queue_size=10)

        self.bbx_sub = message_filters.Subscriber('/human_detected_image/bounding_box', box_list)
        self.human_image_sub = message_filters.Subscriber('/human_detected_image/image', Image)
        self.depth_image_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
        # self.pcl_sub = message_filters.Subscriber('/camera/depth_registered/points', PointCloud2)

        ts = message_filters.ApproximateTimeSynchronizer([self.bbx_sub, self.human_image_sub,
                                                          self.depth_image_sub],
                                                         queue_size=10,
                                                         slop=1)
        ts.registerCallback(self.callback)

    def callback(self, bbx, image, depth):
        if bbx.length:
            cv_depth = self.bridge.imgmsg_to_cv2(depth, 'passthrough')
            cv_image = self.bridge.imgmsg_to_cv2(image, 'bgr8')

            for box in bbx.people_list:
                cood = self.get_human_distance(cv_depth, cv_image, box)
                self.render_human(cood)

    def get_human_distance(self, cv_depth, cv_image, box):
        roi_depth = cv_depth[box.ymin:box.ymax, box.xmin:box.xmax]

        jk = np.shape(roi_depth)
        jk = jk[0]*jk[1]
        filtered_depth = filter_background(roi_depth)
        _size = len(filtered_depth)

        print('size of original:  {}'.format(jk))
        print('size of filtered:  {}'.format(_size))
        print('')
        avg_distance = filtered_depth.sum() / _size / 1000

        x_meter = self.get_x_in_meters(box.xmin, box.xmax, avg_distance)
        cv2.putText(cv_image, '{} meters'.format(avg_distance),
                    (box.xmin, box.ymax - 100),
                    font,
                    font_scale,
                    font_color,
                    line_type)

        # self.roi_pub.publish(self.bridge.cv2_to_imgmsg(filtered_depth.data))
        self.roi_pub.publish(self.bridge.cv2_to_imgmsg(roi_depth))
        self.dist_pub.publish(self.bridge.cv2_to_imgmsg(cv_image))

        return x_meter, 0, avg_distance

    def render_human(self, cood):
        marker = Marker()
        marker.header.frame_id = 'camera_color_optical_frame'
        marker.header.stamp = rospy.Time.now()

        marker.id = 0
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration()
        marker.type = Marker.SPHERE

        marker.pose.position.x = cood[0]
        marker.pose.position.y = cood[1]
        marker.pose.position.z = cood[2]

        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.render_pub.publish(marker)

    def get_x_in_meters(self, xmin, xmax, z_i):
        # Tune z_c to get better value lol.
        z_c = 100
        ret_val = (z_i*(xmax+xmin-600.0))/(2*z_c)
        return ret_val


def main(args):
    rospy.init_node('obj_dist', anonymous=True)
    print("object_to_distance.py is running")
    _ = obj_dist()
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    # rospy.set_param("use_sim_time", 'true')
    main(sys.argv)
