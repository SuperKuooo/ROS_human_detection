#!/usr/bin/env python

import numpy as np
import sys
import cv2
import rospy
import message_filters

from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from human_detection.msg import box_list
from utils import *

text_position = (10, 50)
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 0.75
font_color = (255, 255, 255)
line_type = 2
size_of_moving_avg = 15


class obj_dist:
    def __init__(self):
        # Declare functional variables
        self.bridge = CvBridge()
        self.marker_array = MarkerArray()
        self.moving_average = [3] * size_of_moving_avg
        self.set_marker_array(5, 'camera_color_optical_frame', 'package://get_obj_dist/human_model.STL')

        # Initializing ROS Topics
        self.dist_pub = rospy.Publisher('/obj_to_dist/human_distance', Image, queue_size=10)
        self.render_pub = rospy.Publisher('/obj_to_dist/show_people_marker_array', MarkerArray, queue_size=10)

        self.bbx_sub = message_filters.Subscriber('/human_detected_image/bounding_box', box_list)
        self.human_image_sub = message_filters.Subscriber('/human_detected_image/image', Image)
        self.depth_image_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)

        ts = message_filters.ApproximateTimeSynchronizer([self.bbx_sub, self.human_image_sub,
                                                          self.depth_image_sub],
                                                         queue_size=10,
                                                         slop=1)
        ts.registerCallback(self.callback)

    def callback(self, bbx, image, depth):
        if bbx.length:
            cv_depth = self.bridge.imgmsg_to_cv2(depth, 'passthrough')
            cv_image = self.bridge.imgmsg_to_cv2(image, 'bgr8')

            for i in range(0, 5):
                if i <= bbx.length - 1:
                    cood = self.get_human_distance(cv_depth, cv_image, bbx.people_list[i], i)
                    self.set_model_coordinates(cood, i)
                else:
                    self.set_model_coordinates((-1, -1, -1), i)
        self.render_pub.publish(self.marker_array)

    def get_human_distance(self, cv_depth, cv_image, box, person_id):
        roi_depth = cv_depth[box.ymin:box.ymax, box.xmin:box.xmax]
        filtered_depth, _size = filter_background(roi_depth)
        if _size:
            self.moving_average.pop()
            self.moving_average.insert(0, filtered_depth.sum() / _size / 1000.0)

            current_avg = sum(self.moving_average) / size_of_moving_avg

            x_meter = get_x_in_meters(box.xmin, box.xmax, current_avg)
            cv2.putText(cv_image, '{} meters / Person {}'.format(round(current_avg, 2), person_id),
                        (box.xmin, box.ymax - 100),
                        font,
                        font_scale,
                        font_color,
                        line_type)
            self.dist_pub.publish(self.bridge.cv2_to_imgmsg(cv_image))

            # TODO: Add dynamic find floor
            return x_meter, 0.3, current_avg
        else:
            return -1, -1, -1

    def set_marker_array(self, size, frame_id, package):
        # self.marker_array.markers = [Marker()] * size
        for i in range(0, size):
            self.marker_array.markers.append(Marker())
            self.marker_array.markers[i].header.frame_id = frame_id
            self.marker_array.markers[i].lifetime = rospy.Duration()
            self.marker_array.markers[i].id = i
            self.marker_array.markers[i].type = Marker.MESH_RESOURCE
            self.marker_array.markers[i].mesh_resource = package

    def set_model_coordinates(self, cood, person_id):
        self.marker_array.markers[person_id].header.stamp = rospy.Time.now()
        if cood == (-1, -1, -1):
            self.marker_array.markers[person_id].action = Marker.DELETE
        else:
            self.marker_array.markers[person_id].header.stamp = rospy.Time.now()
            self.marker_array.markers[person_id].action = Marker.ADD

            self.marker_array.markers[person_id].color.r = 1.0
            self.marker_array.markers[person_id].color.g = 1.0
            self.marker_array.markers[person_id].color.b = 1.0
            self.marker_array.markers[person_id].color.a = 1.0

            # TODO: Change the scale of the DAE model so these numbers make more sense
            self.marker_array.markers[person_id].scale.x = 0.0005
            self.marker_array.markers[person_id].scale.y = 0.0009
            self.marker_array.markers[person_id].scale.z = 0.0008

            self.marker_array.markers[person_id].pose.orientation.x = 1.0
            self.marker_array.markers[person_id].pose.orientation.y = 0.0
            self.marker_array.markers[person_id].pose.orientation.z = 0.0

            self.marker_array.markers[person_id].pose.position.x = cood[0] - 0.4
            self.marker_array.markers[person_id].pose.position.y = cood[1]
            self.marker_array.markers[person_id].pose.position.z = cood[2]


def main(args):
    rospy.init_node('obj_dist', anonymous=True)
    print("object_to_distance.py is running")
    _ = obj_dist()
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    # rospy.set_param("use_sim_time", 'true')
    main(sys.argv)
