#!/usr/bin/env python
import time
import sys
import os
import rospy
import numpy as np
import tensorflow as tf

sys.path.append('/home/jerry/Documents/workspaces/human_detection/src/ROS_human_detection/human_detection/src')

# For performance analysis timing, import time.
# from analysis_tools.data_grapher import *
from analysis_tools.define import *
from human_detection.msg import bounding_box, box_list
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Import utilites
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

# Defining Paths
CWD_PATH = '/home/jerry/Documents/workspaces/human_detection/src/ROS_human_detection/human_detection/src'
LABEL_MAPS = ['human_label_map.pbtxt', 'mscoco_label_map.pbtxt']
MODEL_NAME = 'ssd_mobilenet_v1_coco_2017_11_17'
PATH_TO_CKPT = os.path.join(CWD_PATH, MODEL_NAME, 'frozen_inference_graph.pb')
PATH_TO_LABELS = os.path.join(CWD_PATH + '/data', LABEL_MAPS[0])

# Load the label map.
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(
    label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Load the Tensorflow model into memory.
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.compat.v1.GraphDef()
    with tf.io.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    sess = tf.compat.v1.Session(graph=detection_graph)

# Define input and output tensors (i.e. data) for the object detection classifier

# Input tensor is the image
image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

# Output tensors are the detection boxes, scores, and classes
# Each box represents a part of the image where a particular object was detected
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

# Each score represents level of confidence for each of the objects.
# The score is shown on the result image, together with the class label.
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

# Number of objects detected
num_detections = detection_graph.get_tensor_by_name('num_detections:0')


# time_origin = time.time()
class human_detector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

        self.image_pub = rospy.Publisher("/human_detected_image/image", Image, queue_size=10)
        self.bbx_pub = rospy.Publisher("/human_detected_image/bounding_box", box_list, queue_size=10)

    def callback(self, data):
        t0 = time.time()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        frame_expanded = np.expand_dims(cv_image, axis=0)

        # Perform the actual detection by running the model with the image as input
        (boxes, scores, classes, num) = sess.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: frame_expanded})

        # Reducing the elements
        classes = classes[:, 0:5]
        scores = scores[:, 0:5]
        boxes = boxes[:, 0:5]

        # Delete anything that is not a person
        for i in range(0, 5):
            if classes[0][i] != person_id:
                scores[0][i] = 0

        # Draw the results of the detection (aka 'visulaize the results')
        vis_util.visualize_boxes_and_labels_on_image_array(
            cv_image,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            use_normalized_coordinates=True,
            line_thickness=8,
            min_score_thresh=0.60)

        # Calculate frame time
        t1 = time.time()
        run_time = t1 - t0
        if run_time < 1:
            run_time_list.append(run_time)
            if len(run_time_list) > 10:
                del run_time_list[0]

        if len(run_time_list) > 0:
            avg_run_time = round(sum(run_time_list) / len(run_time_list) * 1000, 1)
            cv2.putText(cv_image, 'Run Time: {}ms'.format(avg_run_time),
                        text_position,
                        font,
                        font_scale,
                        font_color,
                        line_type)

        counter = 0
        list_length = 0
        bbx_list = box_list()
        bbx_list.header.stamp = data.header.stamp

        for score in scores[0]:
            if score > 0.6:
                list_length += 1
                coordinates = bounding_box()
                coordinates.ymin = int(boxes[0, counter, 0] * screen_height)
                coordinates.xmin = int(boxes[0, counter, 1] * screen_width)
                coordinates.ymax = int(boxes[0, counter, 2] * screen_height)
                coordinates.xmax = int(boxes[0, counter, 3] * screen_width)
                bbx_list.people_list.append(coordinates)
            counter += 1

        # print(list_length)
        bbx_list.length = list_length
        self.bbx_pub.publish(bbx_list)

        try:
            cvt_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            cvt_msg.header.stamp = data.header.stamp
            self.image_pub.publish(cvt_msg)
        except CvBridgeError as e:
            print(e)

        # Append Accuracy
        # if scores[0][0] > 0.01:
        #     prediction_level_list.append(scores[0][0])


def main():
    _ = human_detector()
    rospy.init_node('human_detector', anonymous=True)
    rospy.spin()
    # graph_average_percentage(prediction_level_list)


if __name__ == '__main__':
    print('Current Tensorflow Version: ' + str(tf.__version__))
    rospy.set_param('use_sim_time', 'True')
    main()




























