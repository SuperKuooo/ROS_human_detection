
import cv2

# Number of classes the object detector can identify
NUM_CLASSES = 1

# Initializing utility variable
person_id = 1
text_position = (10, 450)
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 0.75
font_color = (255,255,255)
line_type = 2
run_time_list = []
prediction_level_list = []