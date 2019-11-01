import numpy.ma as ma
import cv2


def filter_background(roi):
    ret_val = ma.masked_less(roi, 2900)
    # roi[roi <= 2900]
    return ret_val


def mask_the_thing(image, mask):
    cv2.bitwise_and(image, mask)

