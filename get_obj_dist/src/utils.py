import numpy as np
import matplotlib.pyplot as plt

_noise_limit = 100
_diff = 1000
_bin = [i * _diff for i in range(0, 11)]


# Really basic way of separating foreground and background.
def filter_background(roi, max_depth=8000):
    # Anything further than 8000mm, we consider it as background
    # Anything less than 100mm is consider noise
    ret_val = np.ma.masked_greater(roi, max_depth)
    ret_val = np.ma.masked_less(ret_val, _noise_limit)
    unique, counts = np.unique(ret_val.mask, return_counts=True)
    _dict = dict(zip(unique, counts))
    if False in _dict:
        return ret_val, _dict[False]
    else:
        return ret_val, 0


def dynamic_background(roi):
    # Anything less than 100mm is sure noise
    # roi = np.ma.masked_less(roi, 100)
    roi_1d = roi.flatten()
    hist, bins = np.histogram(roi, bins=_bin, density=True)
    max_bin = hist.argmax() + 1

    # plt.hist(roi_1d, bins=_bin, density=True)
    # plt.title('Hello')
    # plt.show()
    return filter_background(roi, max_bin * _diff)


def get_x_in_meters(xmin, xmax, z_i):
    # Tune z_c to get better value lol.
    # 500 is literally randomly chosen lol
    z_c = 500
    ret_val = (z_i * (xmax + xmin - 600.0)) / (2 * z_c)
    return ret_val
