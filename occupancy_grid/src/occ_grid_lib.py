import numpy as np


class OccMap:
    def __init__(self, row=100, col=100):
        self.map = np.ones((row, col))

    def get_map(self):
        return self.map

