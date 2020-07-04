import numpy as np
import matplotlib.pyplot as plt
import math

from typing import List, Tuple

_pixMax = 600
_pixMin = 0
_pixMid = (_pixMax + _pixMin) / 2


class OccMap:
    def __init__(self, row=100, col=100, size_x=20, size_y=20):
        """

        :param row:
        :param col:
        """

        # Note the 0, 0 of the map is the top left corner of the map, where [0][0] of the matrix would be
        self.map = np.ones((row, col)) * 0.50
        self.row_size = row
        self.col_size = col

        # Physical size of the field is assume to be 20 meters
        self.row_dist = size_y / row
        self.col_dist = size_x / col

    def get_map(self):
        return self.map

    def set_percentage(self, xmin: int, xmax: int, distances: List, state: Tuple[float, float, float]) -> bool:
        """
        update the probability of the grid map

        :param xmin: location of left most pixel
        :param xmax: location of right most pixel
        :param distances: list of distances between xmin and xmax
        :param state: the position and orientation of the tractor in meters and radians
        :return:
         bool: The return value. True for success. False for otherwise
        """
        distance_to_image_plane = 150

        col_tractor, row_tractor = self.coordinates_to_index(state[0], state[1])
        theta_tractor = state[2]

        theta_1 = math.atan((xmin - _pixMid) / distance_to_image_plane)
        theta_2 = math.atan((xmax - _pixMid) / distance_to_image_plane)
        angles = np.linspace(theta_1, theta_2, len(distances)) + theta_tractor

        for dist, angle in zip(distances, angles):
            x_dist = dist * math.cos(angle)
            y_dist = -dist * math.sin(angle)
            col_index, row_index = self.coordinates_to_index(state[0] + x_dist, state[1] + y_dist)
            for square in _bresenham(row_tractor, col_tractor, row_index, col_index):
                row = square[0]
                col = square[1]
                if row >= self.row_size or col >= self.col_size:
                    break
                self.map[row][col] *= 0.2
            if row_index >= self.row_size or col_index >= self.col_size:
                break
            self.map[row_index][col_index] = 1


        return True

    def coordinates_to_index(self, x: float, y: float) -> Tuple[int, int]:
        x_max = self.col_dist * self.col_size
        y_max = self.row_dist * self.row_size

        if x > x_max or y > y_max:
            raise ValueError
        col_index = int(x / self.col_dist)
        row_index = int((y_max - y) / self.row_dist)
        return col_index, row_index


def _bresenham(x0, y0, x1, y1):
    # I got this from the Bresenham module, which you can install through pip -install bresenham
    # I only copied this because my IDE was doing some weird stuff and not recognizing the package
    # All credits go Petr Viktorin (Author, I believe)
    """Yield integer coordinates on the line from (x0, y0) to (x1, y1).

    Input coordinates should be integers.

    The result will contain both the start and the end point.
    """
    dx = x1 - x0
    dy = y1 - y0

    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1

    dx = abs(dx)
    dy = abs(dy)

    if dx > dy:
        xx, xy, yx, yy = xsign, 0, 0, ysign
    else:
        dx, dy = dy, dx
        xx, xy, yx, yy = 0, ysign, xsign, 0

    D = 2*dy - dx
    y = 0

    for x in range(dx + 1):
        yield x0 + x*xx + y*yx, y0 + x*xy + y*yy
        if D >= 0:
            y += 1
            D -= 2*dx
        D += 2*dy


if __name__ == '__main__':
    _map = OccMap(50, 50)

    _map.set_percentage(150, 450, [13, 13, 13, 13, 13, 13], (6, 6, 0))
    print(_map.get_map())
    plt.imshow(_map.get_map())
    plt.colorbar()
    plt.show()

