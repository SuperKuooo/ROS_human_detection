#!/usr/bin/env python

import numpy as np
import open3d as o3d
import os
import time

abs_path = '/home/jerry/Documents/Research/Data/plc/file1'


def main():
    vis = o3d.visualization.Visualizer()
    vis.create_window("3D Scene")
    # o3d.visualization.draw_geometries([pcd])
    try:
        while True:
            for filename in os.listdir(abs_path):
                print(filename)
                pcd = o3d.io.read_point_cloud(abs_path+'/'+filename)
                time.sleep(0.1)
                vis.add_geometry(pcd)
                vis.update_geometry()
                vis.poll_events()
                vis.update_renderer()
                # o3d.visualization.Visualizer.draw_geometrie(pcd)
    except KeyboardInterrupt:
        print("Terminating")


if __name__ == "__main__":
    main()
