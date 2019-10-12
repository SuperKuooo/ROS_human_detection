from datetime import datetime
import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import time


# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

# Start streaming
profile = pipeline.start(config)


# Streaming loop
try:
    vis = o3d.visualization.Visualizer()
    vis.create_window("Tests")
    pcd = o3d.geometry.PointCloud()
    while True:
        dt0 = datetime.now()
        vis.add_geometry(pcd)
        pcd.clear()
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if not color or not depth:
            continue
        pc = rs.pointcloud()
        pc.map_to(color)
        points = pc.calculate(depth)
        vtx = np.asanyarray(points.get_vertices())
        print(vtx)
        print(o3d.utility.Vector3dVector(vtx))
        # pcd.points = o3d.utility.Vector3dVector(vtx)
        vis.update_geometry()
        vis.poll_events()
        vis.update_renderer()
        process_time = datetime.now() - dt0
        print("FPS = {0}".format(1/process_time.total_seconds()))
        time.sleep(1)
except KeyboardInterrupt:
    print('Exiting')

finally:
    pipeline.stop()