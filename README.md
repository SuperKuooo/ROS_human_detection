# ROS_human_detection
Human detection model that runs on ROS

This is a human detection mode that uses Intel RealSense D435 Camera and ROS Kinetic/Melodic.
This is still a work in progress and I am continuously updating.
If you have any questions or suggestions, please do not hesistate to contact me.
I will answer your questions to the best of my ability :D

Jerry Kuo: jerrykuo820@gmail.com

## How This Model Works
The basic pipeline looks like this.

RGB Image -> Object Detection -> 2D Bounding Box -> Use ROI to Parse Depth Image ->\
Filter Depth Info -> Get 3D Coordinates -> Show the Human Model on PointCloud

I am using <a href="https://github.com/tensorflow/models/tree/master/research/object_detection" target="_blank">
tensorflow's object detection module </a>for getting the 2D bounding box. I might consider to switch over to YOLO, PyTorch.
 We will see lol.

Aside from that, everything else is hard-coded. What each function does should be pretty self-explanatory.

## Installing The Workspace
If you are using/ plan to use the D435 RGBD Camera, 
visit the GitHub page of Intel RealSense-ROS for full RealSense Camera installation on ROS.
Also keep in mind that the model runs on **Python 2**. Because ros kinetic was designed more for Python 2.
Transferring over to Python 3 is definitely on my list of todo, but just not my priority. 

Ok. Now we can start installing the workspace.

First start by initializing a workspace
```shell
mkdir -p ~/catkin_workspace/src
cd catkin_workspace/src
```

Pull the repository and catkin_make
```shell
git clone https://github.com/SuperKuooo/ROS_human_detection.git
cd ..
catkin_make
source ./devel/setup.bash
```

Then you are good to go! Open three terminal and run the three following commands.
```shell
roslaunch realsense2_camera rs_rgbd.laucnh
rosrun human_detection webcam.py
rosrun get_obj_dist object_to_distance.py
```

## Sample Data Files
Below are some links to example files. These bag files are too big for GitHub so I put them on Google Drive.
Download them as you wish. Here are the types and the message topics.
I recorded them with Intel RealSense D435 Camera at 60 fps (I think).

```
types:       sensor_msgs/Image       [060021388200f6f0f447d0fcd9c64743]
             sensor_msgs/PointCloud2 [1158d486dd51d683ce2f1be655c3c181]
topics:      /camera/color/image_raw           534 msgs    : sensor_msgs/Image      
             /camera/depth/image_rect_raw      534 msgs    : sensor_msgs/Image      
             /camera/depth_registered/points   525 msgs    : sensor_msgs/PointCloud2
```

1. Indoor settings
- *Will update as I go. File is so big lol*

2. Outdoor settings
- *Same as above*