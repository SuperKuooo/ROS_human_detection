# ROS_human_detection
Human detection model that runs on ROS

This is a human detection mode that uses Intel RealSense D435 Camera and ROS Kinetic. Visit the GitHub page of
Intel RealSense-ROS for more camera installation.

## Installing The Workspace
Run the following commands on terminal to get started.

First we start a workspace
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

*Something like that. I will fix it later.*

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
- *Will update as I go*

2. Outdoor settings
- *Will update as I go*