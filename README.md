# Kinect 3D Object Detection and Recognition ROS Node
This is a c++ ROS node code, whose role is to read the RGB and Depth images from the kinect. It passes the RGB images to YOLO wrapper. Then, it takes the objects output from YOLO, maps them to the Depth images, and gets the depth of each object.
##Getting Started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. **Please be noted that this code is tested on Ubuntu OS only**.
### Prerequisites
Things you need on your local machine, in order to be able to compile and use this ROS node.

- Nvidia GPU

- [Compatible Nvidia Driver](http://www.nvidia.com/Download/index.aspx)

- [Cuda Library](https://developer.nvidia.com/cuda-downloads)

- [OpenCV Library 2.4](https://opencv.org/releases.html)

- [Full version of ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)

- [Kinect driver ROS node](https://github.com/code-iai/iai_kinect2)

- [YOLO Standalone Wrapper](https://AhmedFawzyElaraby@bitbucket.org/AhmedFawzyElaraby/yolo_standalone_wrapper.git)
### Installation
```
$ cd [ROS_WorkSpace]/src
$ git clone https://AhmedFawzyElaraby@bitbucket.org/AhmedFawzyElaraby/ros_kinect_to_yolo_node.git
$ source ../depl/setup.bash
$ catkin build kinect_yolo
```
### Deployment
All you have to do is to launch the ROS launch file attached with the node and it will launch roscore, launch the kinect driver's node, and launch yolo node:
```
$ cd [ROS_WorkSpace]/src
$ source ../depl/setup.bash
$ roslaunch kinect_yolo/launch/kinect.launch
```
