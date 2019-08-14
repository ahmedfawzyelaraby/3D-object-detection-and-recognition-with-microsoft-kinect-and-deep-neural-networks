# 3D Object Detection and Recognition Using Microsoft Kinect and Deep Neural Networks
This is a ROS based c++ code, whose role is to read the RGB and Depth images from the kinect, pass the RGB images to YOLO wrapper and then take the objects output from YOLO, maps them to the Depth images, and gets the depth of each object. This code was part of the work submited by [me](https://www.linkedin.com/in/ahmedfawzyelaraby/) in my [master's degree](https://www.researchgate.net/publication/335127899_3D_Object_Detection_and_Classification_Using_Microsoft_Kinect_and_Deep_Neural_Networks).
## Getting Started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. **Please be noted that this code is tested on Ubuntu OS only**.
### System Overview
### Prerequisites
Things you need on your local machine, in order to be able to compile and use this ROS node.

- Nvidia GPU with minimum **2 GB** GPU RAM.
- Compatible Nvidia GPU Driver
```
$ sudo apt-get update
$ sudo apt-get upgrade
$ sudo add-apt-repository ppa:graphics-drivers
$ sudo apt-get update
$ sudo apt-get install nvidia-370
```
- Cuda 8.0 Library
```
$ cd ~/Downloads
$ wget https://developer.download.nvidia.com/compute/cuda/8.0/secure/Prod2/local_installers/cuda_8.0.61_375.26_linux.run
$ sudo bash cuda_8.0.61_375.26_linux-run --silent --toolkit
$ echo "export PATH=/usr/local/cuda-8.0/bin${PATH:+:${PATH}}" >> ~/.bashrc
$ echo "export LD_LIBRARY_PATH=/usr/local/cuda-8.0/lib64 ${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" >> ~/.bashrc
$ source ~/.bashrc
```
- OpenCV 2.4 Library
```
$ sudo apt-get install build-essential
$ sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
$ sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
$ cd ~/Downloads
$ wget https://github.com/opencv/opencv/archive/2.4.13.5.zip
$ unzip -u 2.4.13.5.zip
$ cd opencv-2.4.13.5
$ mkdir build
$ cd build
$ cmake -DWITH_OPENCL=OFF -DBUILD_EXAMPLES=OFF -DWITH_CUDA=OFF ..
$ make -j
$ sudo make install
```
- ROS Indigo
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt-get update
$ sudo apt-get install ros-indigo-desktop-full
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt-get install python-rosinstall
```
- ROS Catkin Workspace
```
$ cd [where-you-want-to-put-your-code-in]
$ mkdir -p ros-workspace/src
$ cd ros-workspace
$ catkin_make
$ source devel/setup.bash
```
- [FreeNect 2 Library](https://github.com/OpenKinect/libfreenect2)
```
$ cd ~/Downloads
$ git clone https://github.com/OpenKinect/libfreenect2.git
$ cd libfreenect2
$ git checkout v0.2.0
$ cd depends
$ ./download_debs_trusty.sh
$ sudo apt-get install build-essential cmake pkg-config
$ sudo dpkg -i debs/libusb*deb
$ sudo apt-get install libturbojpeg libjpeg-turbo8-dev
$ sudo dpkg -i debs/libglfw3*deb; sudo apt-get install -f
$ cd ..
$ mkdir build && cd build
$ cmake  -DCMAKE_INSTALL_PREFIX=[where-you-want-to-put-your-code-in]/freenect2 -DENABLE_CXX11=ON
$ make -j
$ sudo make install
```
- [Kinect V2 ROS Driver](https://github.com/code-iai/iai_kinect2)
```
$ cd [where-you-want-to-put-your-code-in]/ros-workspace/src
$ sudo apt-get install mesa-utils
$ git clone https://github.com/code-iai/iai_kinect2.git
$ cd iai_kinect2
$ rosdep install -r --from-paths .
$ cd [where-you-want-to-put-your-code-in]/ros-workspace
$ catkin_make -DCMAKE_BUILD_TYPE="Release" -DENABLE_OPENCL=OFF -Dfreenect2_DIR=[where-you-want-to-put-your-code-in]/freenect2/lib/cmake/freenect2
```
- [YOLO V2 GPU Wrapper](https://github.com/ahmedfawzyelaraby/yolo-v2-gpu-wrapper.git)
```
$ cd [where-you-want-to-put-your-code-in]
$ git clone https://github.com/ahmedfawzyelaraby/yolo-v2-gpu-wrapper.git
$ cd yolo-v2-gpu-wrapper
$ $ mkdir build
$ cd build
$ cmake ..
$ make -j
$ make -j
$ sudo make install
```
- [3D Object Detection and Recognition to RGB Image Viewer](https://github.com/ahmedfawzyelaraby/3D-object-detection-and-recognition-to-rgb-image-viewer.git)
```
$ cd [where-you-want-to-put-your-code-in]/ros-workspace/src
$ git clone https://github.com/ahmedfawzyelaraby/3D-object-detection-and-recognition-to-rgb-image-viewer.git
```
- YOLO Weights and Configuration Files

This system is tested with specific weights, configuration and label files of YOLO, which you can find as compressed folder [here](https://drive.google.com/file/d/1WvBKo099NwbUBXkKDYVg7UFeungU_ldv/view?usp=sharing). Download this compressed folder to [where-you-want-to-put-your-code-in].
```
$ cd [where-you-want-to-put-your-code-in]
$ tar -zxvf yolo-files.tar.gz
```
### Installation
```
$ cd [where-you-want-to-put-your-code-in]/ros-workspace/src
$ git clone https://github.com/ahmedfawzyelaraby/3D-object-detection-and-recognition-with-microsoft-kinect-and-deep-neural-networks.git
$ cd ../
$ source ./depl/setup.bash
$ catkin_make -only-pkg-with-deps kinect_yolo
```
### Deployment
All you have to do is to launch the ROS launch file attached with the node and it will launch roscore, launch the kinect driver's node, and launch yolo node:
```
$ cd [where-you-want-to-put-your-code-in]/ros-workspace/src
$ source ../depl/setup.bash
$ roslaunch kinect_yolo/launch/kinect.launch cfg_file_path:=[where-you-want-to-put-your-code-in]/yolo-files/yolo.cfg data_file_path:=[where-you-want-to-put-your-code-in]/yolo-files/coco.data weights_file_path:=[where-you-want-to-put-your-code-in]/yolo-files/yolo.weights labels_path:=[where-you-want-to-put-your-code-in]/yolo-files/labels/ names_file_path:=[where-you-want-to-put-your-code-in]/yolo-files/coco.names
```
### Sample Result
