#include <ros/ros.h>
#include "kinect_yolo/KinectToCVMat.hpp"

// #include "opencv2/opencv.hpp"
// #include <stdlib.h>
// #include <stdio.h>
// #include <iostream>
// #include <sstream>
// #include <vector>
// #include <ctime>
// #include "yolo.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinect_yolo");
  ros::NodeHandle nodeHandle;

  kinect_yolo::KinectToCVMat Kinect_To_CV_Mat(nodeHandle);

  ros::spin();
  return 0;
}
