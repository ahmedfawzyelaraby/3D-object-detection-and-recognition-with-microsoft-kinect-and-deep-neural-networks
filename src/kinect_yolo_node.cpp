#include <ros/ros.h>
#include "kinect_yolo/KinectToCVMat.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinect_yolo");
  ros::NodeHandle nodeHandle;

  kinect_yolo::KinectToCVMat Kinect_To_CV_Mat(nodeHandle);

  ros::spin();
  return 0;
}
