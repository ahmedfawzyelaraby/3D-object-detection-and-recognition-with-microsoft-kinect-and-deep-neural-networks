#include "kinect_yolo/KinectToCVMat.hpp"

namespace kinect_yolo {

KinectToCVMat::KinectToCVMat(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle)
{
	RGBImageSubscriber = nodeHandle_.subscribe("/camera/rgb/image_color",10, &KinectToCVMat::RGBImageCallback, this);
	DepthImageSubscriber = nodeHandle_.subscribe("/camera/depth/image",10, &KinectToCVMat::DepthImageCallback, this);
}

KinectToCVMat::~KinectToCVMat()
{
}

void KinectToCVMat::RGBImageCallback(const sensor_msgs::Image& RGBImage)
{
	ROS_INFO_STREAM("RGB Encoding: " << RGBImage.encoding);
}

void KinectToCVMat::DepthImageCallback(const sensor_msgs::Image& DepthImage)
{
	ROS_INFO_STREAM("Depth Encoding: " << DepthImage.encoding);
}

}

