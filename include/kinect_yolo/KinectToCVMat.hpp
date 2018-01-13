#pragma once

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <string>
#include <cmath>

namespace kinect_yolo {

class KinectToCVMat {
public:
	KinectToCVMat(ros::NodeHandle& nodeHandle);
	virtual ~KinectToCVMat();
	void RGBImageCallback(const sensor_msgs::Image&);
	void DepthImageCallback(const sensor_msgs::Image&);

private:
	ros::NodeHandle nodeHandle_;
	ros::Subscriber RGBImageSubscriber;
	ros::Subscriber DepthImageSubscriber;
};

}
