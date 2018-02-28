#pragma once

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "yolo.h"

#include <string>
#include <cmath>

namespace kinect_yolo {

class KinectToCVMat {
public:
	KinectToCVMat(ros::NodeHandle& nodeHandle);
	virtual ~KinectToCVMat();
	void RGBImageCallback(const sensor_msgs::Image&);
	void DepthImageCallback(const sensor_msgs::Image&);
	void IRImageCallback(const sensor_msgs::Image&);
	double DepthImageCalibrated(cv::Mat, cv::Rect);

private:
	ros::NodeHandle nodeHandle_;
	ros::Subscriber RGBImageSubscriber;
	ros::Subscriber DepthImageSubscriber;
	ros::Subscriber IRImageSubscriber;
	Yolo YoloDNN;
	std::vector<DetectedObject> VectorOfDetections;
	std::vector<double> VectorOfDepthOfDetections;
	cv::Size DetectionImageSize;
	cv::Size DepthImageSize;
	const std::string NodeName = "kinect_yolo";
	const std::string ParamNameSeparator = "/";
	std::string CFGFilePathParamName = "node_cfg_file_path";
	std::string DataFilePathParamName = "node_data_file_path";
	std::string WeightsFilePathParamName = "node_weights_file_path";
	std::string LabelsFilePathParamName = "node_labels_path";
	std::string NamesFilePathParamName = "node_names_file_path";
	std::string RGBImageTopicNameParamName = "node_rgb_image_topic";
	std::string DepthImageTopicNameParamName = "node_depth_image_topic";
	std::string IRImageTopicNameParamName = "node_ir_image_topic";
	std::string CFGFilePath;
	std::string DataFilePath;
	std::string WeightsFilePath;
	std::string LabelsFilePath;
	std::string NamesFilePath;
	std::string RGBImageTopicName;
	std::string DepthImageTopicName;
	std::string IRImageTopicName;
};

}
