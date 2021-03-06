#pragma once

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"
#include "kinect_yolo/DepthAndDetections.h"

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "yolo.h"

#include <string>
#include <cmath>
#include <vector>
#include <fstream>
#include <ctime>
#include <cstdlib>
#include <cmath>

namespace kinect_yolo {

class KinectToCVMat {
public:
	KinectToCVMat(ros::NodeHandle& nodeHandle);
	virtual ~KinectToCVMat();
	void RGBImageCallback(const sensor_msgs::Image&);
	void DepthImageCallback(const sensor_msgs::Image&);
	// void IRImageCallback(const sensor_msgs::Image&);
	double CalibratedDepthValue(cv::Mat, cv::Rect);
	std::vector<std::string> GetClassesNames(std::string);
	void InitiateClassesColors(int);
	double depthCalculationAlgorithm(cv::Mat);

private:
	ros::NodeHandle nodeHandle_;
	ros::Subscriber RGBImageSubscriber;
	ros::Subscriber DepthImageSubscriber;
	ros::Subscriber IRImageSubscriber;
	ros::Publisher DetectionsAndDepthsPublisher;
	Yolo YoloDNN;
	std::vector<DetectedObject> VectorOfDetections;
	std::vector<double> VectorOfDepthOfDetections;
	std::vector<std::string> VectorOfClassesNames;
	std::vector<int> VectorOfClassesRColor;
	std::vector<int> VectorOfClassesGColor;
	std::vector<int> VectorOfClassesBColor;
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
	std::string DetectionsTopicName = "detections_and_depth";
	double ConversionFromMiliMeterToMeter = 1000.0;
	int minimumClippingValueMM = 0;
	int maximumClippingValueMM = 8000;
	int depthAlgorithmNumberOfGridCells = 9;
};

}
