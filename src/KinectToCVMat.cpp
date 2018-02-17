#include "kinect_yolo/KinectToCVMat.hpp"

namespace kinect_yolo {

KinectToCVMat::KinectToCVMat(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle)
{
	CFGFilePathParamName = ParamNameSeparator + NodeName + ParamNameSeparator + CFGFilePathParamName;
	DataFilePathParamName = ParamNameSeparator + NodeName + ParamNameSeparator + DataFilePathParamName;
	WeightsFilePathParamName = ParamNameSeparator + NodeName + ParamNameSeparator + WeightsFilePathParamName;
	LabelsFilePathParamName = ParamNameSeparator + NodeName + ParamNameSeparator + LabelsFilePathParamName;
	NamesFilePathParamName = ParamNameSeparator + NodeName + ParamNameSeparator + NamesFilePathParamName;
	RGBImageTopicNameParamName = ParamNameSeparator + NodeName + ParamNameSeparator + RGBImageTopicNameParamName;
	DepthImageTopicNameParamName = ParamNameSeparator + NodeName + ParamNameSeparator + DepthImageTopicNameParamName;
	IRImageTopicNameParamName = ParamNameSeparator + NodeName + ParamNameSeparator + IRImageTopicNameParamName;

	nodeHandle_.getParam(CFGFilePathParamName, CFGFilePath);
	nodeHandle_.getParam(DataFilePathParamName, DataFilePath);
	nodeHandle_.getParam(WeightsFilePathParamName, WeightsFilePath);
	nodeHandle_.getParam(LabelsFilePathParamName, LabelsFilePath);
	nodeHandle_.getParam(NamesFilePathParamName, NamesFilePath);
	nodeHandle_.getParam(RGBImageTopicNameParamName, RGBImageTopicName);
	nodeHandle_.getParam(DepthImageTopicNameParamName, DepthImageTopicName);
	nodeHandle_.getParam(IRImageTopicNameParamName, IRImageTopicName);

	RGBImageSubscriber = nodeHandle_.subscribe(RGBImageTopicName,1, &KinectToCVMat::RGBImageCallback, this);
	DepthImageSubscriber = nodeHandle_.subscribe(DepthImageTopicName,1, &KinectToCVMat::DepthImageCallback, this);
	IRImageSubscriber = nodeHandle_.subscribe(IRImageTopicName, 1, &KinectToCVMat::IRImageCallback, this);

	YoloDNN.setConfigFilePath(CFGFilePath.c_str());
	YoloDNN.setDataFilePath(DataFilePath.c_str());
	YoloDNN.setWeightFilePath(WeightsFilePath.c_str());
	YoloDNN.setAlphabetPath(LabelsFilePath.c_str());
	YoloDNN.setNameListFile(NamesFilePath.c_str());
}

KinectToCVMat::~KinectToCVMat()
{
}

void KinectToCVMat::RGBImageCallback(const sensor_msgs::Image& RGBImage)
{
	ROS_INFO_STREAM("RGB Encoding: " << RGBImage.encoding);
	
	cv_bridge::CvImagePtr RGBCVImage;
	RGBCVImage = cv_bridge::toCvCopy(RGBImage, RGBImage.encoding);

	std::vector<DetectedObject> VectorOfDetections;
	YoloDNN.detect(RGBCVImage->image, VectorOfDetections);
}

void KinectToCVMat::DepthImageCallback(const sensor_msgs::Image& DepthImage)
{
	ROS_INFO_STREAM("Depth Encoding: " << DepthImage.encoding);

	cv_bridge::CvImagePtr DepthCVImage;
	DepthCVImage = cv_bridge::toCvCopy(DepthImage, DepthImage.encoding);

}

void KinectToCVMat::IRImageCallback(const sensor_msgs::Image& IRImage)
{
	ROS_INFO_STREAM("IR Encoding: " << IRImage.encoding);

	cv_bridge::CvImagePtr IRCVImage;
	IRCVImage = cv_bridge::toCvCopy(IRImage, IRImage.encoding);
}

}

