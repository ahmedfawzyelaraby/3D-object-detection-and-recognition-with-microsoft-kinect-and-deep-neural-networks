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

	nodeHandle_.getParam(CFGFilePathParamName, CFGFilePath);
	nodeHandle_.getParam(DataFilePathParamName, DataFilePath);
	nodeHandle_.getParam(WeightsFilePathParamName, WeightsFilePath);
	nodeHandle_.getParam(LabelsFilePathParamName, LabelsFilePath);
	nodeHandle_.getParam(NamesFilePathParamName, NamesFilePath);
	nodeHandle_.getParam(RGBImageTopicNameParamName, RGBImageTopicName);
	nodeHandle_.getParam(DepthImageTopicNameParamName, DepthImageTopicName);

	RGBImageSubscriber = nodeHandle_.subscribe(RGBImageTopicName,1000, &KinectToCVMat::RGBImageCallback, this);
	DepthImageSubscriber = nodeHandle_.subscribe(DepthImageTopicName,1000, &KinectToCVMat::DepthImageCallback, this);

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
	RGBCVImage = cv_bridge::toCvCopy(RGBImage, sensor_msgs::image_encodings::BGR8);

	std::vector<DetectedObject> VectorOfDetections;
	YoloDNN.detect(RGBCVImage->image, VectorOfDetections);
}

void KinectToCVMat::DepthImageCallback(const sensor_msgs::Image& DepthImage)
{
	ROS_INFO_STREAM("Depth Encoding: " << DepthImage.encoding);
}

}

