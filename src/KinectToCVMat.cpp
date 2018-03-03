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

	VectorOfDetections.clear();
	VectorOfDepthOfDetections.clear();
}

KinectToCVMat::~KinectToCVMat()
{
}

void KinectToCVMat::RGBImageCallback(const sensor_msgs::Image& RGBImage)
{
	ROS_INFO_STREAM("RGB Encoding: " << RGBImage.encoding);
	
	cv_bridge::CvImagePtr RGBCVImage;
	RGBCVImage = cv_bridge::toCvCopy(RGBImage, RGBImage.encoding);

	DetectionImageSize = RGBCVImage->image.size();

	// std::vector<DetectedObject> LocalVectorOfDetections;
	// YoloDNN.detect(RGBCVImage->image, LocalVectorOfDetections);

	// VectorOfDetections = LocalVectorOfDetections;

	// ROS_INFO_STREAM("RGB Vector Size: " << VectorOfDetections.size());
}

void KinectToCVMat::DepthImageCallback(const sensor_msgs::Image& DepthImage)
{
	ROS_INFO_STREAM("Depth Encoding: " << DepthImage.encoding);

	cv_bridge::CvImagePtr DepthCVImage;
	DepthCVImage = cv_bridge::toCvCopy(DepthImage, DepthImage.encoding);

	DepthImageSize = DepthCVImage->image.size();

	// ROS_INFO_STREAM("Depth Vector Size: " << VectorOfDetections.size());
	// ROS_INFO_STREAM("Depth Vector Content: " << VectorOfDetections.empty());

	// if (!(VectorOfDetections.empty()))
	// {
	// 	for (int i = 0; i < VectorOfDetections.size(); i++)
	// 	{
	// 		ROS_INFO_STREAM("Depth Pixel Value: " << DepthImageCalibrated(DepthCVImage->image, VectorOfDetections[i].bounding_box));
	// 	}
	// }
}

void KinectToCVMat::IRImageCallback(const sensor_msgs::Image& IRImage)
{
	ROS_INFO_STREAM("IR Encoding: " << IRImage.encoding);

	cv_bridge::CvImagePtr IRCVImage;
	IRCVImage = cv_bridge::toCvCopy(IRImage, IRImage.encoding);

	// cv::Mat IRCVImage3Channel;
	// cv::cvtColor(IRCVImage->image, IRCVImage3Channel, cv::COLOR_GRAY2BGR);

	// std::vector<DetectedObject> LocalVectorOfDetections;
	// YoloDNN.detect(IRCVImage3Channel, LocalVectorOfDetections);

	// VectorOfDetections = LocalVectorOfDetections;
}

double KinectToCVMat::DepthImageCalibrated(cv::Mat DepthImage, cv::Rect ObjectBoundingBox)
{
	cv::Rect ScalledObjectBoundingBox = cv::Rect( int(ObjectBoundingBox.x*DepthImageSize.width/DetectionImageSize.width), int(ObjectBoundingBox.y*DepthImageSize.height/DetectionImageSize.height), int(ObjectBoundingBox.width*DepthImageSize.width/DetectionImageSize.width), int(ObjectBoundingBox.height*DepthImageSize.height/DetectionImageSize.height));

	cv::Mat CroppedDepthImage = DepthImage(ScalledObjectBoundingBox);

	ROS_INFO_STREAM("Before Resize: " << ObjectBoundingBox);
	ROS_INFO_STREAM("Before Resize: " << ScalledObjectBoundingBox);

	return CroppedDepthImage.at<unsigned short>(CroppedDepthImage.cols/2, CroppedDepthImage.rows/2);
}

}

