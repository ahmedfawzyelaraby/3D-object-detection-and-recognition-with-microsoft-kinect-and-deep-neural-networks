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

	DetectionsTopicName = ParamNameSeparator + NodeName + ParamNameSeparator + DetectionsTopicName;

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
	// IRImageSubscriber = nodeHandle_.subscribe(IRImageTopicName, 1, &KinectToCVMat::IRImageCallback, this);

	DetectionsAndDepthsPublisher = nodeHandle_.advertise<kinect_yolo::DepthAndDetections>(DetectionsTopicName, 1, this);

	YoloDNN.setConfigFilePath(CFGFilePath.c_str());
	YoloDNN.setDataFilePath(DataFilePath.c_str());
	YoloDNN.setWeightFilePath(WeightsFilePath.c_str());
	YoloDNN.setAlphabetPath(LabelsFilePath.c_str());
	YoloDNN.setNameListFile(NamesFilePath.c_str());
	
	VectorOfClassesNames = GetClassesNames(NamesFilePath);

	VectorOfDetections.clear();
	VectorOfDepthOfDetections.clear();

	InitiateClassesColors(VectorOfClassesNames.size());
}

KinectToCVMat::~KinectToCVMat()
{
}

void KinectToCVMat::RGBImageCallback(const sensor_msgs::Image& RGBImage)
{
	cv_bridge::CvImagePtr RGBCVImage;
	RGBCVImage = cv_bridge::toCvCopy(RGBImage, RGBImage.encoding);

	DetectionImageSize = RGBCVImage->image.size();

	std::vector<DetectedObject> LocalVectorOfDetections;
	YoloDNN.detect(RGBCVImage->image, LocalVectorOfDetections);

	VectorOfDetections = LocalVectorOfDetections;
}

void KinectToCVMat::DepthImageCallback(const sensor_msgs::Image& DepthImage)
{
	cv_bridge::CvImagePtr DepthCVImage;
	DepthCVImage = cv_bridge::toCvCopy(DepthImage, DepthImage.encoding);

	DepthImageSize = DepthCVImage->image.size();



	std::vector<double> LocalVectorOfDepthOfDetections;
	std::vector<unsigned int> DetectionsX;
	std::vector<unsigned int> DetectionsY;
	std::vector<unsigned int> DetectionsW;
	std::vector<unsigned int> DetectionsH;
	std::vector<std::string> DetectionsN;
	std::vector<unsigned int> DetectionsR;
	std::vector<unsigned int> DetectionsG;
	std::vector<unsigned int> DetectionsB;
	if (!(VectorOfDetections.empty()))
	{
		for (int i = 0; i < VectorOfDetections.size(); i++)
		{
			DetectionsX.push_back(VectorOfDetections[i].bounding_box.x);
			DetectionsY.push_back(VectorOfDetections[i].bounding_box.y);
			DetectionsW.push_back(VectorOfDetections[i].bounding_box.width);
			DetectionsH.push_back(VectorOfDetections[i].bounding_box.height);
			DetectionsN.push_back(VectorOfClassesNames[VectorOfDetections[i].object_class]);
			DetectionsR.push_back(VectorOfClassesRColor[VectorOfDetections[i].object_class]);
			DetectionsG.push_back(VectorOfClassesGColor[VectorOfDetections[i].object_class]);
			DetectionsB.push_back(VectorOfClassesBColor[VectorOfDetections[i].object_class]);
			LocalVectorOfDepthOfDetections.push_back(this->CalibratedDepthValue(DepthCVImage->image, VectorOfDetections[i].bounding_box));
		}
	}
	VectorOfDepthOfDetections = LocalVectorOfDepthOfDetections;

	kinect_yolo::DepthAndDetections DetectionsMessage;
	DetectionsMessage.Light = true;
	DetectionsMessage.VectorLength = VectorOfDetections.size();
	DetectionsMessage.DetectionsX = DetectionsX;
	DetectionsMessage.DetectionsY = DetectionsY;
	DetectionsMessage.DetectionsWidth = DetectionsW;
	DetectionsMessage.DetectionsHeight = DetectionsH;
	DetectionsMessage.EquivalentDepth = VectorOfDepthOfDetections;
	DetectionsMessage.DetectionsName = DetectionsN;
	DetectionsMessage.DetectionsRColor = DetectionsR;
	DetectionsMessage.DetectionsGColor = DetectionsG;
	DetectionsMessage.DetectionsBColor = DetectionsB;
	DetectionsAndDepthsPublisher.publish(DetectionsMessage);
}

// void KinectToCVMat::IRImageCallback(const sensor_msgs::Image& IRImage)
// {
// 	ROS_INFO_STREAM("IR Encoding: " << IRImage.encoding);

// 	cv_bridge::CvImagePtr IRCVImage;
// 	IRCVImage = cv_bridge::toCvCopy(IRImage, IRImage.encoding);

// 	// cv::Mat IRCVImage3Channel;
// 	// cv::cvtColor(IRCVImage->image, IRCVImage3Channel, cv::COLOR_GRAY2BGR);

// 	// std::vector<DetectedObject> LocalVectorOfDetections;
// 	// YoloDNN.detect(IRCVImage3Channel, LocalVectorOfDetections);

// 	// VectorOfDetections = LocalVectorOfDetections;
// }

double KinectToCVMat::CalibratedDepthValue(cv::Mat DepthImage, cv::Rect ObjectBoundingBox)
{
	cv::Rect ScalledObjectBoundingBox = cv::Rect( int(ObjectBoundingBox.x*DepthImageSize.width/DetectionImageSize.width), int(ObjectBoundingBox.y*DepthImageSize.height/DetectionImageSize.height), int(ObjectBoundingBox.width*DepthImageSize.width/DetectionImageSize.width), int(ObjectBoundingBox.height*DepthImageSize.height/DetectionImageSize.height));

	cv::Mat CroppedDepthImage = DepthImage(ScalledObjectBoundingBox);

	// double returnValue = (cv::mean(CroppedDepthImage, CroppedDepthImage != this->minimumClippingValueMM).val[0] + cv::mean(CroppedDepthImage, CroppedDepthImage < this->maximumClippingValueMM).val[0]) / 2.0;

	// return returnValue / this->ConversionFromMiliMeterToMeter; 
	//return CroppedDepthImage.at<unsigned short>(CroppedDepthImage.cols/2, CroppedDepthImage.rows/2) / this->ConversionFromMiliMeterToMeter;
	// test which is better
	return depthCalculationAlgorithm(CroppedDepthImage);
}

std::vector<std::string> KinectToCVMat::GetClassesNames(std::string NamesFilePath)
{
	std::ifstream NamesFile(NamesFilePath);
	std::vector<std::string> ClassesVector;
	
	std::string Line;

	while(std::getline(NamesFile, Line))
	{
		ClassesVector.push_back(Line);
	}

	return ClassesVector;
}

void KinectToCVMat::InitiateClassesColors(int NumberOfClasses)
{
	srand(time(0));
	for (int iterator = 0; iterator < NumberOfClasses; iterator++)
	{
		int R = (rand() % 256);
		int G = (rand() % 256);
		int B = (rand() % 256);
		VectorOfClassesRColor.push_back(R);
		VectorOfClassesGColor.push_back(G);
		VectorOfClassesBColor.push_back(B);
	}
}

double KinectToCVMat::depthCalculationAlgorithm(cv::Mat depthImageBoundingBoxedPart)
{
	int girdCellsSideNumber = sqrt(this->depthAlgorithmNumberOfGridCells);
	int gridCellWidth = double(depthImageBoundingBoxedPart.cols) / double(girdCellsSideNumber);
	int gridCellHeight = double(depthImageBoundingBoxedPart.rows) / double(girdCellsSideNumber);
	std::vector<double> listOfGridCellsMeans;
	for(int widthIterator = 0; widthIterator < girdCellsSideNumber; widthIterator++)
	{
		for(int heightIterator = 0; heightIterator < girdCellsSideNumber; heightIterator++)
		{
			cv::Mat gridImage = depthImageBoundingBoxedPart(cv::Rect((widthIterator * gridCellWidth), (heightIterator * gridCellHeight), ((((widthIterator+1) * gridCellWidth) <= depthImageBoundingBoxedPart.cols) ? gridCellWidth : (depthImageBoundingBoxedPart.cols - widthIterator * gridCellWidth)), ((((heightIterator+1) * gridCellHeight) <= depthImageBoundingBoxedPart.rows) ? gridCellHeight : (depthImageBoundingBoxedPart.rows - heightIterator * gridCellHeight))));
			listOfGridCellsMeans.push_back((cv::mean(gridImage, gridImage != this->minimumClippingValueMM).val[0] + cv::mean(gridImage, gridImage < this->maximumClippingValueMM).val[0]) / 2.0);
		}
	}
	std::sort(listOfGridCellsMeans.begin(), listOfGridCellsMeans.end());
	return listOfGridCellsMeans[this->depthAlgorithmNumberOfGridCells / 2] / this->ConversionFromMiliMeterToMeter; 
}

}
