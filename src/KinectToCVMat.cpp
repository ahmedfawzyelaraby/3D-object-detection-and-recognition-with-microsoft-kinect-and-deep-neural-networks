#include "kinect_yolo/KinectToCVMat.hpp"

namespace kinect_yolo {

KinectToCVMat::KinectToCVMat(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle)
{
	RGBImageSubscriber = nodeHandle_.subscribe("/camera/rgb/image_color",10, &KinectToCVMat::RGBImageCallback, this);
	DepthImageSubscriber = nodeHandle_.subscribe("/camera/depth/image",10, &KinectToCVMat::DepthImageCallback, this);

	YoloDNN.setConfigFilePath("/home/ahmedfawzyelaraby/Thesis_WS/Code/YOLO_Darknet/darknet_old/cfg/tiny-yolo-voc.cfg");
	YoloDNN.setDataFilePath("/home/ahmedfawzyelaraby/Thesis_WS/Code/YOLO_Darknet/darknet_old/cfg/voc.data");
	YoloDNN.setWeightFilePath("/home/ahmedfawzyelaraby/Thesis_WS/Code/YOLO_Darknet/darknet_old/tiny-yolo-voc.weights");
	YoloDNN.setAlphabetPath("/home/ahmedfawzyelaraby/Thesis_WS/Code/YOLO_Darknet/darknet_old/data/labels/");
	YoloDNN.setNameListFile("/home/ahmedfawzyelaraby/Thesis_WS/Code/YOLO_Darknet/darknet/data/voc.names");
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

