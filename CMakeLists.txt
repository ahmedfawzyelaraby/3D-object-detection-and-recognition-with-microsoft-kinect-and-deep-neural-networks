CMAKE_MINIMUM_REQUIRED  (VERSION 2.8.3)
PROJECT(kinect_yolo)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge message_generation)
FIND_PACKAGE(CUDA REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
  
ADD_MESSAGE_FILES(DIRECTORY msg FILES DepthAndDetections.msg)
GENERATE_MESSAGES(DEPENDENCIES std_msgs)

CATKIN_PACKAGE(INCLUDE_DIRS include CATKIN_DEPENDS roscpp sensor_msgs cv_bridge message_runtime) #LIBRARIES #DEPENDS 

INCLUDE_DIRECTORIES(include ${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(/usr/local/include/yolo-v2-gpu-wrapper)
INCLUDE_DIRECTORIES(${CUDA_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECT_NAME} src/${PROJECT_NAME}_node.cpp src/KinectToCVMat.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} libYOLOGPUWrapper.so)
