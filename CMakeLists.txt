cmake_minimum_required(VERSION 2.8.3)
project(kinect_yolo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
#  LIBRARIES
  CATKIN_DEPENDS 
    roscpp
    sensor_msgs
    cv_bridge
    image_transport
#  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} #${SOURCE_FILES} )
  src/${PROJECT_NAME}_node.cpp
  src/KinectToCVMat.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)


