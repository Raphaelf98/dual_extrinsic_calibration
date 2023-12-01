cmake_minimum_required(VERSION 3.0.2)
project(dual_extrinsic_calibration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  message_filters
  tf
  geometry_msgs
)

## System dependencies are found with CMake's conventions

find_package(OpenCV 4.2.0 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  
)

## Declare a C++ library
add_library(poseEstimation
src/poseEstimation.cpp include/${PROJECT_NAME}/poseEstimation.h
 )
target_include_directories(poseEstimation PRIVATE include/dual_extrinsic_calibration)
target_link_libraries(poseEstimation Eigen3::Eigen ${catkin_LIBRARIES})

add_library(dualExtrinsicCalibration
src/dualExtrinsicCalibration.cpp
include/${PROJECT_NAME}/dualExtrinsicCalibration.h
 )
target_include_directories(dualExtrinsicCalibration PRIVATE include/dual_extrinsic_calibration)
target_link_libraries(dualExtrinsicCalibration Eigen3::Eigen poseEstimation )

add_executable(${PROJECT_NAME}_node src/dualExtrinsicCalibrationNode.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE include/dual_extrinsic_calibration)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   dualExtrinsicCalibration
 )
add_executable(tfPosePublisher src/tfPosePublisher.cpp)
target_link_libraries(tfPosePublisher
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   dualExtrinsicCalibration
 )


