cmake_minimum_required(VERSION 2.8.3)
project(ueye)

set(${PROJECT_NAME}_CATKIN_DEPS
  roscpp
  nodelet
  dynamic_reconfigure
  std_msgs
  image_transport
  camera_calibration_parsers
  cv_bridge
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
find_package(OpenCV REQUIRED)

set(${PROJECT_NAME}_INCLUDE_EXPORTS
  ${OpenCV_INCLUDE_DIRS}
  include
)

generate_dynamic_reconfigure_options(cfg/mono.cfg cfg/stereo.cfg)

catkin_package(
  INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE_EXPORTS}
  LIBRARIES ueye_nodelets
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
  DEPENDS OpenCV
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${${PROJECT_NAME}_INCLUDE_EXPORTS}
)

add_library(ueye_nodelets
  src/Camera.cpp
  src/camera_nodelet.cpp src/CameraNode.cpp
  src/stereo_nodelet.cpp src/StereoNode.cpp
  src/framerate_nodelet.cpp src/FramerateNode.cpp
)
add_dependencies(ueye_nodelets ${PROJECT_NAME}_gencfg)
target_link_libraries(ueye_nodelets
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ueye_api
)

add_executable(camera src/camera_node.cpp)
target_link_libraries(camera
  ueye_nodelets
)

add_executable(stereo src/stereo_node.cpp)
target_link_libraries(stereo
  ueye_nodelets
)

add_executable(framerate src/framerate_node.cpp)
target_link_libraries(framerate
  ueye_nodelets
)

