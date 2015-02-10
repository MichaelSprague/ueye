cmake_minimum_required(VERSION 2.8.3)
project(ueye)

# set default build type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

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
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${${PROJECT_NAME}_INCLUDE_EXPORTS}
)

# Check for installed uEye SDK
include(CheckIncludeFileCXX)
check_include_file_cxx("ueye.h" HAVE_UEYE_H)
if (NOT HAVE_UEYE_H)
  include(TargetArch.cmake)
  target_architecture(TARGET_ARCH)
  message("-- uEye SDK not found. Using downloaded files for '${TARGET_ARCH}' architecture.")

  if (TARGET_ARCH STREQUAL "amd64")
    file(DOWNLOAD 
      https://bitbucket.org/kmhallen/ueye/downloads/uEye_SDK_4_60_amd64.tar.gz
      #http://download.ros.org/data/ueye/uEye_SDK_4_60_amd64.tar.gz
      ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/3rdparty/uEye_SDK_amd64.tar.gz
      SHOW_PROGRESS
      INACTIVITY_TIMEOUT 60
      EXPECTED_MD5 e56ede9549b84e9093a1d3b4b9ff9344
      TLS_VERIFY on)
    execute_process(
      COMMAND ${CMAKE_COMMAND} -E tar xzf ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/3rdparty/uEye_SDK_amd64.tar.gz
      WORKING_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/3rdparty
    )
    include_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/3rdparty/amd64)
    link_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/3rdparty/amd64)
  elseif (TARGET_ARCH STREQUAL "i386")
    file(DOWNLOAD 
      https://bitbucket.org/kmhallen/ueye/downloads/uEye_SDK_4_60_i386.tar.gz
      #http://download.ros.org/data/ueye/uEye_SDK_4_60_i386.tar.gz
      ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/3rdparty/uEye_SDK_i386.tar.gz
      SHOW_PROGRESS
      INACTIVITY_TIMEOUT 60
      EXPECTED_MD5 20db4268dd3c0416d289067277fffef7
      TLS_VERIFY on)
    execute_process(
      COMMAND ${CMAKE_COMMAND} -E tar xzf ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/3rdparty/uEye_SDK_i386.tar.gz
      WORKING_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/3rdparty
    )
    include_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/3rdparty/i386)
    link_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/3rdparty/i386)
  elseif (TARGET_ARCH STREQUAL "arm")
    file(DOWNLOAD 
      https://bitbucket.org/kmhallen/ueye/downloads/uEye_SDK_4_30_arm.tar.gz
      ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/3rdparty/uEye_SDK_arm.tar.gz
      SHOW_PROGRESS
      INACTIVITY_TIMEOUT 60
      EXPECTED_MD5 0297e76450084ab21ff9db1a3070f323
      TLS_VERIFY on)
    execute_process(
      COMMAND ${CMAKE_COMMAND} -E tar xzf ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/3rdparty/uEye_SDK_arm.tar.gz
      WORKING_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/3rdparty
    )
    include_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/3rdparty/arm)
    link_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/3rdparty/arm)
  else ()
    message("-- Warning: Downloads for '${TARGET_ARCH}' architecture not found.")
  endif ()
endif (NOT HAVE_UEYE_H)

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

install(TARGETS ueye_nodelets camera stereo framerate
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)

  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

   # parse check all the launch/*.launch files
  roslaunch_add_file_check(launch)

endif (CATKIN_ENABLE_TESTING)
