/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Kevin Hallenbeck
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Kevin Hallenbeck nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// ROS communication
#include <ros/ros.h>
#include <ros/package.h>	// finds package paths
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse_ini.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include "ueye/cameraConfig.h"

// File IO
#include <sstream>
#include <fstream>

#include "ueye_camera.h"


// ROS messages
sensor_msgs::Image MsgImage;
sensor_msgs::CameraInfo MsgCameraInfo;

ros::ServiceServer SrvSetCameraInfo;
image_transport::CameraPublisher PubStream;

// Global pointer to the instance of the UEyeUSB class
ueye::Camera *g_cam;

bool g_running = false;
bool g_configured = false;
bool g_force_streaming = false;

// Add properties to image message
void processFrame(IplImage* frame, sensor_msgs::Image &img, sensor_msgs::CameraInfo &cam_info)
{
	cam_info.header.stamp = ros::Time::now();
	cam_info.header.seq++;
	cam_info.roi.x_offset = 0;
	cam_info.roi.y_offset = 0;
	cam_info.height = g_cam->getHeight();
	cam_info.width = g_cam->getWidth();
	cam_info.roi.width = cam_info.width;
	cam_info.roi.height = cam_info.height;

	cv::Mat imgMat = frame;
	cv_bridge::CvImage ImageConverter(cam_info.header, "bgr8", imgMat);
	ImageConverter.toImageMsg(img);
}

// Timestamp and publish the image. Called by the streaming thread.
void publishImage(IplImage * frame)
{
	processFrame(frame, MsgImage, MsgCameraInfo);
	PubStream.publish(MsgImage, MsgCameraInfo);
}

void startCamera()
{
	if(g_running || !g_configured)
		return;
	ROS_INFO("Started video stream.");
	g_cam->startVideoCapture(publishImage);
	g_running = true;
}

void stopCamera()
{
	if(!g_running)
		return;
	ROS_INFO("Stopped video stream.");
	g_cam->stopVideoCapture();
	g_running = false;
}

// Handle Dynamic Reconfigure requests
void configure(ueye::cameraConfig& config, uint32_t level)
{
	g_force_streaming = config.force_streaming;
	MsgImage.header.frame_id = MsgCameraInfo.header.frame_id = config.frame_id;

	// Hardware Gamma Correction
	if (g_cam->getHardwareGamma() != config.hardware_gamma){
		g_cam->setHardwareGamma(&config.hardware_gamma);
	}

	// Zoom
	if (g_cam->getZoom() != config.zoom){
		g_cam->setZoom(&config.zoom);
	}

	// Pixel Clock
	if (g_cam->getPixelClock() != config.pixel_clock){
		g_cam->setPixelClock(&config.pixel_clock);
	}

	// Frame Rate
	g_cam->setFrameRate(&config.frame_rate);

	// Exposure
	if (g_cam->getAutoExposure() != config.auto_exposure){
		g_cam->setAutoExposure(&config.auto_exposure);
	}
	if (!config.auto_exposure){
		g_cam->setExposure(&config.exposure_time);
	}

	g_configured = true;
}

// Try to load previously saved camera calibration from a file.
void loadIntrinsics()
{
	char buffer[12800];

	std::string MyPath = ros::package::getPath("ueye") + "/Calibration-" + g_cam->getCameraName() + ".txt";
	fstream param_file;
	param_file.open(MyPath.c_str(), ios::out | ios::in);

	if (param_file.is_open()) {
		param_file.read(buffer, 12800);
		param_file.close();
	}

	// Parse calibration file
	std::string camera_name;
	if (camera_calibration_parsers::parseCalibrationIni(buffer, camera_name, MsgCameraInfo)){
		ROS_INFO("Loaded calibration for camera '%s'", camera_name.c_str());
	}else{
		ROS_WARN("Failed to load intrinsics for camera from file");
	}
}

// Support camera calibration requests
// http://www.ros.org/wiki/camera_calibration/Tutorials/MonocularCalibration
bool setCameraInfo(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& rsp)
{
	ROS_INFO("New camera info received");
	sensor_msgs::CameraInfo &info = req.camera_info;

	// Sanity check: the image dimensions should match the resolution of the sensor.
	unsigned int height = g_cam->getHeight();
	unsigned int width = g_cam->getWidth();

	if (info.width != width || info.height != height) {
		rsp.success = false;
		rsp.status_message = (boost::format("Camera_info resolution %ix%i does not match current video "
		"setting, camera running at resolution %ix%i.")
		% info.width % info.height % width % height).str();
		ROS_ERROR("%s", rsp.status_message.c_str());
		return true;
	}

	std::string g_camname = g_cam->getCameraName();
	std::stringstream ini_stream;
	if (!camera_calibration_parsers::writeCalibrationIni(ini_stream, g_camname, info)) {
		rsp.status_message = "Error formatting camera_info for storage.";
		rsp.success = false;
	}else{
		std::string ini = ini_stream.str();
		fstream param_file;
		std::string filename = ros::package::getPath("ueye") + "/Calibration-";
		filename += g_cam->getCameraName();
		filename += ".txt";
		param_file.open(filename.c_str(), ios::in | ios::out | ios::trunc);

		if (param_file.is_open()) {
			param_file<< ini.c_str();
			param_file.close();

			MsgCameraInfo = info;
			rsp.success = true;
		}else{
			rsp.success = false;
			rsp.status_message = "file write failed";
		}
	}
	if (!rsp.success){
		ROS_ERROR("%s", rsp.status_message.c_str());
	}
	return true;
}

// This timer starts streaming when there are any subscribers,
// and stops streaming if there are none.
void timerCallback(const ros::TimerEvent& event)
{
	if((PubStream.getNumSubscribers() > 0) || g_force_streaming){
		startCamera();
	}else{
		stopCamera();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera");
	ros::NodeHandle n("camera");

	ueye::Camera cam;
	g_cam = &cam;

	// Check for a valid uEye installation and supported version
	char *Version;
	int Major, Minor, Build;
	if(cam.checkVersion(Major, Minor, Build, Version)){
		ROS_INFO("Loaded uEye SDK %s.", Version);
	}else{
		ROS_WARN("Loaded uEye SDK %d.%d.%d. Expecting %s.", Major, Minor, Build, Version);
	}

	// Make sure there is at least one camera available
	int NumberOfCameras = cam.getNumberOfCameras();
	if(NumberOfCameras > 0){
		if(NumberOfCameras == 1){
			ROS_INFO("Found 1 uEye camera.");
		}else{
			ROS_INFO("Found %d uEye cameras.", NumberOfCameras);
		}
	}else{
		ROS_ERROR("Found 0 uEye cameras.");
		ros::shutdown();
		sleep(5);
		return 0;
	}

	// Grab the cameraId from the ROS parameter
	ros::NodeHandle nh_ns("~");
	int cameraId;
	nh_ns.param("cameraId", cameraId, 0);

	// Open the camera
	if(!cam.openCamera(cameraId)){
		ROS_ERROR("Failed to open uEye camera.");
		ros::shutdown();
		sleep(5);
		return 0;
	}
	ROS_INFO("Opened camera %s.", cam.getCameraName());

	// Try to load intrinsics from file.
	loadIntrinsics();

	// Service call for setting calibration.
	SrvSetCameraInfo = n.advertiseService("set_camera_info", setCameraInfo);

	// Special publisher for images to support compression
	image_transport::ImageTransport it(n);
	PubStream = it.advertiseCamera("image_raw", 1);

	// Dynamic Reconfigure
	dynamic_reconfigure::Server<ueye::cameraConfig> srv;
	dynamic_reconfigure::Server<ueye::cameraConfig>::CallbackType f = boost::bind(configure, _1, _2);
	srv.setCallback(f);

	// Timer to start and stop the stream based on the number of subscribers
	ros::Timer StreamTimer = n.createTimer(ros::Duration(1/5.0), timerCallback);

	ros::spin();

	stopCamera();

	return 0;
}
