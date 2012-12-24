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

#include <ueye/CameraNode.h>

namespace ueye {

CameraNode::CameraNode(ros::NodeHandle node, ros::NodeHandle priv_nh)
: it_(node)
{
	running_ = false;
	configured_ = false;
	force_streaming_ = false;

	// Check for a valid uEye installation and supported version
	char *Version;
	int Major, Minor, Build;
	if(cam_.checkVersion(Major, Minor, Build, Version)){
		ROS_INFO("Loaded uEye SDK %s.", Version);
	}else{
		ROS_WARN("Loaded uEye SDK %d.%d.%d. Expecting %s.", Major, Minor, Build, Version);
	}

	// Grab the cameraId from the ROS parameter
	int cameraId = 0;
	priv_nh.getParam("cameraId", cameraId);

	// Make sure there is at least one camera available
	int NumberOfCameras = cam_.getNumberOfCameras();
	if(NumberOfCameras > 0){
		if(NumberOfCameras == 1){
			ROS_INFO("Found 1 uEye camera.");
		}else{
			ROS_INFO("Found %d uEye cameras.", NumberOfCameras);
		}
	}else{
		ROS_ERROR("Found 0 uEye cameras.");
		ros::shutdown();
		return;
	}

	// Open the camera
	if(!cam_.openCamera(cameraId)){
		ROS_ERROR("Failed to open uEye camera.");
		ros::shutdown();
		return;
	}
	ROS_INFO("Opened camera %s.", cam_.getCameraName());

	// Try to load intrinsics from file.
	loadIntrinsics();

	// Setup Dynamic Reconfigure
	dynamic_reconfigure::Server<cameraConfig>::CallbackType f = boost::bind(&CameraNode::reconfig, this, _1, _2);
	srv_.setCallback(f);	//start dynamic reconfigure

	// Service call for setting calibration.
	srv_cam_info_ = node.advertiseService("set_camera_info", &CameraNode::setCameraInfo, this);

	// Special publisher for images to support compression
	pub_stream_ = it_.advertiseCamera("image_raw", 0);

	// Set up Timer
	timer_ = node.createTimer(ros::Duration(1/5.0), &CameraNode::timerCallback, this);
}

CameraNode::~CameraNode()
{
	stopCamera();
}

void CameraNode::reconfig(cameraConfig &config, uint32_t level)
{
	force_streaming_ = config.force_streaming;
	msg_image_.header.frame_id = msg_camera_info_.header.frame_id = config.frame_id;

	// Hardware Gamma Correction
	if (cam_.getHardwareGamma() != config.hardware_gamma){
		cam_.setHardwareGamma(&config.hardware_gamma);
	}

	// Zoom
	if (cam_.getZoom() != config.zoom){
		cam_.setZoom(&config.zoom);
	}

	// Pixel Clock
	if (cam_.getPixelClock() != config.pixel_clock){
		cam_.setPixelClock(&config.pixel_clock);
	}

	// Frame Rate
	cam_.setFrameRate(&config.frame_rate);

	// Exposure
	if (cam_.getAutoExposure() != config.auto_exposure){
		cam_.setAutoExposure(&config.auto_exposure);
	}
	if (!config.auto_exposure){
		cam_.setExposure(&config.exposure_time);
	}

	configured_ = true;
}

void CameraNode::timerCallback(const ros::TimerEvent& event)
{
	if((pub_stream_.getNumSubscribers() > 0) || force_streaming_){
		startCamera();
	}else{
		stopCamera();
	}
}

// Support camera calibration requests
// http://www.ros.org/wiki/camera_calibration/Tutorials/MonocularCalibration
bool CameraNode::setCameraInfo(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& rsp)
{
	ROS_INFO("New camera info received");
	sensor_msgs::CameraInfo &info = req.camera_info;

	// Sanity check: the image dimensions should match the resolution of the sensor.
	unsigned int height = cam_.getHeight();
	unsigned int width = cam_.getWidth();

	if (info.width != width || info.height != height) {
		rsp.success = false;
		rsp.status_message = (boost::format("Camera_info resolution %ix%i does not match current video "
		"setting, camera running at resolution %ix%i.")
		% info.width % info.height % width % height).str();
		ROS_ERROR("%s", rsp.status_message.c_str());
		return true;
	}

	std::string camname = cam_.getCameraName();
	std::stringstream ini_stream;
	if (!camera_calibration_parsers::writeCalibrationIni(ini_stream, camname, info)) {
		rsp.status_message = "Error formatting camera_info for storage.";
		rsp.success = false;
	}else{
		std::string ini = ini_stream.str();
		fstream param_file;
		std::string filename = ros::package::getPath("ueye") + "/Calibration-";
		filename += cam_.getCameraName();
		filename += ".txt";
		param_file.open(filename.c_str(), ios::in | ios::out | ios::trunc);

		if (param_file.is_open()) {
			param_file<< ini.c_str();
			param_file.close();

			msg_camera_info_ = info;
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

// Try to load previously saved camera calibration from a file.
void CameraNode::loadIntrinsics()
{
	char buffer[12800];

	std::string MyPath = ros::package::getPath("ueye") + "/Calibration-" + cam_.getCameraName() + ".txt";
	fstream param_file;
	param_file.open(MyPath.c_str(), ios::out | ios::in);

	if (param_file.is_open()) {
		param_file.read(buffer, 12800);
		param_file.close();
	}

	// Parse calibration file
	std::string camera_name;
	if (camera_calibration_parsers::parseCalibrationIni(buffer, camera_name, msg_camera_info_)){
		ROS_INFO("Loaded calibration for camera '%s'", camera_name.c_str());
	}else{
		ROS_WARN("Failed to load intrinsics for camera from file");
	}
}

// Add properties to image message
void CameraNode::processFrame(IplImage* frame, sensor_msgs::Image &img, sensor_msgs::CameraInfo &cam_info)
{
	cam_info.header.stamp = ros::Time::now();
	cam_info.header.seq++;
	cam_info.roi.x_offset = 0;
	cam_info.roi.y_offset = 0;
	cam_info.height = cam_.getHeight();
	cam_info.width = cam_.getWidth();
	cam_info.roi.width = cam_info.width;
	cam_info.roi.height = cam_info.height;

	cv::Mat imgMat = frame;
	cv_bridge::CvImage ImageConverter(cam_info.header, "bgr8", imgMat);
	ImageConverter.toImageMsg(img);
}

// Timestamp and publish the image. Called by the streaming thread.
void CameraNode::publishImage(IplImage * frame)
{
	processFrame(frame, msg_image_, msg_camera_info_);
	pub_stream_.publish(msg_image_, msg_camera_info_);
}

void CameraNode::startCamera()
{
	if(running_ || !configured_)
		return;
	ROS_INFO("Started video stream.");
	cam_.startVideoCapture(boost::bind(&CameraNode::publishImage, this, _1));
	running_ = true;
}

void CameraNode::stopCamera()
{
	if(!running_)
		return;
	ROS_INFO("Stopped video stream.");
	cam_.stopVideoCapture();
	running_ = false;
}

} // namespace ueye
