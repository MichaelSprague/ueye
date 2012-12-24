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

#include "ueye/Camera.h"

using namespace std;

#if DEBUG_ERROR_CHECKS
#define CHECK_ERR	CheckError
#else
#define CHECK_ERR(fnc)	                        			\
do {                                                       	\
	IS_CHAR* msg;											\
	int err = fnc;                                       	\
	if (err != IS_SUCCESS) {								\
		if (hCam_ != 0){									\
			is_GetError(hCam_, &err, &msg);					\
			if(err != IS_SUCCESS){							\
				throw uEyeException(err, msg);				\
			}												\
		}else{												\
			throw uEyeException(err, "Camera failed to initialize");\
		}													\
	}                                                       \
} while (false)
#endif

namespace ueye{

void Camera::CheckError(INT err)
{
	INT err2 = IS_SUCCESS;
	IS_CHAR* msg;
	if (err != IS_SUCCESS) {
		if (hCam_ != 0){
			is_GetError(hCam_, &err2, &msg);
			if(err2 != IS_SUCCESS){
				throw ueye::uEyeException(err, msg);
			}
		}else{
			throw ueye::uEyeException(err, "Camera failed to initialize");
		}
	}
}

void Camera::InitPrivateVariables()
{
	Streaming_ = false;
	StopCapture_ = false;
	ColorMode_ = RGB;
	AutoExposure_ = false;
	ExposureTime_ = 99.0;
	HardwareGamma_ = true;
	Zoom_ = 1;
	PixelClock_ = 20;
	FrameRate_ = 5.0;
	hCam_ = 0;
	memset(&camInfo_, 0x00, sizeof(camInfo_));
	NumBuffers_ = 0;
	StreamCallback_ = NULL;
}

Camera::Camera()
{
	InitPrivateVariables();
}

bool Camera::checkVersion(int &Major, int &Minor, int &Build, char *&Expected)
{
	Expected = (char*)"4.2.11";
	Build = is_GetDLLVersion();
	Major = (Build >> 24) & 0x000000FF;
	Minor = (Build >> 16) & 0x000000FF;
	Build &= 0x0000FFFF;
	if((Major == 4) && (Minor == 2) && (Build == 11)){
		return true;
	}
	return false;
}

int Camera::getNumberOfCameras()
{
	int Num = 0;
	CHECK_ERR(is_GetNumberOfCameras(&Num));
	return Num;
}

bool Camera::openCamera(unsigned char Id)
{
	if(getNumberOfCameras() < 1){
		return false;
	}

	hCam_ = Id;
	CHECK_ERR(is_InitCamera(&hCam_, 0));

	CHECK_ERR(is_GetSensorInfo (hCam_, &camInfo_));

	setColorMode(ColorMode_);
	setAutoExposure(&AutoExposure_);
	if(!AutoExposure_){
		setExposure(&ExposureTime_);
	}
	setHardwareGamma(&HardwareGamma_);
	setZoom(&Zoom_);
	setPixelClock(&PixelClock_);
	setFrameRate(&FrameRate_);
	return true;
}

char * Camera::getCameraName()
{
	return camInfo_.strSensorName;
}
int Camera::getZoom()
{
	return Zoom_;
}
int Camera::getWidthMax()
{
	return camInfo_.nMaxWidth;
}
int Camera::getHeightMax()
{
	return camInfo_.nMaxHeight;
}
int Camera::getWidth()
{
	return camInfo_.nMaxWidth / Zoom_;
}
int Camera::getHeight()
{
	return camInfo_.nMaxHeight / Zoom_;
}
bool Camera::getAutoExposure()
{
	return AutoExposure_;
}
bool Camera::getHardwareGamma()
{
	return HardwareGamma_;
}
int Camera::getPixelClock()
{
	return PixelClock_;
}

void Camera::setColorMode(uEyeColor mode)
{
	CHECK_ERR(is_SetColorMode(hCam_, mode));
	ColorMode_ = mode;
}
void Camera::setAutoExposure(bool *Enable)
{
	double param1 = *Enable ? 1.0 : 0.0;
	double param2 = 0;
	if(IS_SUCCESS != is_SetAutoParameter(hCam_, IS_SET_ENABLE_AUTO_SHUTTER, &param1, &param2)){
		param1 = 0;
		is_SetAutoParameter(hCam_, IS_SET_ENABLE_AUTO_SHUTTER, &param1, &param2);
		*Enable = false;
	}
	AutoExposure_ = *Enable;
}
void  Camera::setExposure(double *time_ms)
{
	CHECK_ERR(is_Exposure (hCam_, IS_EXPOSURE_CMD_SET_EXPOSURE, time_ms, sizeof(double)));
	ExposureTime_ = *time_ms;
}
void Camera::setHardwareGamma(bool *Enable)
{
	if(*Enable){
		if(IS_SUCCESS != is_SetHardwareGamma(hCam_, IS_SET_HW_GAMMA_ON)){
			is_SetHardwareGamma(hCam_, IS_SET_HW_GAMMA_OFF);
			*Enable = false;
		}
	}else{
		is_SetHardwareGamma(hCam_, IS_SET_HW_GAMMA_OFF);
	}
	HardwareGamma_ = *Enable;
}
void Camera::setZoom(int *zoom)
{
	if(Zoom_ != *zoom){
		// Reset zoom
		is_SetSubSampling(hCam_, 0);
		is_SetBinning(hCam_, 0);

		// Try Subsampling then Binning
		if(IS_SUCCESS != is_SetSubSampling(hCam_, getSubsampleParam(zoom))){
			is_SetSubSampling(hCam_, 0);
			if(IS_SUCCESS != is_SetBinning(hCam_, getBinningParam(zoom))){
				is_SetBinning(hCam_, 0);
				*zoom = 1;
			}
		}

		// Zoom affects the frame-rate and needs a restart to change the buffer sizes
		is_HotPixel(hCam_, IS_HOTPIXEL_ENABLE_CAMERA_CORRECTION, NULL, 0);
		setFrameRate(&FrameRate_);
		restartVideoCapture();
	}
	Zoom_ = *zoom;
}
void Camera::setPixelClock(int *MHz)
{
	int Ranges[3];
	memset(Ranges, 0x00, sizeof(Ranges));

	// Sanitize to increment, minimum, and maximum
	CHECK_ERR(is_PixelClock(hCam_, IS_PIXELCLOCK_CMD_GET_RANGE, Ranges, sizeof(Ranges)));
	if(Ranges[2] > 1){
		if((*MHz - Ranges[0]) % Ranges[2] != 0){
			*MHz -= (*MHz - Ranges[0]) % Ranges[2];
		}
	}
	if(*MHz < Ranges[0]){
		*MHz = Ranges[0];
	}
	if(*MHz > Ranges[1]){
		*MHz = Ranges[1];
	}

	CHECK_ERR(is_PixelClock(hCam_, IS_PIXELCLOCK_CMD_SET, MHz, sizeof(int)));
	setFrameRate(&FrameRate_);
}
void Camera::setFrameRate(double *rate)
{
	CHECK_ERR(is_SetFrameRate(hCam_, *rate, rate));
	FrameRate_ = *rate;
}

int Camera::getSubsampleParam(int *scale)
{
	int param;
	if(*scale == 3){
		*scale = 2;
	}
	switch(*scale){
		case 2:
			param = IS_SUBSAMPLING_2X_VERTICAL | IS_SUBSAMPLING_2X_HORIZONTAL;
			break;
		case 4:
			param = IS_SUBSAMPLING_4X_VERTICAL | IS_SUBSAMPLING_4X_HORIZONTAL;
			break;
		default:
			*scale = 1;
			param = IS_SUBSAMPLING_DISABLE;
			break;
	}
	return param;
}
int Camera::getBinningParam(int *scale)
{
	int param;
	if(*scale == 3){
		*scale = 2;
	}
	switch(*scale){
		case 2:
			param = IS_BINNING_2X_VERTICAL | IS_BINNING_2X_HORIZONTAL;
			break;
		case 4:
			param = IS_BINNING_4X_VERTICAL | IS_BINNING_4X_HORIZONTAL;
			break;
		default:
			*scale = 1;
			param = IS_BINNING_DISABLE;
			break;
	}
	return param;
}

void Camera::closeCamera(){
	if(hCam_ > 0){
		// Release camera and all associated memory
		CHECK_ERR(IS_SUCCESS != is_ExitCamera(hCam_));
		InitPrivateVariables();
	}
}

Camera::~Camera(){
	closeCamera();
}

void Camera::initMemoryPool(int size)
{
	int Width = getWidth();
	int Height = getHeight();
	if(size < 2){
		size = 2;
	}
	imgMem_ = new char* [size];
	imgMemId_ = new int [size];
	for (int i = 0; i < size; i++){
		if (IS_SUCCESS != is_AllocImageMem (hCam_, Width, Height, 24, &imgMem_[i], &imgMemId_[i])){
			throw uEyeException(-1, "Failed to initialize memory.");
		}
		//add memory to  memory pool
		if (IS_SUCCESS != is_SetImageMem(hCam_, imgMem_[i], imgMemId_[i])){
			throw uEyeException(-1, "Failed to initialize memory.");
		}
	}
	NumBuffers_ = size;
}
void Camera::destroyMemoryPool()
{
	for (int i=0; i<NumBuffers_; i++){
		CHECK_ERR(is_FreeImageMem(hCam_,  imgMem_[i], imgMemId_[i]));
	}
	NumBuffers_ = 0;
}

void Camera::startCaptureThread(camCaptureCB callback)
{
	Streaming_ = true;
	int Width = getWidth();
	int Height = getHeight();
	void * imgMem;
	StopCapture_ = false;

	initMemoryPool((int)FrameRate_);

	// Setup video event
	CHECK_ERR(is_EnableEvent(hCam_, IS_SET_EVENT_FRAME));
	CHECK_ERR(is_CaptureVideo(hCam_, IS_WAIT));
	IplImage * p_img = cvCreateImageHeader(cvSize(Width,Height), IPL_DEPTH_8U, 3);

	while(!StopCapture_){
		is_WaitEvent(hCam_, IS_SET_EVENT_FRAME, (int)(2000/FrameRate_));
		is_GetImageMem(hCam_, &imgMem);
		p_img->imageData = (char*)imgMem;
		callback(p_img);
	}

	// Stop video event
	CHECK_ERR(is_DisableEvent(hCam_, IS_SET_EVENT_FRAME));
	CHECK_ERR(is_StopLiveVideo(hCam_, IS_WAIT));

	destroyMemoryPool();
	Streaming_ = false;
}

void Camera::startVideoCapture(camCaptureCB callback){
	StreamCallback_ = callback;
	VidThread_ = boost::thread(&Camera::startCaptureThread, this, callback);
}
void Camera::stopVideoCapture(){
	StopCapture_ = true;
	if (VidThread_.joinable()){
		VidThread_.join();
	}
}
void Camera::restartVideoCapture(){
	if(Streaming_){
		if(StreamCallback_ != NULL){
			stopVideoCapture();
			startVideoCapture(StreamCallback_);
		}
	}
}

}//namespace ueye
