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

#ifndef UEYE_CAMERA_H_
#define UEYE_CAMERA_H_

#include <uEye.h>
#include <opencv/cv.h>
#include <stdexcept>
#include <string>
#include <boost/function.hpp>
#include <boost/thread.hpp>

using namespace std;

namespace ueye{

	struct uEyeException : public std::runtime_error
	{
		int error_code;
		uEyeException(int code, const char* msg)
			: std::runtime_error(msg), error_code(code)
		{}
	};

	enum uEyeColor{
		RGB		= IS_CM_BGR8_PACKED,
		YCbYCr	= IS_CM_CBYCRY_PACKED,
	};

	class Camera
	{
	public:
		Camera();
		~Camera();

		// Initialization functions in order they should be called.
		bool checkVersion(int &Major, int &Minor, int &Build, char *&Expected);
		int getNumberOfCameras();
		bool openCamera(unsigned char Id);

		// Get Properties
		char * getCameraName();
		int getWidthMax();
		int getHeightMax();
		int getWidth();
		int getHeight();
		int getZoom();
		bool getAutoExposure();
		bool getHardwareGamma();
		int getPixelClock();

		// Set Properties
		void setColorMode(uEyeColor mode);
		void setAutoExposure(bool *Enable);
		void setExposure(double *time_ms);
		void setHardwareGamma(bool *Enable);
		void setZoom(int *zoom);
		void setPixelClock(int *MHz);
		void setFrameRate(double *rate);

		typedef boost::function<void (IplImage *)> camCaptureCB;
		void startVideoCapture(camCaptureCB);
		void stopVideoCapture();

		void closeCamera();

	private:
		void CheckError(INT err);
		void InitPrivateVariables();
		int getSubsampleParam(int *scale);
		int getBinningParam(int *scale);

		char **imgMem_;
		int  *imgMemId_;
		int NumBuffers_;
		void initMemoryPool(int size);
		void destroyMemoryPool();
		void startCaptureThread(camCaptureCB captureCallback);
		void restartVideoCapture();

		uEyeColor ColorMode_;
		bool AutoExposure_;
		double ExposureTime_;
		bool HardwareGamma_;
		int Zoom_;
		int PixelClock_;
		double FrameRate_;
		HIDS hCam_;
		SENSORINFO camInfo_;

		bool Streaming_;
		bool StopCapture_;
		camCaptureCB StreamCallback_;
		boost::thread VidThread_;

	};
}//namespace ueye

#endif /* UEYE_CAMERA_H_ */
