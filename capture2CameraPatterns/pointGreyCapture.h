/*
	 Image acqusition using FlyCapture2 API
	 Version 1:
	 Author: Song Zhang (song.zhang@VEOptics.com)
	 Date: March 29, 2020

	 Version 2:
	 Author: Song Zhang (song.zhang@VEOptics.com)
	 Date: April 10, 2020
	 Changes are made to consider streaming case and single snapshot or single set capture
	 Streaming mode has the advantages of speed, yet could be unstable the a user is doing
	 something else after image acqusition.
	 None streaming mode is robust and stable yet with the caveat of slower speed because
	 camera has to start and stop for each call.
*/

#pragma once
#include "FlyCapture2.h"
#pragma comment(lib, "FlyCapture2_v140.lib")

using namespace FlyCapture2;

class pointGreyCapture
{
public:
	pointGreyCapture();
	~pointGreyCapture();

public:
	bool openCamera(unsigned int cameraSerialNumber = 0);
	bool closeCamera();

	bool initCamera(unsigned int imageWidth, unsigned int imageHeight, unsigned int offsetX, unsigned int offsetY, float frameRate = 60.0f, float exposureTime_ms = 2.0f, bool isHardwareTrigger = true);
	bool setExposureTime(float& exposureTimeToSet);
	bool setFrameRate(float frameRateToSet);
	bool setGainValue(float gainvalueToSet = 1.0f);
	bool setHardwareTrigger(bool isHardwareTrigger = true);
	bool setWhiteBalance(float gainRedToSet, float gainBlueToSet);

	bool captureSingleImage(Image& captureImage, bool isStreamMode = false);
	bool captureImageSet(Image captureImage[], int numberOfFrames, int firstFrameCounter = 0, bool isStreamMode = true);
	bool captureSingleImageData(unsigned char* captureImage, bool isStreamMode = false);
	bool captureImageSetData(unsigned char *captureImage[], int numberOfFrames, int firstFrameCounter = 0, bool isStreamMode = true);

	bool startAcquisition();
	bool stopAcquisition();

private:
	bool _checkLogError(FlyCapture2::Error error);
	bool setImageResolution(unsigned int& widthToSet, unsigned int& heightToSet);
	bool setImageResolution(unsigned int& widthToSet, unsigned int& heightToSet, unsigned int offsetX, unsigned int offsetY);
	bool setGammaEnabled(bool isEnabled = false);
	bool setBufferedGrab(int buffers = 10);
	bool setFrameCounterEnabled(bool isEnabled = true);


	Camera m_pCam;		// camera handle
	PGRGuid m_cameraGUID; // camera GUID
	Image m_rawImageBuffer; // shared raw image buffer for temporary storage

	//	Constants used by PointGrey to specify registers and
	//	needed values on their camera
	const unsigned int c_cameraPower;
	const unsigned int c_cameraPowerValue;
	bool m_acquisitionStarted;
	bool m_isCameraStarted;
	unsigned long m_previousFrameNumber;
	int m_imageWidth, m_imageHeight, m_imageSize;
};

