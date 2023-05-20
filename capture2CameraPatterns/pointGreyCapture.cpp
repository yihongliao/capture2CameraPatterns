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

//#include "StdAfx.h"
#include "pointGreyCapture.h"
#include <iostream>
using namespace std;

pointGreyCapture::pointGreyCapture() :
	c_cameraPower(0x610), c_cameraPowerValue(0x80000000)
{
	m_acquisitionStarted = false;
	m_isCameraStarted = false;
}

pointGreyCapture::~pointGreyCapture()
{
	
}

// turn on the camera
bool pointGreyCapture::openCamera(unsigned int cameraSerialNumber)
{
	BusManager	m_busManager; // Bus manager for all cameras
	unsigned int numCameras;
	if (!_checkLogError(m_busManager.GetNumOfCameras(&numCameras)))
	{
		return false;
	}
	if (numCameras)
	{
		if (cameraSerialNumber == 0)
		{
			if (!_checkLogError(m_busManager.GetCameraFromIndex(0, &m_cameraGUID)))
			{
				return false;
			}
		}
		else
		{
			if (!_checkLogError(m_busManager.GetCameraFromSerialNumber(cameraSerialNumber, &m_cameraGUID)))
			{
				return false;
			}
		}

	}
	else
	{
		cout << "no camera detected" << endl;
		return false;
	}

	if (!_checkLogError(m_pCam.Connect(&m_cameraGUID)))
	{
		return false;
	}

	// Power on the camera
	if (!_checkLogError(m_pCam.WriteRegister(c_cameraPower, c_cameraPowerValue)))
	{
		return false;
	}

	//	Wait for the camera to power up
	unsigned int currentPowerValue = 0;
	do
	{
		if (!_checkLogError(m_pCam.ReadRegister(c_cameraPower, &currentPowerValue)))
		{
			return false;
		}

	} while ((currentPowerValue & c_cameraPowerValue) == 0);

	return true;
}

// turn off the camera
bool pointGreyCapture::closeCamera()
{
	if (!_checkLogError(m_pCam.StopCapture()))
	{
		return false;
	} //	Failed to stop the camera

	if (!_checkLogError(m_pCam.Disconnect()))
	{
		return false;
	} // Failed to disconnect from the camera

	return true;
}

// Exposure time is in ms 
bool pointGreyCapture::initCamera(unsigned int imageWidth, unsigned int imageHeight, unsigned int offsetX, unsigned int offsetY, float frameRate, float exposureTime_ms, bool isHardwareTrigger)
{
	// Set the frame counter on so that we can check what frame we are on
	EmbeddedImageInfo imageInfo;
	imageInfo.frameCounter.onOff = true;
	if (!_checkLogError(m_pCam.SetEmbeddedImageInfo(&imageInfo)))
	{
		return false;
	}

	// setup gain to be always 0.0 to minimize noise impact
	setGainValue(0.0);

	// disable gamma
	setGammaEnabled(false);

	// setup hardware trigger 
	setHardwareTrigger(isHardwareTrigger);

	// set desired image resolution
	setImageResolution(imageWidth, imageHeight, offsetX, offsetY);

	// set frame rate
	setFrameRate(frameRate);

	// set number of 
	setBufferedGrab(60);

	// set up expTime, this should be called after setFrameRate()
	setExposureTime(exposureTime_ms);

	// start frame counter
	setFrameCounterEnabled(true);

	setWhiteBalance(650, 600);

	return true;
}
// enable hardware trigger
bool pointGreyCapture::setHardwareTrigger(bool isHardwareTrigger)
{
	//	Setup external trigger
	TriggerMode triggerMode;

	if (!_checkLogError(m_pCam.GetTriggerMode(&triggerMode)))
	{
		return false;
	}

	triggerMode.onOff = isHardwareTrigger;
	triggerMode.mode = 14;// 15; // Mode 14 overlapped readout trigger; Mode 0 is standard trigger; mode 15v is for multi shot
	//triggerMode.parameter = 7;
	triggerMode.source = 0;

	if (!_checkLogError(m_pCam.SetTriggerMode(&triggerMode)))
	{
		return false;
	}

	int triggerValue = isHardwareTrigger ? 1 : 0;
	const unsigned int k_softwareTrigger = 0x62C;
	unsigned int regVal = 0;
	Error error;
	do
	{
		if (!_checkLogError(m_pCam.ReadRegister(k_softwareTrigger, &regVal)))
		{
			error.PrintErrorTrace();
			//return false;
		}

	} while ((regVal >> 31) == triggerValue);

	cout << "trigger updated to: " << to_string(isHardwareTrigger) << "..." << endl;
	return true;
}
// set exposure time in ms
bool pointGreyCapture::setExposureTime(float& exposureTimeToSet)
{
	Property exposureProperty;
	exposureProperty.type = SHUTTER;
	
	if (!_checkLogError(m_pCam.GetProperty(&exposureProperty)))
	{
		cout << "camera exposure property cannot be retrieved" << endl;
		return false;
	}

	exposureProperty.onePush = false;
	exposureProperty.onOff = true;
	exposureProperty.autoManualMode = false;
	exposureProperty.absControl = true;
	exposureProperty.absValue = exposureTimeToSet;

	if (!_checkLogError(m_pCam.SetProperty(&exposureProperty)))
	{
		cout << "camera exposure time cannot be updated" << endl;
		return false;
	}
	cout << "exposure time set to " << exposureTimeToSet << endl;
	return true;
}

// set image resolution 
bool pointGreyCapture::setImageResolution(unsigned int &widthToSet, unsigned int& heightToSet)
{
	// Get camera's full nature resolution
	CameraInfo pCameraInfo;
	if (!_checkLogError(m_pCam.GetCameraInfo(&pCameraInfo)))
	{
		cout << "camera information cannot be retrieved" << endl;
		return false;
	}

	//m_pCam.GetCameraInfo(&pCameraInfo);
	int fullresX = 1920;
	int fullresY = 1200;
	char* imageFullRes = pCameraInfo.sensorResolution;
	sscanf_s(imageFullRes, "%dx%d", &fullresX, &fullresY);

	// center the image for capture
	int offsetX = (int)((fullresX - widthToSet) / 4) * 2;
	int offsetY = (int)((fullresY - heightToSet) / 4) * 2;

	Format7ImageSettings cameraImageSettings;
	cameraImageSettings.mode = MODE_0;
	cameraImageSettings.width = widthToSet;
	cameraImageSettings.height = heightToSet;
	cameraImageSettings.offsetX = offsetX;
	cameraImageSettings.offsetY = offsetY;
	cameraImageSettings.pixelFormat = PIXEL_FORMAT_RAW8;

	bool valid;
	Format7PacketInfo packetInfo;

	if (!_checkLogError(m_pCam.ValidateFormat7Settings(&cameraImageSettings, &valid, &packetInfo)))
	{
		cout << "cannot update camera resolution" << endl;
		return false;
	}

	if (!valid)
	{
		cout << "cannot update camera resolution" << endl;
		return false;
	}

	// Have to stop the capture before we can set this 
	// configuration since we are changing the image size
	stopAcquisition();

	if (!_checkLogError(m_pCam.SetFormat7Configuration(
		&cameraImageSettings, packetInfo.recommendedBytesPerPacket)))
	{
		cout << "cannot update camera resolution" << endl;
		return false;
	}

	unsigned int packetSize = 0;
	float packetPercentage = 0;

	if (!_checkLogError(m_pCam.GetFormat7Configuration(&cameraImageSettings, &packetSize, &packetPercentage)))
	{
		cout << "cannot update camera resolution" << endl;
		return false;
	}

	m_imageWidth = cameraImageSettings.width;
	m_imageHeight = cameraImageSettings.height;
	m_imageSize = m_imageWidth * m_imageHeight;

	cout << "camera resolution set to (" << m_imageWidth << "," << m_imageHeight << ")" << endl;

	return true;
}

// set image resolution 
bool pointGreyCapture::setImageResolution(unsigned int& widthToSet, unsigned int& heightToSet, unsigned int offsetX, unsigned int offsetY)
{
	// Get camera's full nature resolution
	CameraInfo pCameraInfo;
	if (!_checkLogError(m_pCam.GetCameraInfo(&pCameraInfo)))
	{
		cout << "camera information cannot be retrieved" << endl;
		return false;
	}

	//m_pCam.GetCameraInfo(&pCameraInfo);
	int fullresX = 1920;
	int fullresY = 1200;
	char* imageFullRes = pCameraInfo.sensorResolution;
	sscanf_s(imageFullRes, "%dx%d", &fullresX, &fullresY);

	Format7ImageSettings cameraImageSettings;
	cameraImageSettings.mode = MODE_0;
	cameraImageSettings.width = widthToSet;
	cameraImageSettings.height = heightToSet;
	cameraImageSettings.offsetX = offsetX;
	cameraImageSettings.offsetY = offsetY;
	cameraImageSettings.pixelFormat = PIXEL_FORMAT_RAW8;

	bool valid;
	Format7PacketInfo packetInfo;

	if (!_checkLogError(m_pCam.ValidateFormat7Settings(&cameraImageSettings, &valid, &packetInfo)))
	{
		cout << "cannot update camera resolution" << endl;
		return false;
	}

	if (!valid)
	{
		cout << "cannot update camera resolution" << endl;
		return false;
	}

	// Have to stop the capture before we can set this 
	// configuration since we are changing the image size
	stopAcquisition();

	if (!_checkLogError(m_pCam.SetFormat7Configuration(
		&cameraImageSettings, packetInfo.recommendedBytesPerPacket)))
	{
		cout << "cannot update camera resolution" << endl;
		return false;
	}

	unsigned int packetSize = 0;
	float packetPercentage = 0;

	if (!_checkLogError(m_pCam.GetFormat7Configuration(&cameraImageSettings, &packetSize, &packetPercentage)))
	{
		cout << "cannot update camera resolution" << endl;
		return false;
	}

	m_imageWidth = cameraImageSettings.width;
	m_imageHeight = cameraImageSettings.height;
	m_imageSize = m_imageWidth * m_imageHeight;

	cout << "camera resolution set to (" << m_imageWidth << "," << m_imageHeight << ")" << endl;

	return true;
}

// set frame rate
bool pointGreyCapture::setFrameRate(float frameRateToSet)
{
	Property frameRateSetting;

	frameRateSetting.type = FlyCapture2::FRAME_RATE;
	frameRateSetting.onOff = true;
	frameRateSetting.absControl = true;
	frameRateSetting.absValue = frameRateToSet;

	if (!_checkLogError(m_pCam.SetProperty(&frameRateSetting)))
	{
		cout << "cannot configure frame rate" << endl;
		return false;
	}

	cout << "frame rate is set as " << frameRateToSet << endl;

	return true;
}

// set gain value
bool pointGreyCapture::setGainValue(float gainvalueToSet)
{
	FlyCapture2::Property gainProperty;
	gainProperty.type = FlyCapture2::GAIN;

	if (!_checkLogError(m_pCam.GetProperty(&gainProperty)))
	{
		cout << "camera gain property cannot be retrieved" << endl;
		return false;
	}
	gainProperty.absControl = true;
	gainProperty.onePush = false;
	gainProperty.onOff = true;
	gainProperty.autoManualMode = false;
	gainProperty.absValue = gainvalueToSet;

	if (!_checkLogError(m_pCam.SetProperty(&gainProperty)))
	{
		cout << "cannot configure gain" << endl;
		return false;
	}

	cout << "gain is set as " << gainvalueToSet << endl;

	return true;
}
// disable white blance
bool pointGreyCapture::setGammaEnabled(bool isEnabled)
{
	Property gammaProperty;
	gammaProperty.type = FlyCapture2::GAMMA;

	if (!_checkLogError(m_pCam.GetProperty(&gammaProperty)))
	{
		cout << "camera gamma property cannot be retrieved" << endl;
		return false;
	}

	gammaProperty.onOff = false;

	if (!_checkLogError(m_pCam.SetProperty(&gammaProperty)))
	{
		cout << "cannot configure gamma" << endl;
		return false;
	}
	cout << "gamma is disabled" << endl;

	return true;
}

// set white blance
bool pointGreyCapture::setWhiteBalance(float gainRedToSet, float gainBlueToSet)
{
	Property whiteBalanceProperty;
	whiteBalanceProperty.type = FlyCapture2::WHITE_BALANCE;

	if (!_checkLogError(m_pCam.GetProperty(&whiteBalanceProperty)))
	{
		cout << "camera white balance property cannot be retrieved" << endl;
		return false;
	}

	whiteBalanceProperty.onOff = true;
	whiteBalanceProperty.autoManualMode = false;
	whiteBalanceProperty.valueA = (size_t)gainRedToSet;
	whiteBalanceProperty.valueB = (size_t)gainBlueToSet;

	if (!_checkLogError(m_pCam.SetProperty(&whiteBalanceProperty)))
	{
		cout << "cannot configure white blance" << endl;
		return false;
	}

	cout << "white balance is updated" << endl;

	return true;
}

// set buffered grab, if necessary
bool pointGreyCapture::setBufferedGrab(int buffers)
{
	// Setup buffer grab mode
	FC2Config config;

	if (!_checkLogError(m_pCam.GetConfiguration(&config)))
	{
		cout << "camera configuration cannot be retrieved" << endl;
		return false;
	}

	config.grabMode = BUFFER_FRAMES;
	config.grabTimeout = TIMEOUT_INFINITE;
	config.numBuffers = buffers;

	if (!_checkLogError(m_pCam.SetConfiguration(&config)))
	{
		cout << "camera buffer cannot be upated" << endl;
		return false;
	}

	return true;
}

// enable frame counter
bool pointGreyCapture::setFrameCounterEnabled(bool isEnabled)
{
	EmbeddedImageInfo EmbeddedInfo;
	if (!_checkLogError(m_pCam.GetEmbeddedImageInfo(&EmbeddedInfo)))
	{
		cout << "camera information cannot be retrieved" << endl;
		return false;
	}

	if (EmbeddedInfo.frameCounter.available == true) 
	{
		EmbeddedInfo.frameCounter.onOff = isEnabled;
	}
	else 
	{
		cout << "Frame counter is not available!" << endl;
		return false;
	}
	if (!_checkLogError(m_pCam.SetEmbeddedImageInfo(&EmbeddedInfo)))
	{
		cout << "camera frame counter cannot be changed" << endl;
		return false;
	}

	return true;
}

// check PRG error messages
bool pointGreyCapture::_checkLogError(Error error)
{
	if (error != PGRERROR_OK)
	{
		return false;
	} // Log our error and return false

	//  No error, return true
	return true;
}

// start capture
bool pointGreyCapture::startAcquisition()
{
	if (!_checkLogError(m_pCam.StartCapture()))
	{
		return false;
	}
	m_acquisitionStarted = true;
	return true;
}

// stop acqusition
bool pointGreyCapture::stopAcquisition()
{
	if (!_checkLogError(m_pCam.StopCapture()))
	{
		return false;
	}
	m_acquisitionStarted = false;
	return true;
}

// capture single image
// store the image in Flycapture2 image buffer
// Stream Mode: Pros: this is faster without starting and stopping 
//					image acquistion for each frame
//				Cons: This is unstable and affected by others (e.g., user interface messaging)
//				Stream Mode not recommended for applications with frame by frame graphical
//				user interactions
bool pointGreyCapture::captureSingleImage(Image &captureImage, bool isStreamMode)
{
	if (!isStreamMode) startAcquisition();
	if (!m_acquisitionStarted)
	{
		cout << "image acquisition has not started, check startAcqusition()" << endl;
		return false;
	}
	if (!_checkLogError(m_pCam.RetrieveBuffer(&m_rawImageBuffer)))
	{
		cout << "frame is not properly retrieved" << endl;
	}
	if (!_checkLogError(m_rawImageBuffer.Convert(PIXEL_FORMAT_MONO8, &captureImage)))
	{
		cout << "frame is not propery converted to mono8" << endl;
	}
	if (!isStreamMode) stopAcquisition();

	return true;
}

// capture a set of images with a given firstFrame counter 
// store the set of images in FlyCapture2 Image buffer
// Stream Mode: Pros: this is faster without starting and stopping 
//					image acquistion for each frame
//				Cons: This is unstable and affected by others (e.g., user interface messaging)
//				Stream Mode not recommended for applications with frame by frame graphical
//				user interactions
bool pointGreyCapture::captureImageSet(Image captureImage[], int numberOfFrames, 
	int firstFrameCounter, bool isStreamMode)
{
	if (!isStreamMode) startAcquisition();
	if (!m_acquisitionStarted)
	{
		cout << "image acquisition has not started, call startAcqusition() first" << endl;
		return false;
	}
	
	// skip frames that until the first frame
	long int currentFrameCounter = firstFrameCounter;
	while ((currentFrameCounter - firstFrameCounter) % numberOfFrames != 1)
	{
		if (!_checkLogError(m_pCam.RetrieveBuffer(&m_rawImageBuffer)))
		{
			cout << "frame is not properly retrieved" << endl;
		}
		currentFrameCounter = m_rawImageBuffer.GetMetadata().embeddedFrameCounter;
	}
	// copy first frame
	if (!_checkLogError(m_rawImageBuffer.Convert(PIXEL_FORMAT_MONO8, &captureImage[0])))
	{
		cout << "frame is not propery converted to mono8" << endl;
	}
	m_previousFrameNumber = currentFrameCounter;

	// grab the rest number of frames
	for (int k = 1; k < numberOfFrames; k++)
	{
		
		if (!_checkLogError(m_pCam.RetrieveBuffer(&m_rawImageBuffer)))
		{
			cout << "frame is not properly retrieved" << endl;
			return false;
		}
		currentFrameCounter = m_rawImageBuffer.GetMetadata().embeddedFrameCounter;
		if (currentFrameCounter - m_previousFrameNumber == 1)
		{
			////cout << "frame counter:" << m_rawImageBuffer.GetMetadata().embeddedFrameCounter << endl;
			if (!_checkLogError(m_rawImageBuffer.Convert(PIXEL_FORMAT_MONO8, &captureImage[k])))
			{
				cout << "frame is not propery converted to Mono8" << endl;
				return false;
			}
			m_previousFrameNumber = currentFrameCounter;
		}
		else
		{
			cout << "...frame skiped: " << currentFrameCounter - m_previousFrameNumber << endl;
			return false;
		}
		cout << "frame counter: " << currentFrameCounter << endl;
	
	}
	if (!isStreamMode) stopAcquisition();
	return true;
}

// capture single image
// store the image in an image data array
// Stream Mode: Pros: this is faster without starting and stopping 
//					image acquistion for each frame
//				Cons: This is unstable and affected by others (e.g., user interface messaging)
//				Stream Mode not recommended for applications with frame by frame graphical
//				user interactions
bool pointGreyCapture::captureSingleImageData(unsigned char *captureImage, bool isStreamMode)
{
	if (!isStreamMode) startAcquisition();
	if (!m_acquisitionStarted)
	{
		cout << "image acquisition has not started, check startAcqusition()" << endl;
		return false;
	}
	if (!_checkLogError(m_pCam.RetrieveBuffer(&m_rawImageBuffer)))
	{
		cout << "frame is not properly retrieved" << endl;
	}
	memcpy(captureImage, m_rawImageBuffer.GetData(), sizeof(captureImage[0]) * m_imageSize );
	if (!isStreamMode) stopAcquisition();
	return true;
}

// capture a set of images with a given firstFrame counter 
// store the set of images in image data arrays
// Stream Mode: Pros: this is faster without starting and stopping 
//					image acquistion for each frame
//				Cons: This is unstable and affected by others (e.g., user interface messaging)
//				Stream Mode not recommended for applications with frame by frame graphical
//				user interactions
bool pointGreyCapture::captureImageSetData(unsigned char *captureImage[], int numberOfFrames, 
	int firstFrameCounter, bool isStreamMode)
{
	if (!isStreamMode) startAcquisition();
	if (!m_acquisitionStarted)
	{
		cout << "image acquisition has not started, call startAcqusition() first" << endl;
		return false;
	}

	// skip frames that until the first frame
	long int currentFrameCounter = firstFrameCounter;
	while ((currentFrameCounter - firstFrameCounter) % numberOfFrames != 1)
	{
		if (!_checkLogError(m_pCam.RetrieveBuffer(&m_rawImageBuffer)))
		{
			cout << "frame is not properly retrieved" << endl;
		}
		currentFrameCounter = m_rawImageBuffer.GetMetadata().embeddedFrameCounter;
	}
	// copy first frame
	memcpy(captureImage[0], m_rawImageBuffer.GetData(), sizeof(captureImage[0][0]) * m_imageSize);
	m_previousFrameNumber = currentFrameCounter;

	// grab the rest number of frames
	for (int k = 1; k < numberOfFrames; k++)
	{

		if (!_checkLogError(m_pCam.RetrieveBuffer(&m_rawImageBuffer)))
		{
			cout << "frame is not properly retrieved" << endl;
			return false;
		}
		currentFrameCounter = m_rawImageBuffer.GetMetadata().embeddedFrameCounter;
		if (currentFrameCounter - m_previousFrameNumber == 1)
		{
			memcpy(captureImage[k], m_rawImageBuffer.GetData(), sizeof(captureImage[k][0]) * m_imageSize);
			m_previousFrameNumber = currentFrameCounter;
		}
		else
		{
			cout << "...frame skiped: " << currentFrameCounter - m_previousFrameNumber << endl;
			return false;
		}
		cout << "frame counter: " << currentFrameCounter << endl;

	}
	if (!isStreamMode) stopAcquisition();
	return true;
}
