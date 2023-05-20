#pragma once

#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

class CPngFileIO
{
public:
	CPngFileIO(void);
	~CPngFileIO(void);

	bool ReadPngFile(const char* fileName, unsigned char*& imageData, int& imageWidth, int& imageHeight, int& nChannels);
	bool WritePngFile(const char* fileName, unsigned char* imageData, int imageWidth, int imageHeight, int nChannels);
	bool WritePngFileFT(const char* fileName, float* imageData, int imageWidth, int imageHeight, unsigned char* mask = NULL);
	bool WritePngFilePhase(const char* fileName, float* imageData, int imageWidth, int imageHeight);
};

