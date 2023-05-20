//#include "stdafx.h"
#include "PngFileIO.h"
#include <iostream>



CPngFileIO::CPngFileIO(void)
{
}

CPngFileIO::~CPngFileIO(void)
{
}

//--------------------------------------------------------------------
// Read png image
//
// Input:
//		fileName	= name of files to store the data in ASCII format
//		imageData	= image data
//		imageWidth	= number of cols
//		imageHeight = number of rows
//		nChannels	= number of channels
//
// Return:		
//
// Source code was finalized by Professor Song Zhang at Purdue University
// Contact: szhang15@purdue.edu
// Date: 08 / 15 / 2015
//--------------------------------------------------------------------
bool CPngFileIO::ReadPngFile(const char* fileName, unsigned char*& imageData, int& imageWidth, int& imageHeight, int& nChannels)
{
	Mat img = imread(fileName);

	if (img.empty())
	{
		//		MessageBox(0, _T("Cannot read png file!"), _T("Error"), 0);
		cout << "cannot read file: " << fileName;
		return false;
	}

	imageWidth = img.cols;
	imageHeight = img.rows;
	nChannels = img.channels();

	int imageSize = imageWidth * imageHeight;
	if (imageData) delete[] imageData;
	imageData = new unsigned char[imageSize * nChannels];
	memcpy(imageData, img.data, sizeof(imageData[0]) * imageSize * nChannels);

	return true;
}
//--------------------------------------------------------------------
// Write png image
//
// Input:
//		fileName	= name of files to store the data in ASCII format
//		imageData	= image data
//		imageWidth	= number of cols
//		imageHeight = number of rows
//		nChannels	= number of channels
//
// Return:		
//
// Source code was finalized by Professor Song Zhang at Purdue University
// Contact: szhang15@purdue.edu
// Date: 08 / 15 / 2015
//--------------------------------------------------------------------
bool CPngFileIO::WritePngFile(const char* fileName, unsigned char* imageData, int imageWidth, int imageHeight, int nChannels)
{
	Mat img;
	if (nChannels == 1)
	{
		img = Mat(Size(imageWidth, imageHeight), CV_8UC1, imageData);
	}
	else
	{
		img = Mat(Size(imageWidth, imageHeight), CV_8UC3, imageData);
	}
	return imwrite(fileName, img);
}

//--------------------------------------------------------------------
// Write png floating image
// normalize floating data based on valid data points,
// scale the data to 0-255 and write it out
//
// Input:
//		fileName	= name of files to store the data in ASCII format
//		imageData	= image data
//		imageWidth	= number of cols
//		imageHeight = number of rows
//		mask		= indicator valid data points (none zero valid point)
//
// Return:		
//
// Source code was finalized by Professor Song Zhang at Purdue University
// Contact: szhang15@purdue.edu
// Date: 08 / 15 / 2015
//--------------------------------------------------------------------
bool CPngFileIO::WritePngFileFT(const char* fileName, float* imageData, int imageWidth, int imageHeight, unsigned char* mask)
{
	// find min and max
	float minz = FLT_MAX;
	float maxz = -FLT_MAX;
	int imageSize = imageWidth * imageHeight;
	unsigned char* saveData = new unsigned char[imageSize];
	memset(saveData, 0, sizeof(saveData[0]) * imageSize);
	if (mask)
	{
		for (int i = 0; i < imageSize; i++)
		{
			if (mask[i])
			{
				float t = imageData[i];
				minz = minz < t ? minz : t;
				maxz = maxz > t ? maxz : t;
			}
		}
		float scale = 1.0f / (maxz - minz);
		for (int i = 0; i < imageSize; i++)
		{
			if (mask[i])
			{
				saveData[i] = (int)((imageData[i] - minz) * 255.0f * scale);
			}
		}
	}
	else
	{
		for (int i = 0; i < imageSize; i++)
		{
			float t = imageData[i];
			minz = minz < t ? minz : t;
			maxz = maxz > t ? maxz : t;
		}
		float scale = 1.0f / (maxz - minz);
		for (int i = 0; i < imageSize; i++)
		{
			saveData[i] = (int)((imageData[i] - minz) * 255.0f * scale);
		}

	}

	Mat img = Mat(Size(imageWidth, imageHeight), CV_8UC1, saveData);
	bool rValue = imwrite(fileName, img);
	delete[] saveData;

	return rValue;
}


//--------------------------------------------------------------------
// Write png floating image
// normalize floating data based on valid data points,
// scale the data to 0-255 and write it out
//
// Input:
//		fileName	= name of files to store the data in ASCII format
//		imageData	= image data
//		imageWidth	= number of cols
//		imageHeight = number of rows
//		mask		= indicator valid data points (none zero valid point)
//
// Return:		
//
// Source code was finalized by Professor Song Zhang at Purdue University
// Contact: szhang15@purdue.edu
// Date: 08 / 15 / 2015
//--------------------------------------------------------------------
bool CPngFileIO::WritePngFilePhase(const char* fileName, float* imageData, int imageWidth, int imageHeight)
{
	// find min and max
	float minz = FLT_MAX;
	float maxz = -FLT_MAX;
	int imageSize = imageWidth * imageHeight;
	unsigned char* saveData = new unsigned char[imageSize];
	memset(saveData, 0, sizeof(saveData[0]) * imageSize);
	
	for (int i = 0; i < imageSize; i++)
	{
		saveData[i] = (unsigned char)(imageData[i] * 255.0f);
	}


	Mat img = Mat(Size(imageWidth, imageHeight), CV_8UC1, saveData);
	bool rValue = imwrite(fileName, img);
	delete[] saveData;

	return rValue;
}
