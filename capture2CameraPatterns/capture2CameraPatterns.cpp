// CaptureDICpatterns.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <thread>
#include <Windows.h>
#include <direct.h>

#include "opencv2/opencv.hpp"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "PngFileIO.h"
#include "pointGreyCapture.h"

std::mutex mtx;

Ptr<cv::SimpleBlobDetector> markerDetector()
{
    SimpleBlobDetector::Params detectorParams;
    detectorParams.minThreshold = 20.0f;
    detectorParams.maxThreshold = 240.0f;
    detectorParams.thresholdStep = 5.0f;
    detectorParams.minDistBetweenBlobs = 5.0; // size of featrue points

    detectorParams.filterByCircularity = true;
    detectorParams.minCircularity = .4f;
    detectorParams.maxCircularity = 1.0;

    detectorParams.filterByConvexity = true;
    detectorParams.minConvexity = .4f;
    detectorParams.maxConvexity = 1.0;

    detectorParams.filterByInertia = false;

    detectorParams.filterByArea = true;
    detectorParams.minArea = 400.0;		// size of feature points
    detectorParams.maxArea = 10000.0;

    detectorParams.filterByColor = true;
    detectorParams.blobColor = 255;		// flipped the color to 255 for white circles

    Ptr<SimpleBlobDetector> blob_detector = SimpleBlobDetector::create(detectorParams);
    return blob_detector;
}

class CGrabImages
{
public:
    CGrabImages();
    ~CGrabImages();

public:
    int m_cameraWidth = 1920;
    int m_cameraHeight = 1200;
    int m_offsetX = 0;// 640;
    int m_offsetY = 0;// 360;
    int m_projectorWidth = 912;
    int m_projectorHeight = 1140;

    int m_cameraSize = m_cameraWidth * m_cameraHeight;

    float frameRate = 15.0;
    float expTime = 3.0f;

    bool stopCapture = false;

public:
    bool createSubDirectory(string folderDir);
    void grabImage(unsigned int cameraSerialNo, string folderDir, int totalPosNo);
    void grabImageSet(unsigned int cameraSerialNo, string folderDir, int totalPosNo);
    void rectSequence(vector<Mat>rawFringeMat, vector<Mat>& outputFringeMat);
    void savePosFringe(string rootPath, vector<Mat>setFringeMat);
    void runMultiThread(unsigned int cameraSerialNo1, unsigned int cameraSerialNo2, string folderDir, int totalPosNo);
    void runSetMultiThread(unsigned int cameraSerialNo1, unsigned int cameraSerialNo2, string folderDir, int totalPosNo);
};


CGrabImages::CGrabImages(void)
{
}

CGrabImages::~CGrabImages(void)
{
}

bool CGrabImages::createSubDirectory(string folderDir)
{
    if (_mkdir(folderDir.c_str()))
    {
        //string  tmpString = "del " + folderDir + "\\\\f*.png";
        //system(tmpString.c_str());
        return true;
    }
    else
    {
        return false;
    }
}

void CGrabImages::rectSequence(vector<Mat>rawFringeMat, vector<Mat>& outputFringeMat)
{
    double minDiffValue = 100000000;
    int maxID = 0;
    vector<Scalar>meanValues;
    meanValues.clear();

    // maximum two bright fringes
    for (int k = 0; k < rawFringeMat.size(); k++)
    {
        Scalar mValue = mean(rawFringeMat[k]);
        meanValues.push_back(mValue);
    }
    double maxValue = -100000000;
    for (int k = 0; k < rawFringeMat.size(); k++)
    {
        double sumValue = meanValues[k](0) + meanValues[(k + 1) % rawFringeMat.size()](0);
        if (maxValue < sumValue)
        {
            maxValue = sumValue;
            maxID = k;
        }
    }

    for (int k = 0; k < rawFringeMat.size() - 2; k++)
    {
        outputFringeMat.push_back(rawFringeMat[(k + maxID + 2) % rawFringeMat.size()].clone());
    }
}


void CGrabImages::savePosFringe(string rootPath, vector<Mat>setFringeMat)
{
    createSubDirectory(rootPath);
    for (int k = 0; k < setFringeMat.size(); k++)
    {
        string fileName = rootPath + "/f" + to_string(k) + ".png";
        imwrite(fileName, setFringeMat[k]);
    }
}

void CGrabImages::runMultiThread(unsigned int cameraSerialNo1, unsigned int cameraSerialNo2, string folderDir, int totalPosNo)
{
    std::thread t1(&CGrabImages::grabImage, this, cameraSerialNo1, folderDir, totalPosNo);
    std::thread t2(&CGrabImages::grabImage, this, cameraSerialNo2, folderDir, totalPosNo);
    t1.join();
    t2.join();
}

void CGrabImages::runSetMultiThread(unsigned int cameraSerialNo1, unsigned int cameraSerialNo2, string folderDir, int totalPosNo)
{
    std::thread t1(&CGrabImages::grabImageSet, this, cameraSerialNo1, folderDir, totalPosNo);
    std::thread t2(&CGrabImages::grabImageSet, this, cameraSerialNo2, folderDir, totalPosNo);
    t1.join();
    t2.join();
}

void CGrabImages::grabImageSet(unsigned int cameraSerialNo, string folderDir, int totalPosNo)
{
    pointGreyCapture m_grab;

    // turn on the camera based on camera serial number
    m_grab.openCamera(cameraSerialNo);

    // initialize camera
    bool isHardwareTrigger = true;
    m_grab.initCamera(m_cameraWidth, m_cameraHeight, m_offsetX, m_offsetY, frameRate, expTime, isHardwareTrigger);
    m_grab.setExposureTime(expTime);
    m_grab.startAcquisition();

    const int setImageNo = 64;
    unsigned char* textureImage = new unsigned char[m_cameraSize];
    vector<Mat> rawSetFringeMat, setFringeMat;
    unsigned char* setFringeData[setImageNo];
    for (int k = 0; k < setImageNo; k++)
    {
        setFringeData[k] = new unsigned char[m_cameraSize];
    }

    Mat image;
    for (int posNo = 0; posNo < totalPosNo; posNo++) {
        while (!stopCapture)
        {
            m_grab.captureSingleImageData(textureImage, true);
            image = Mat(Size(m_cameraWidth, m_cameraHeight), CV_8UC1, textureImage);

            //vector<Point2f> cameraPoints;
            //const Size featureDimensions(12, 19);
            //cameraPoints.clear();

            //Mat image_w_points;
            //image.copyTo(image_w_points);
            //bool found = findCirclesGrid(image_w_points, featureDimensions, cameraPoints, CALIB_CB_CLUSTERING | CALIB_CB_SYMMETRIC_GRID, markerDetector());
            //drawChessboardCorners(image_w_points, featureDimensions, Mat(cameraPoints), found);
            
            //Mat imageRGB;
            //imageRGB = image;
            //demosaicing(image, imageRGB, COLOR_BayerBG2BGR);
            namedWindow(to_string(cameraSerialNo), WINDOW_NORMAL);
            resizeWindow(to_string(cameraSerialNo), m_cameraWidth / 2, m_cameraHeight / 2);
            imshow(to_string(cameraSerialNo), image);

            int c = waitKey(1);
            if (c == 27)
            {
                stopCapture = true;
            }
        }

        destroyWindow(to_string(cameraSerialNo));
        m_grab.stopAcquisition();

        //mtx.lock();

        // capture fringe set
        cout << "Capture & save camera " << cameraSerialNo << " set " << posNo << endl;
        m_grab.setExposureTime(expTime);
        m_grab.startAcquisition();
        m_grab.captureImageSetData(setFringeData, setImageNo, 0, true);
        rawSetFringeMat.clear();
        setFringeMat.clear();
        for (int k = 0; k < setImageNo; k++)
        {
            Mat fringeMat = Mat(Size(m_cameraWidth, m_cameraHeight), CV_8UC1, setFringeData[k]);
            rawSetFringeMat.push_back(fringeMat.clone());
        }
        rectSequence(rawSetFringeMat, setFringeMat);
        savePosFringe(folderDir + to_string(cameraSerialNo) + "/posEval" + to_string(posNo+2), setFringeMat);

        stopCapture = false;
        //mtx.unlock();
    
    }
    
    // turn off the camera
    m_grab.stopAcquisition();
    m_grab.closeCamera();

    for (int k = 0; k < setImageNo; k++)
    {
        delete[] setFringeData[k];
    }
    delete[] textureImage;
}

void CGrabImages::grabImage(unsigned int cameraSerialNo, string folderDir, int totalPosNo)
{
    pointGreyCapture m_grab;

    // turn on the camera based on camera serial number
    m_grab.openCamera(cameraSerialNo);

    // initialize camera
    bool isHardwareTrigger = true;
    m_grab.initCamera(m_cameraWidth, m_cameraHeight, m_offsetX, m_offsetY, frameRate, expTime, isHardwareTrigger);
    m_grab.setExposureTime(expTime); 
    m_grab.startAcquisition();

    unsigned char* textureImage = new unsigned char[m_cameraSize];

    Mat image;
    for (int posNo = 0; posNo < totalPosNo; posNo++) {
        while (!stopCapture)
        {
            m_grab.captureSingleImageData(textureImage, true);
            image = Mat(Size(m_cameraWidth, m_cameraHeight), CV_8UC1, textureImage);

            //vector<Point2f> cameraPoints;
            //const Size featureDimensions(12, 19);
            //cameraPoints.clear();

            //Mat image_w_points;
            //image.copyTo(image_w_points);
            //bool found = findCirclesGrid(image_w_points, featureDimensions, cameraPoints, CALIB_CB_CLUSTERING | CALIB_CB_SYMMETRIC_GRID, markerDetector());
            //drawChessboardCorners(image_w_points, featureDimensions, Mat(cameraPoints), found);

            //Mat imageRGB;
            //imageRGB = image;
            //demosaicing(image, imageRGB, COLOR_BayerBG2BGR);
            namedWindow(to_string(cameraSerialNo), WINDOW_NORMAL);
            resizeWindow(to_string(cameraSerialNo), m_cameraWidth / 2, m_cameraHeight / 2);
            imshow(to_string(cameraSerialNo), image);

            int c = waitKey(1);
            if (c == 27)
            {
                stopCapture = true;
            }
        }

        destroyWindow(to_string(cameraSerialNo));

        // capture single image
        string fileName = folderDir + to_string(cameraSerialNo) + "/" + to_string(posNo) + ".png";
        imwrite(fileName, image);
        stopCapture = false;
    }


    // turn off the camera
    m_grab.stopAcquisition();
    m_grab.closeCamera();

    delete[] textureImage;
}

int main()
{
    unsigned int cameraSerialNo1 = 17081637;
    unsigned int cameraSerialNo2 = 17081624;
    string folderDir = "C:/Users/yhosc/Desktop/deer_images2/";
    CGrabImages grabImages;
    //grabImages.grabImage(cameraSerialNo2, folderDir);
    //grabImages.runMultiThread(cameraSerialNo1, cameraSerialNo2, folderDir, 3);
    grabImages.runSetMultiThread(cameraSerialNo1, cameraSerialNo2, folderDir, 2);
    return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
