#ifndef CAMERADRIVER_H
#define CAMERADRIVER_H

#include <opencv2/opencv.hpp>
#include "ImageServer.h"
#include "Types.h"

/*
CameraDriver:
*/

class CameraDriver : public ImageServer {
  public:
    // constructor / desctructor
    CameraDriver(Debuglevel cameraDriverDebugLevel);
    CameraDriver();
    // other
    void run(); // runs the camera driver.
    void calibrate(); // runs a calibration based on images on the file system. Images must contain 6 x 9 chessboards like in the example provided.
    void writeCameraCalibrationDataToFile(); // writes the intrinsic matrix and the distortion coefficient to a file "intrinsic.xml"
    void readCameraCalibrationDataFromFile(); // reads the intrinsic matrix and the distortion coefficient from a file "intrinsic.xml" and sets ready to true (if the file access has been successful)
  private:
    void undistortImage(cv::Mat& source, cv::Mat& destination); // undistorts the raw image using the intrinsix matrix and the distortion coefficients.
    void manageStateSwitches(); // this method manages switchs in States =  {initializing, running, freezed, terminated} and calls the appropriate methdos
    void runInRunningState(); // this method is called when State = running
    void runInInitializingState(); // this method is called when State = initializing
    void runInFreezedState(); // this method is called when State = freezed
    void runInTerminatedState(); // this method is called when State = terminated
    cv::Mat intrinsicMatrix; // intrinisic matrix of the camera, please see openCV-Documentation for details
    cv::Mat distortionCoefficients; // distortion coefficients of the camera, please see openCV-Documentation for details
    cv::Size imageSize; // size of the images that the driver provides
    const unsigned int recordQueueBufferSize = 3; // size of the buffer of records that the camera driver shall maintain
};

#endif