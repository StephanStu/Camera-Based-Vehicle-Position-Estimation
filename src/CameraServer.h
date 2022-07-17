#ifndef CAMERASERVER_H
#define CAMERASERVER_H

#include <opencv2/opencv.hpp>
#include <future>
#include <mutex>
#include "RecordServer.h"
#include "MovableTimestampedType.h"
#include "Types.h"

/*
CameraServer:
This software component provides raw & undistorted images as a record of type PositionServiceRecord.
The raw & undistorted image in the record are frequently updated by pulling raw images from a source of cv::Mat's.
*/

class CameraServer : public RecordServer {
  public:
    // constructor / desctructor
    CameraServer(Debuglevel cameraServerDebugLevel);
    CameraServer();
    // other
    void run(); // realization of virtual function defined in Runnable Entity
    void calibrate(); // runs a calibration based on images on the file system. Images must contain 6 x 9 chessboards like in the example provided.
    void writeCameraCalibrationDataToFile(); // writes the intrinsic matrix and the distortion coefficient to a file "intrinsic.xml"
    void readCameraCalibrationDataFromFile(); // reads the intrinsic matrix and the distortion coefficient from a file "intrinsic.xml" and sets ready to true (if the file access has been successful)
    void undistortImage(cv::Mat& source, cv::Mat& destination); // undistorts the raw image using the intrinsix matrix and the distortion coefficients.
    void getImageSize(cv::Size& size); // returns the size of the image the CameraServer seves to consumers if it's services
    void createBlackRecord(MovableTimestampedType<PositionServiceRecord>& record); // fills the provided record with a black raw image & a black undistorted image
  private:
    friend class PositionServer;
    friend class ImageTransformer;
    void manageStateSwitches(); // this method manages switchs in States =  {initializing, running, freezed, terminated} and calls the appropriate methdos
    void runInRunningState(); // this method is called when State = running; it sources a raw image and updates the record with the raw & undistorted image frequently and in a thread-save manner
    void runInInitializingState(); // this method is called when State = initializing; it checks if the instance is ready for the running state
    void runInFreezedState(); // this method is called when State = freezed; it stops any action on the data
    void runInTerminatedState(); // this method is called when State = terminated; after fillign the record with balck images, it blocks any further action on the data of the instance of CameraServer
    cv::Mat intrinsicMatrix; // intrinisic matrix of the camera, please see openCV-Documentation for details
    cv::Mat distortionCoefficients; // distortion coefficients of the camera, please see openCV-Documentation for details
    cv::Size imageSize; // size of the images that the driver provides
    bool calibrationDataIsLoaded; // this true once calibration data (intriniscMatrix, distortionCoefficient) is loaded (from file or by running CameraServer::calibrate)
};

#endif