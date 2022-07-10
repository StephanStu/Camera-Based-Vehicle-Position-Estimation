#ifndef CAMERADRIVER_H
#define CAMERADRIVER_H

#include <opencv2/opencv.hpp>
#include <future>
#include <mutex>
#include "RunnableEntity.h"
#include "MovableTimestampedType.h"
#include "Types.h"

/*
CameraDriver:
This software component is the beginning of a service; it provides raw & undistorted images as records of type PositionServiceRecord.
The latest record can be accessed with the public interfaces, both "receiving via promise-future" (as part of the position servive) as well as "getting-directly" is possible (e.g. for testing).
The raw & undistorted image are frequently updated by pulling raw images from a source of cv::Mat's.
*/

class CameraDriver : public RunnableEntity {
  public:
    // constructor / desctructor
    CameraDriver(Debuglevel cameraDriverDebugLevel);
    CameraDriver();
    // other
    void run(); // realization of virtual function defined in Runnable Entity
    void calibrate(); // runs a calibration based on images on the file system. Images must contain 6 x 9 chessboards like in the example provided.
    void writeCameraCalibrationDataToFile(); // writes the intrinsic matrix and the distortion coefficient to a file "intrinsic.xml"
    void readCameraCalibrationDataFromFile(); // reads the intrinsic matrix and the distortion coefficient from a file "intrinsic.xml" and sets ready to true (if the file access has been successful)
    void receiveRecord(std::promise<MovableTimestampedType<PositionServiceRecord>> &&recordPromise); // this method allows consumers to receive the latest record with a promise-future-instance; the raw and undistorted images are filled with the most current data
    MovableTimestampedType<PositionServiceRecord> getRecord(); // this method allows consumers to get the latest record; the raw and undistorted images are filled with the most current data.
    bool recordIsCurrent(); // this method returns the state of the record = {raw image, undistorted image and age} that may be consumed
  private:
    friend class PositionService;
    void undistortImage(cv::Mat& source, cv::Mat& destination); // undistorts the raw image using the intrinsix matrix and the distortion coefficients.
    void manageStateSwitches(); // this method manages switchs in States =  {initializing, running, freezed, terminated} and calls the appropriate methdos
    void runInRunningState(); // this method is called when State = running; it sources a raw image and updates the newRecord with the raw & undistorted image frequently and in a thread-save manner
    void runInInitializingState(); // this method is called when State = initializing; it does everything needed to get the instance ready for the running state / checks if the instance is ready for the running state
    void runInFreezedState(); // this method is called when State = freezed; it stops any action on the data
    void runInTerminatedState(); // this method is called when State = terminated; it releases resources when necessary
    void setNewRecord(cv::Mat &&rawImage); // this method allows the driver itself sets the newRecord frequently and thread-safe 
    cv::Mat intrinsicMatrix; // intrinisic matrix of the camera, please see openCV-Documentation for details
    cv::Mat distortionCoefficients; // distortion coefficients of the camera, please see openCV-Documentation for details
    cv::Size imageSize; // size of the images that the driver provides
    MovableTimestampedType<PositionServiceRecord> newRecord; // this is the latest raw + undistorted image wrapped in a record that the camera driver offers to it's consumers
    bool newRecordIsCurrent; // this must be true, when the newRecord is updated and this must become false if a consumer pulled the record.
    std::mutex newRecordProtection; // this mutex is protecting access to the newUndistortedImage
    std::condition_variable condition; // condition varibale is needed to notify clients that newUndistortedImage can be accessed again
    const unsigned int recordQueueBufferSize = 3; // size of the buffer of records that the camera driver shall maintain
};

#endif