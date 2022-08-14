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

ImageSource:
Provides Images from i) a video file or ii) one static image (in a repetitive manner) to the CameraServer.
Note: This entity must be mounted to CamerDriver outside of Position Server, because the source of images is provided outside of PositioServer.

VelocitySource:
Provides a simualted velocity measurment to the CameraServer by adding some artifical noise to a mean velocity provided by caller.
Note: This entity must be mounted to CamerDriver outside of Position Server, because mean & variance of velocity are provided outside of PositioServer.
*/

class VelocitySource{
  public:
    VelocitySource(const int meanVelocity, const int varianceVelocity); // constructor
    void getNextVelocityMeasurement(float& velocityMeasurement); // provides a random velocity measuement (in m/s)
  private:
    int mean; // velocity mean in km/h
    int variance; // velocity variance in km/h
};

class ImageSource{
  public:
    ImageSource(const std::string nameOfFile);
    void getNextImage(cv::Mat& image);
  private:
    void setFileType(const std::string fileName);
    Filetype filetype;
    std::string filename;  
};

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
    void mountImageSource(std::shared_ptr<ImageSource> pointerToImageSource); // "mounts" the instance of ImageSource to the instance of CameraDriver by saving the shared pointer in the member variables
    void mountVelocitySource(std::shared_ptr<VelocitySource> pointerToVelocitySource); // "mounts" the instance of ImageSource to the instance of CameraDriver by saving the shared pointer in the member variables
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
    bool velocitySourceIsMounted; // this is true once an :VelocitySource is mounted
    bool imageSourceIsMounted; // this is true once an :ImageSource is mounted
    std::shared_ptr<VelocitySource> accessVelocitySource; // source for the velocity signal to be merged with an image in the camera server
    std::shared_ptr<ImageSource> accessImageSource; // source for the image to be merged with a velocity signal in the camera server
};

#endif