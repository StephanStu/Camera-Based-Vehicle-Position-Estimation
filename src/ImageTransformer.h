#ifndef IMAGETRANSFORMER_H
#define IMAGETRANSFORMER_H

#include <opencv2/opencv.hpp>
#include "CameraServer.h"
#include "RecordServer.h"
#include "Types.h"

/*
ImageTransformer:
This software component is responsible for consuming an undistorted image and computing the gray bird eye's view from that input.
This software component holds all meta-parameter, e.g. kernelsize to get the job done.
*/

class ImageTransformer : public RecordServer {
  public:
    // constructor / desctructor
    ImageTransformer(Debuglevel imageTransformerDebugLevel);
    ImageTransformer();
    // other
    void run(); // realization of virtual function defined in Runnable Entity
    void mountCameraServer(std::shared_ptr<CameraServer> pointerToCameraServer); // "mounts" the instance of CameraServer to the instance of ImageTransformer by saving the shared pointer in the member variables
    void convertToBinaryImage(cv::Mat& source, cv::Mat& destination); // convert the gray(!) source image to a binary image (= total black everything below a threshold and totla white anything above threshold)
    void convertToBirdEyesView(cv::Mat& source, cv::Mat& destination); // compute the bird eye's view of the source image
    void convertToGrayImage(cv::Mat& source, cv::Mat& destination); // convert source image to a gray image
    void detectEdges(cv::Mat& source, cv::Mat& destination); // detect edges in source image 
    void applyGausianBlurr(cv::Mat& source, cv::Mat& destination); // apply a Gaussian Noise kernel to the source image
    void setBirdEyesTransformMatrix(); // compute the bird eye's transformation Matrix mapping the original image into the 'bird eye's view'.
  private:
    friend class PositionServer;
    friend class PositionEstimator;
    void manageStateSwitches(); // this method manages switchs in States =  {initializing, running, freezed, terminated} and calls the appropriate methdos
    void runInRunningState(); // this method is called when State = running; it consumes records provided by the CameraServer, transforms these and adds the results to the record-member variable where they can be consumed.
    void runInInitializingState(); // this method is called when State = initializing; it does everything needed to make the ImageTransformer ready to operate in the running state/checks if the ImageTransformer is ready to operate in the running state
    void runInFreezedState(); // this method is called when State = freezed; it prevents data from being changed while being freezed
    void runInTerminatedState(); // this method is called when State = terminated
    void applyImageTransformations(PositionServiceRecord& record); // applies the image transformations in the correct order: convert to gray, convert to binary,...
    const int queueBufferSize = 2; // default = 2, determines how much records are kept in the queue before they are popped without processing the images 
    const int kernelsize = 5; // default = 5, determines degree of blurring in gausian blur method
    const int birdsEyeImageWidth = 500; // default = 500 (taken from GitHub), width of bird's eye image (after running the transform)
    const int birdsEyeImageHeight = 600; // default = 600 (taken from GitHub), height of bird's eye image (after running the transform)
    const int objectPoint1XValue = 375; // value of x coordinate in mapping from object-points -> image-points in bird's eye transform
    const int objectPoint1YValue = 480; // value of y coordinate in mapping from object-points -> image-points in bird's eye transform
    const int objectPoint2XValue = 905; // value of x coordinate in mapping from object-points -> image-points in bird's eye transform
    const int objectPoint2YValue = 480; // value of y coordinate in mapping from object-points -> image-points in bird's eye transform
    const int objectPoint3XValue = 1811; // value of x coordinate in mapping from object-points -> image-points in bird's eye transform
    const int objectPoint3YValue = 685; // value of y coordinate in mapping from object-points -> image-points in bird's eye transform
    const int objectPoint4XValue = -531; // value of x coordinate in mapping from object-points -> image-points in bird's eye transform
    const int objectPoint4YValue = 685; // value of y coordinate in mapping from object-points -> image-points in bird's eye transform
    const int maxBinaryValue = 255; // value of the visible pixels in the binary image == "white"
    const int binaryThresholdValue = 90; // value of the grayscale(!) pixels that are converted to maxBinaryValue in the bird eye's image; anything below that value is converted to zero = "black". 
    cv::Mat birdsEyeTransformMatrix; // Matrix for bird's eye perspective transform  
    std::shared_ptr<CameraServer> accessCameraServer; // shared pointer to an instance of CameraDriver delivering images upond request
    bool newRecordIsAvailable(); // verifies, that the server has a new record (before requesting to send it)
    bool cameraServerIsMounted; // this is true once an instance of CameraDriver has been mounted
    bool birdsEyeTransformationMatrixIsReady; // this is true once the bird eye's transformation matrix has been calculated
};

#endif