#ifndef IMAGETRANSFORMER_H
#define IMAGETRANSFORMER_H

#include <opencv2/opencv.hpp>
#include "CameraDriver.h"
#include "ImageServer.h"
#include "Types.h"

/*
ImageTransformer:
*/

class ImageTransformer : public ImageServer {
  public:
    // constructor / desctructor
    ImageTransformer(Debuglevel imageTransformerDebugLevel);
    ImageTransformer();
    // other
    void run(); // runs the service: Consume undistorted images from the camera driver, blur, warp,... and move the bird eye's view into the image queue.
    void mountCamerDriver(std::shared_ptr<CameraDriver> pointerToCameraDriver); // "Mounts" the instance of CameraDriver to the instance of ImageTransformer by saving the shared pointer in the member variables
    cv::Mat getImageFromMountedCameraDriver(); // Given the Camera Driver is mounted, this methof pulls an image.
  private:
    void convertToBinaryImage(cv::Mat& source, cv::Mat& destination); // convert the gray(!) source image to a binary image (= total black everything below a threshold and totla white anything above threshold)
    void convertToBirdEyesView(cv::Mat& source, cv::Mat& destination); // compute the bird eye's view of the source image
    void convertToGrayImage(cv::Mat& source, cv::Mat& destination); // convert source image to a gray image
    void detectEdges(cv::Mat& source, cv::Mat& destination); // detect edges in source image 
    void applyGausianBlurr(cv::Mat& source, cv::Mat& destination); // apply a Gaussian Noise kernel to the source image
    void maskBirdEyesView(cv::Mat& source, cv::Mat& destination); // keep the region of the image defined by the polygon formed from `vertices`. The rest of the image is set to black. Source-Image must be of size birdsEyeImageWidth x birdsEyeImageHeight
    void setBirdEyesTransformMatrix(); // compute the bird eye's transformation Matrix mapping the original image into the 'bird eye's view'.
    void transformAndStoreImage(); // runs the pipeline from "drawing an image" to "storing in the queue" once.
    const int kernelsize = 5;
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
    std::shared_ptr<CameraDriver> accessCameraDriver; // shared pointer to an instance of CameraDriver delivering images upond request
};

#endif