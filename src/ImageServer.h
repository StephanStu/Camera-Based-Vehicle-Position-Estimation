#ifndef IMAGESERVER_H
#define IMAGESERVER_H

#include <opencv2/opencv.hpp>

#include "CameraDriver.h"

class ImageServer{
  public:
  // constructor / desctructor
  ImageServer(Debuglevel debuglevel);
  ImageServer();
  // getters / setters
  
  // typical behaviour methods
  void convert2BirdsEyeView(cv::Mat& source, cv::Mat& destination); // compute the bird eye's view of the source image
  void convert2GrayImage(cv::Mat& source, cv::Mat& destination); // convert source image to a gray image
  void detectEdges(cv::Mat& source, cv::Mat& destination); // detect edges in source image 
  void applyGausianBlurr(cv::Mat& source, cv::Mat& destination); // apply a Gaussian Noise kernel to the source image
  void maskImage(cv::Mat& source, cv::Mat& destination); // keep the region of the image defined by the polygon formed from `vertices`. The rest of the image is set to black.
  void undistortImage(CameraDriver& camera, cv::Mat& source, cv::Mat& destination); // using (a reference to) the camera's intrinsic matrix & distortion-coefficients this method undistorts the source image
  private:
  // data
  Debuglevel imageServerDebuglevel;
  const int kernelsize = 5;
  const int birdsEyeImageWidth = 500; // default = 500 (taken from GitHub), width of bird's eye image (after running the transform)
  const int birdsEyeImageHeight = 600; // defualt = 600 (taken from GitHub), height of bird's eye image (after running the transform)
  const int objectPoint1XValue = 375; // value of x coordinate in mapping from object-points -> image-points in bird's eye transform
  const int objectPoint1YValue = 480; // value of y coordinate in mapping from object-points -> image-points in bird's eye transform
  const int objectPoint2XValue = 905; // value of x coordinate in mapping from object-points -> image-points in bird's eye transform
  const int objectPoint2YValue = 480; // value of y coordinate in mapping from object-points -> image-points in bird's eye transform
  const int objectPoint3XValue = 1811; // value of x coordinate in mapping from object-points -> image-points in bird's eye transform
  const int objectPoint3YValue = 685; // value of y coordinate in mapping from object-points -> image-points in bird's eye transform
  const int objectPoint4XValue = -531; // value of x coordinate in mapping from object-points -> image-points in bird's eye transform
  const int objectPoint4YValue = 685; // value of y coordinate in mapping from object-points -> image-points in bird's eye transform
  cv::Mat birdsEyeTransformMatrix; // Matrix for bird's eye perspective transform
  // typical behaviour methods
  void pullImageFromCameraDriver(CameraDriver& camera);
  void setBirdsEyeTransformMatrix();
};

#endif