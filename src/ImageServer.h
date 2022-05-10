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
  void convert2GrayImage(cv::Mat& source, cv::Mat& destination); // convert source image to a gray image
  void detectEdges(cv::Mat& source, cv::Mat& destination); // detect edges in source image 
  void applyGausianBlurr(cv::Mat& source, cv::Mat& destination); // apply a Gaussian Noise kernel to the source image
  void maskImage(cv::Mat& source, cv::Mat& destination); // keep the region of the image defined by the polygon formed from `vertices`. The rest of the image is set to black.
  void undistortImage(CameraDriver& camera, cv::Mat& source, cv::Mat& destination); // using (a reference to) the camera's intrinsic matrix & distortion-coefficients this method undistorts the source image
  private:
  // data
  Debuglevel imageServerDebuglevel;
  const int kernelsize = 5;
  // typical behaviour methods
  void pullImageFromCameraDriver(CameraDriver& camera);
};

#endif