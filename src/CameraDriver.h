#ifndef CAMERADRIVER_H
#define CAMERADRIVER_H

#include <opencv2/opencv.hpp>

#include "Types.h"

class CameraDriver{
  public:
  // constructor / desctructor
  CameraDriver(Debuglevel debuglevel);
  CameraDriver();
  // getters / setters
  void getIntrinsicMatrix(cv::Mat& matrix);
  void getDistortionCoefficients(cv::Mat& matrix);
  // typical behaviour methods
  void calibrate();
  void getRawImage(cv::Mat& destination);
  void getUndistortedImage(cv::Mat& destination);
  private:
  // data
  Debuglevel camerDriverDebuglevel;
  cv::Mat intrinsicMatrix;
  cv::Mat distortionCoefficients;
  cv::Mat rawImage;
  // typical behaviour methods
};

#endif
