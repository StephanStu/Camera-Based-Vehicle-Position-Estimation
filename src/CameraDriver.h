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
  cv::Mat getIntrinsicMatrix();
  cv::Mat getDistortionCoefficients();
  // typical behaviour methods
  void calibrate();
  private:
  // data
  Debuglevel camerDriverDebuglevel;
  cv::Mat intrinsicMatrix;
  cv::Mat distortionCoefficients;
  // typical behaviour methods
};

#endif
