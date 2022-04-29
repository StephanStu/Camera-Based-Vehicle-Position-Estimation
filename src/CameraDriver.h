#ifndef CAMERADRIVER_H
#define CAMERADRIVER_H

#include <opencv2/opencv.hpp>

#include "MovableImageData.h"
#include "Types.h"

class CameraDriver{
  public:
  // constructor / desctructor
  CameraDriver(Debuglevel debuglevel);
  CameraDriver();
  // getters / setters
  void getIntrinsicMatrix(cv::Mat& mat);
  void getDistortionCoefficients(cv::Mat& mat);
  // typical behaviour methods
  void calibrate();
  void getImageData(MovableImageData& targetImage);
  private:
  // data
  Debuglevel camerDriverDebuglevel;
  cv::Mat intrinsicMatrix;
  cv::Mat distortionCoefficients;
  // typical behaviour methods
};

#endif
