#ifndef TYPES_H
#define TYPES_H
#include <opencv2/opencv.hpp>

enum Debuglevel {none, verbose, all}; // level of command line outputs / log-level

enum State {initializing, running, freezed, terminated}; // states to control the application

struct PositionServiceRecord{
  cv::Mat rawImage;
  cv::Mat undistortedImage;
  cv::Mat birdEyesViewImage;
  cv::Mat binaryBirdEyesViewImage;
  cv::Mat houghLinesImage;
  float distanceToLeftLane;
  float distanceToRightLane;
  float angle;
  float deviation;
};

struct Position{
  float angle;
  float deviation;
};

#endif