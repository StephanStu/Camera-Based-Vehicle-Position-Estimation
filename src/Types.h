#ifndef TYPES_H
#define TYPES_H
#include <opencv2/opencv.hpp>

enum Debuglevel {none, verbose}; // level of command line outputs / log-level

enum State {initializing, running, freezed, terminated}; // states to control the application

enum Stimulation{image, video}; // ways to excite the service for testing on Linux: Statis image with artificial timestep or video

enum Filetype{mp4, jpg, unknown}; // allowable filetypes that can be consumed by the service

struct PositionServiceRecord{
  cv::Mat rawImage;
  cv::Mat undistortedImage;
  cv::Mat birdEyesViewImage;
  cv::Mat binaryBirdEyesViewImage;
  cv::Mat houghLinesImage;
  float distanceToLeftLane; // measured in the image
  float distanceToRightLane; // measured in the image
  float deviation; // output of the kalman filter
  float angle; // output of the kalman filter
  float velocity; // measured in the vehicle, usually by catching wheel rotational speed
};

struct TripRecorderRecord{
  float time; // time 
  float px; // estimated state: position of road vehicle in x-direction 
  float py; // estimated state: position of road vehicle in y-direction 
  float vx; // estimated state: velocity of road vehicle in x-direction 
  float vy; // estimated state: velocity of road vehicle in y-direction 
  float angle; // derived from estimated state: angle between road vehicle's direction of travelling and the road's center line
  float leftDeviation; // distance to left lane "measured" in the imge
  float leftAngle; // angle between road vehicle's direction of travelling and the road's center line, "measured" in image
  float rightDeviation; // distance to right lane "measured" in the imge
  float rightAngle; // angle between road vehicle's direction of travelling and the road's center line, "measured" in image
};

struct Position{
  float deviation;
  float angle;
};

struct Measurement{
  float deviation;
  float angle;
  float velocity;
};

#endif