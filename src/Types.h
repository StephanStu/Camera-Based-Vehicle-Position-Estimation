#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H

#include <opencv2/opencv.hpp>

enum Debuglevel {none, verbose, all}; // level of command line outputs / log-level

struct ImageData { //structure to hold the raw image + derived images from that raw image, e.g. the gray-scale, the undistored, ....and an identifier, timestamp and more meta-data needed to run the image-processor
  cv::Mat rawImage;
  cv::Mat grayImage;
  unsigned long int identifier;
};

#endif