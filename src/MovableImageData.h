#ifndef MOVABLEIMAGEDATA_H
#define MOVABLEIMAGEDATA_H

#include <opencv2/opencv.hpp>
#include "Types.h"

class MovableImageData{
    public:
    MovableImageData(cv::Mat raw);  // simple constructor
    MovableImageData(cv::Mat raw, Debuglevel debuglevel);  // constructor with setting level of command-line outputs
    ~MovableImageData();  // destructor
    MovableImageData &operator=(const MovableImageData &source);  // copy assignment operator
    MovableImageData(MovableImageData &&source); // move constructor
    MovableImageData(const MovableImageData &source); // copy constructor
    MovableImageData &operator=(MovableImageData &&source); // move assignment operator
    void getImageSize(cv::Size& size); // get / print size of the image to the commandline
    private:
    cv::Mat *rawImage; // raw image captured with camera
    cv::Mat *grayImage; // raw image converted to gray scale
    Debuglevel movableImageDataDebuglevel; // level of command line output useful in debugging
    unsigned long int identifier; // identifier, incremented when a new instance is created
};

#endif