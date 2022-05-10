#ifndef MOVABLEIMAGEDATA_H
#define MOVABLEIMAGEDATA_H

#include <opencv2/opencv.hpp>
#include "CameraDriver.h"
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
    void getRawImageSize(cv::Size& size); // get / print size of the image to the commandline
    void getIdentifier(unsigned long int& id); // get / print identifier of instance
    void getCounter(unsigned long int& n); // get number of instances
    void getRawImage(cv::Mat& raw); // get the gray image from the image data set
    private:
    const int kernelsize = 5; 
    cv::Mat *rawImage; // raw image captured with camera
    Debuglevel movableImageDataDebuglevel; // level of command line output useful in debugging
    unsigned long int identifier; // identifier, incremented when a new instance is created
    static unsigned long int counter; // number of image-data-instances created during runtime
};

#endif