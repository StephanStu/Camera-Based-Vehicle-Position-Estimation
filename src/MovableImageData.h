#ifndef TYPES_H
#define TYEPS_H

#include <opencv2/opencv.hpp>
#include "Types.h"

class MovableImageData{
    public:
    MovableImageData(cv::Mat raw) // constructor
    MovableImageData(cv::Mat raw, Debuglevel debuglevel) // constructor
    MovableImageData(MovableImageData &&source) // 4 : move constructor
    ~MovableImageData() // 1 : destructor
    MovableImageData(const MovableImageData &source) // 2 : copy constructor
    private:
    ImageData *data;
    Debuglevel movableImageDataDebuglevel;
};

MovableImageData &operator=(const MovableImageData &source) // 3 : copy assignment operator
MovableImageData &operator=(MovableImageData &&source) // 5 : move assignment operator

#endif