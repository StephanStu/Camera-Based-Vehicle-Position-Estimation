#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "MovableImageData.h"
#include "Types.h"

MovableImageData::MovableImageData(cv::Mat raw){ // constructor
}

MovableImageData::MovableImageData(cv::Mat raw, Debuglevel debuglevel){ // constructor
}

MovableImageData::MovableImageData(MovableImageData &&source){ // 4 : move constructor
}

MovableImageData::~MovableImageData(){ // 1 : destructor
}

MovableImageData::MovableImageData(const MovableImageData &source){ // 2 : copy constructor
}

MovableImageData &operator=(const MovableImageData &source){// 3 : copy assignment operator
}
MovableImageData &operator=(MovableImageData &&source){ // 5 : move assignment operator
} 
   
