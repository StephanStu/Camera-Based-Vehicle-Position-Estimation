#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "MovableImageData.h"
#include "Types.h"

MovableImageData::MovableImageData(cv::Mat raw){ // simple constructor
  rawImage = new cv::Mat;
  grayImage = new cv::Mat;
  movableImageDataDebuglevel = Debuglevel::none;
}

MovableImageData::MovableImageData(cv::Mat raw, Debuglevel debuglevel){ // constructor with setting level of command-line outputs
  rawImage = new cv::Mat;
  *rawImage = raw;
  grayImage = new cv::Mat;
  movableImageDataDebuglevel = debuglevel;
}

MovableImageData::MovableImageData(MovableImageData &&source){ //move constructor
}

MovableImageData::~MovableImageData(){ // destructor
  delete rawImage;
  delete grayImage;
}

MovableImageData::MovableImageData(const MovableImageData &source){ // copy constructor
}

MovableImageData &MovableImageData::operator=(const MovableImageData &source){// copy assignment operator
}

MovableImageData &MovableImageData::operator=(MovableImageData &&source){ // move assignment operator
} 

void MovableImageData::getImageSize(cv::Size& size){ // get /print image size to the command line
  size = (*rawImage).size();
}
