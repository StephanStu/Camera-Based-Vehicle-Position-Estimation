#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "CameraDriver.h"
#include "MovableImageData.h"
#include "Types.h"

MovableImageData::MovableImageData(cv::Mat raw) : identifier(++counter) { // simple constructor
  rawImage = new cv::Mat;
  *rawImage = raw;
  movableImageDataDebuglevel = Debuglevel::none;
}

MovableImageData::MovableImageData(cv::Mat raw, Debuglevel debuglevel) : identifier(++counter) { // constructor with setting level of command-line outputs
  rawImage = new cv::Mat;
  *rawImage = raw;
  movableImageDataDebuglevel = debuglevel;
}

MovableImageData::MovableImageData(MovableImageData &&source){ //move constructor
}

MovableImageData::~MovableImageData(){ // destructor
  delete rawImage;
}

MovableImageData::MovableImageData(const MovableImageData &source){ // copy constructor
}

MovableImageData &MovableImageData::operator=(const MovableImageData &source){// copy assignment operator
}

MovableImageData &MovableImageData::operator=(MovableImageData &&source){ // move assignment operator
} 

void MovableImageData::getRawImageSize(cv::Size& size){ // get /print image size to the command line
  if((movableImageDataDebuglevel == Debuglevel::verbose) || (movableImageDataDebuglevel == Debuglevel::all)){
    std::cout << "# MovableImageData::getRawImageSize: Returning the size of the raw image:" << std::endl;
    std::cout << (*rawImage).size() << std::endl;
  }
  size = (*rawImage).size();
}

void MovableImageData::getIdentifier(unsigned long int& id){ // get / print identifier of instance
  if((movableImageDataDebuglevel == Debuglevel::verbose) || (movableImageDataDebuglevel == Debuglevel::all)){
    std::cout << "# MovableImageData::getIdentifier: Returning this instance's identifier:" << std::endl;
    std::cout << this->identifier << std::endl;
  }
  id = this->identifier;
}

void MovableImageData::getCounter(unsigned long int& n){ // get number of instances
  if((movableImageDataDebuglevel == Debuglevel::verbose) || (movableImageDataDebuglevel == Debuglevel::all)){
    std::cout << "# MovableImageData::getCounter: Returning the counter:" << std::endl;
    std::cout << this->counter << std::endl;
  }
  n = this->counter;
}

void MovableImageData::getRawImage(cv::Mat& raw){ // get the gray image from the image data set
  if((movableImageDataDebuglevel == Debuglevel::verbose) || (movableImageDataDebuglevel == Debuglevel::all)){
    std::cout << "# MovableImageData::getRawImage: Returning the raw image." << std::endl;
  }
  cv::Size size = (*rawImage).size();
  if ((size.height > 0) && (size.width > 0)){
    raw = (*rawImage);
  }else{
    if((movableImageDataDebuglevel == Debuglevel::verbose) || (movableImageDataDebuglevel == Debuglevel::all)){
      std::cout << "# MovableImageData::getRawImage: Error. Raw image is not available, raw image is empty." << std::endl;
    }  
  }
}
    
unsigned long int MovableImageData::counter = 0; //initializing the static member variable here