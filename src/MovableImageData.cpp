#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "MovableImageData.h"
#include "Types.h"

MovableImageData::MovableImageData(cv::Mat raw) : identifier(++counter) { // simple constructor
  rawImage = new cv::Mat;
  grayImage = new cv::Mat;
  movableImageDataDebuglevel = Debuglevel::none;
}

MovableImageData::MovableImageData(cv::Mat raw, Debuglevel debuglevel) : identifier(++counter) { // constructor with setting level of command-line outputs
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
  if((movableImageDataDebuglevel == Debuglevel::verbose) || (movableImageDataDebuglevel == Debuglevel::all)){
    std::cout << "# MovableImageData::getImageSize: Returning the size of the (raw) image:" << std::endl;
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

void MovableImageData::getGrayImage(cv::Mat& gray){ // get the gray image from the image data set
  if((movableImageDataDebuglevel == Debuglevel::verbose) || (movableImageDataDebuglevel == Debuglevel::all)){
    std::cout << "# MovableImageData::getGrayImage: Returning the gray image." << std::endl;
  }
  gray = (*grayImage);
}
    
void MovableImageData::convert2GrayImage(){ // set the gray image in the image data set based on the raw image (assumed to be colored)
  if((movableImageDataDebuglevel == Debuglevel::verbose) || (movableImageDataDebuglevel == Debuglevel::all)){
    std::cout << "# MovableImageData::convert2GrayImage: Adding the gray image to the data set." << std::endl;
  }
  cv::cvtColor(*rawImage, *grayImage, cv::COLOR_BGR2GRAY);
}

void MovableImageData::detectEdges(){ // detect edges in the gray image

}

unsigned long int MovableImageData::counter = 0; //initializing the static member variable here