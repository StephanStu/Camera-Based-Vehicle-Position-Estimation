#include <iostream>
#include <string> 
#include <vector>
#include <opencv2/opencv.hpp>

#include "CameraDriver.h"
#include "ImageServer.h"
#include "Types.h"

/* Implementation of class "ImageServer" */

ImageServer::ImageServer(Debuglevel debuglevel){
  imageServerDebuglevel = debuglevel;
  if((debuglevel == Debuglevel::verbose) || (debuglevel == Debuglevel::all)){
    std::cout << "# ImageServer::ImageServer called." << std::endl;
  }
}

ImageServer::ImageServer(){
  imageServerDebuglevel = Debuglevel::none;
}

void ImageServer::convert2GrayImage(cv::Mat& source, cv::Mat& destination){ // convert source image to a gray image
  if((imageServerDebuglevel == Debuglevel::verbose) || (imageServerDebuglevel == Debuglevel::all)){
    std::cout << "# ImageServer::convert2GrayImage called." << std::endl;
  }
  cv::cvtColor(source, destination, cv::COLOR_BGR2GRAY);
}

void ImageServer::detectEdges(cv::Mat& source, cv::Mat& destination){ // detect edges in source image 
  /*NOT IMPLEMENTED YET*/
  destination = source;
}

void ImageServer::applyGausianBlurr(cv::Mat& source, cv::Mat& destination){ // apply a Gaussian Noise kernel to the source image
  if((imageServerDebuglevel == Debuglevel::verbose) || (imageServerDebuglevel == Debuglevel::all)){
    std::cout << "# ImageServer::applyGausianBlurr called." << std::endl;
  }
  cv::Size size(kernelsize, kernelsize);
  cv::GaussianBlur(source, destination, size, 0, 0);
}

void ImageServer::maskImage(cv::Mat& source, cv::Mat& destination){ // keep the region of the image defined by the polygon formed from `vertices`. The rest of the image is set to black.
}

void ImageServer::undistortImage(CameraDriver& camera, cv::Mat& source, cv::Mat& destination){ // using (a reference to) the camera's intrinsic matrix & distortion-coefficients this method undistorts the source image
  cv::Mat intrinsicMatrix;
  cv::Mat distortionCoefficients;
  camera.getIntrinsicMatrix(intrinsicMatrix);
  camera.getDistortionCoefficients(distortionCoefficients);
  if((imageServerDebuglevel == Debuglevel::verbose) || (imageServerDebuglevel == Debuglevel::all)){
    std::cout << "#  ImageServer::undistortImage called." << std::endl;
  }
  cv::undistort(source, destination, intrinsicMatrix, distortionCoefficients);  
}