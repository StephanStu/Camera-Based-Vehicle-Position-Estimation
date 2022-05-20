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
  setBirdsEyeTransformMatrix();
}

ImageServer::ImageServer(){
  imageServerDebuglevel = Debuglevel::none;
  setBirdsEyeTransformMatrix();
}

void ImageServer::convert2BirdsEyeView(cv::Mat& source, cv::Mat& destination){
  cv::Size birdsEyeImageSize(birdsEyeImageWidth, birdsEyeImageHeight);
  cv::warpPerspective(source,			// Source image
                        destination, 	// Destination image
                        birdsEyeTransformMatrix, // Transformation matrix
                        birdsEyeImageSize,   // Size for output image. Before: image.size()
                        cv::INTER_LINEAR, // cv::WARP_INVERSE_MAP | cv::INTER_LINEAR,
                        cv::BORDER_CONSTANT, cv::Scalar::all(0) // Fill border with black
                        );
} 

void ImageServer::setBirdsEyeTransformMatrix(){
  cv::Point2f objPts[4], imgPts[4];
  objPts[0].x = objectPoint1XValue; objPts[0].y = 480; 
  objPts[1].x = objectPoint2XValue; objPts[1].y = 480;
  objPts[2].x = objectPoint3XValue; objPts[2].y = 685; 
  objPts[3].x = objectPoint4XValue; objPts[3].y = 685; 
  imgPts[0].x = 0; imgPts[0].y = 0;
  imgPts[1].x = birdsEyeImageWidth; imgPts[1].y = 0;
  imgPts[2].x = birdsEyeImageWidth; imgPts[2].y = birdsEyeImageHeight; 
  imgPts[3].x = 0; imgPts[3].y = birdsEyeImageHeight; 
  double Z = 1;
  cv::Mat H = cv::getPerspectiveTransform(objPts, imgPts);
  H.at<double>(2, 2) = Z;
  birdsEyeTransformMatrix = H;
  if((imageServerDebuglevel == Debuglevel::verbose) || (imageServerDebuglevel == Debuglevel::all)){
    std::cout << "# ImageServer::setBirdsEyeTransformMatrix called. Birds eye's view transformation matrix is:" << std::endl;
    std::cout << birdsEyeTransformMatrix << std::endl;
  }
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