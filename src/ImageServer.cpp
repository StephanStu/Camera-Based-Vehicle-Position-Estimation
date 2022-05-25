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

void ImageServer::addImageToQueue(cv::Mat &&image){
  // perform modification under the lock
  cv::Size size = image.size();
  std::lock_guard<std::mutex> uLock(queueMutex);
  imageQueue.push_back(std::move(image));
  // notify clients
  queueCondition.notify_one(); 
  if((imageServerDebuglevel == Debuglevel::verbose) || (imageServerDebuglevel == Debuglevel::all)){
    std::cout << "# ImageServer::addImageToQueue called. Adding another image of size " << size << " to the queue. " << std::endl;
    std::cout << "# ImageServer::imageQueue is now of size " << imageQueue.size() << std::endl;
  }
}

cv::Mat ImageServer::getImageFromQueue(){
  /*In the method popBack, we need to create the lock first - it can not be a lock_guard any more as we need to pass it to the condition variable - to its method wait. Thus it must be a unique_lock. Now we can enter the wait state while at same time releasing the lock. It is only inside wait, that the mutex is temporarily unlocked - which is a very important point to remember: We are holding the lock before AND after our call to wait - which means that we are free to access whatever data is protected by the mutex. */
  // perform queue modification under the lock
  std::unique_lock<std::mutex> uLock(queueMutex);
  /*If the vector is empty, wait is called. When the thread wakes up again, the condition is immediately re-checked and - in case it has not been a spurious wake-up we can continue with our job and retrieve the vector.*/
  if((imageServerDebuglevel == Debuglevel::verbose) || (imageServerDebuglevel == Debuglevel::all)){
    if(imageQueue.empty()){
      std::cout << "# ImageServer::getImageFromQueue called. Image queue is currently empty." << std::endl;
    }
  }
  queueCondition.wait(uLock, [this] { return !imageQueue.empty(); }); // pass unique lock to condition variable
  // remove last vector element from queue
  cv::Mat result = std::move(imageQueue.back());
  imageQueue.pop_back();
  if((imageServerDebuglevel == Debuglevel::verbose) || (imageServerDebuglevel == Debuglevel::all)){
    std::cout << "# ImageServer::getImageFromQueue called. Popping an image from the queue. " << std::endl;
    std::cout << "# ImageServer::imageQueue is now of size " << imageQueue.size() << std::endl;
  }
  /*Pipeline for image processing starts here*/
  applyGausianBlurr(result, result);
  convert2GrayImage(result, result); 
  convert2BirdsEyeView(result, result);
  convert2BinaryImage(result, result);
  /*Pipeline for image processing ends here*/
  return result; // will not be copied due to return value optimization (RVO) in C++
}

void ImageServer::convert2BinaryImage(cv::Mat& source, cv::Mat& destination){
  if((imageServerDebuglevel == Debuglevel::verbose) || (imageServerDebuglevel == Debuglevel::all)){
    std::cout << "# ImageServer::convert2BinaryImage called." << std::endl;
  }
  //int maxBinaryValue = 255;
  //int thresholdValue = 50;
  cv::threshold(source, destination, binaryThresholdValue, maxBinaryValue, cv::THRESH_BINARY);
}

void ImageServer::convert2BirdsEyeView(cv::Mat& source, cv::Mat& destination){
  if((imageServerDebuglevel == Debuglevel::verbose) || (imageServerDebuglevel == Debuglevel::all)){
    std::cout << "# ImageServer::convert2BirdsEyeView called." << std::endl;
  }
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

void ImageServer::maskBirdsEyeImage(cv::Mat& source, cv::Mat& destination){ // keep the region of the image defined by the polygon formed from `vertices`. The rest of the image is set to black.
}