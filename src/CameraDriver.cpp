#include "CameraDriver.h"

CameraDriver::CameraDriver(Debuglevel cameraDriverDebugLevel){
  debugLevel = cameraDriverDebugLevel;
  printToConsole("CameraDriver::CameraDriver called.");
}

void CameraDriver::run(){
  printToConsole("CameraDriver::run called.");
  threads.emplace_back(std::thread(&CameraDriver::sourceRawImages, this));
}
    
void CameraDriver::calibrate(){
  printToConsole("CameraDriver::calibrate called.");
}

void CameraDriver::undistortImage(cv::Mat& source, cv::Mat& destination){
  printToConsole("CameraDriver::undistortImage called.");
  cv::undistort(source, destination, intrinsicMatrix, distortionCoefficients);
}

void CameraDriver::sourceRawImages(){
  while(true){
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    printToConsole("CameraDriver::sourceRawImages is still running.");
    cv::Mat image = cv::imread("test/test01.jpg" ,cv::IMREAD_COLOR);
    addImageToQueue(std::move(image));
  }
};