#include "CameraDriver.h"

CameraDriver::CameraDriver(){
  debugLevel = Debuglevel::none;
  printToConsole("CameraDriver::CameraDriver called.");
}

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
    if(currentState == initializing){
      printToConsole("CameraDriver::sourceRawImages is called, instance is waiting in state initializing.");
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
    if(currentState == running){
      printToConsole("CameraDriver::sourceRawImages is running and feeding images into its queue.");
      cv::Mat image = cv::imread("test/test01.jpg" ,cv::IMREAD_COLOR);
      addImageToQueue(std::move(image));
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
    if(currentState == terminated){
      printToConsole("CameraDriver::sourceRawImages is called, instance has reached state terminated. Quitting.");
      break;
    }
    if(currentState == freezed){
      printToConsole("CameraDriver::sourceRawImages is called, instance is waiting in state freezed.");
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
  }
};