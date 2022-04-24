#include <iostream>
#include <string> 
#include <vector>
#include <opencv2/opencv.hpp>

#include "CameraDriver.h"
#include "Types.h"

/* Implementation of class "CameraDriver" */

CameraDriver::CameraDriver(Debuglevel debuglevel){
  camerDriverDebuglevel = debuglevel;
  if((debuglevel == Debuglevel::verbose) || (debuglevel == Debuglevel::all)){
    std::cout << "# CameraDriver::CameraDriver called." << std::endl;
  }
}

CameraDriver::CameraDriver(){
  camerDriverDebuglevel = Debuglevel::none;
}

void CameraDriver::calibrate(){
  int numberOfBoards = 12;
  float imageScalingFactor = 1.0f;
  int boardWidth = 9;
  int boardHeight = 6;
  std::vector<cv::Mat> images;
  std::string path("calibration/");
  std::string fileNameBase("calibration");
  std::string zero("0");
  std::string fileType(".jpg");
  std::string imageFileName;
  for (int i = 1; i <= numberOfBoards; i++) {
    std::string fileNumber = std::to_string(i);
    if(i<10){
       imageFileName = path + fileNameBase + zero + fileNumber + fileType;    
    }else{
       imageFileName = path + fileNameBase + fileNumber + fileType;    
    }
    if((camerDriverDebuglevel == Debuglevel::verbose) || (camerDriverDebuglevel == Debuglevel::all)){
      std::cout << "# CameraDriver::calibrate: Added image "<< imageFileName << " to set of calibration images." << std::endl;
    }
    images.push_back(cv::imread(imageFileName, cv::IMREAD_COLOR));
    std::cout << images.back().size() << std::endl;
  }
  int numberOfTiles = boardWidth * boardHeight;
  cv::Size boardSize = cv::Size(boardWidth, boardHeight);
  std::vector<std::vector<cv::Point2f>> imagePoints;
  std::vector<std::vector<cv::Point3f>> objectPoints;
  cv::Size imageSize;
  while (images.size() > 0){
    // Find the board
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(images.back(), boardSize, corners);
    if(found){
      drawChessboardCorners(images.back(), boardSize, corners, found);
      if(camerDriverDebuglevel == Debuglevel::verbose){
        std::cout << "# CameraDriver::calibrate: Found the chessboard of size " << boardSize << "." << std::endl;
      }
      if(camerDriverDebuglevel == Debuglevel::all){
        std::cout << "# CameraDriver::calibrate: Found the chessboard of size " << boardSize << "." << std::endl;
        cv::imshow("Calibration", images.back());
        cv::waitKey(0);
      }
      images.pop_back();
      imagePoints.push_back(corners);
      objectPoints.push_back(std::vector<cv::Point3f>());
      std::vector<cv::Point3f> &opts = objectPoints.back();
      opts.resize(numberOfTiles);
      for (int j = 0; j < numberOfTiles; j++) {
        opts[j] = cv::Point3f(static_cast<float>(j / boardWidth), static_cast<float>(j % boardWidth), 0.0f);
      }
    }else{
      std::cout << "# CameraDriver::calibrate: Cannot find a chessboard of size " << boardSize << ". Rejecting this image in the calibration process." << std::endl;
      cv::imshow("Calibration", images.back());
      cv::waitKey(0);
      images.pop_back();
    }
  }
  // Calibration step to arrive at camera-matriox and distortion coefficients
  cv::Mat intrinsicMatrix, distortionCoeffs;
  double err = cv::calibrateCamera(objectPoints, imagePoints, imageSize, intrinsicMatrix, distortionCoeffs, cv::noArray(), cv::noArray(), cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT);  
}
