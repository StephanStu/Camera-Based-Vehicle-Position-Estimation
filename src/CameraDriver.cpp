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
  // use a sample image as "the next raw image made available"
  rawImage = cv::imread("SimpleRunwayTestImage.png" ,cv::IMREAD_COLOR);
  cv::resize(rawImage, rawImage, cv::Size(1280, 720));
}

CameraDriver::CameraDriver(){
  camerDriverDebuglevel = Debuglevel::none;
  // use a sample image as "the next raw image made available"
  rawImage = cv::imread("SimpleRunwayTestImage.png" ,cv::IMREAD_COLOR);
  cv::resize(rawImage, rawImage, cv::Size(1280, 720));
}

void CameraDriver::calibrate(){
  int numberOfBoards = 17;
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
    //std::cout << images.back().size() << std::endl;
  }
  int numberOfTiles = boardWidth * boardHeight;
  cv::Size boardSize = cv::Size(boardWidth, boardHeight);
  std::vector<std::vector<cv::Point2f>> imagePoints;
  std::vector<std::vector<cv::Point3f>> objectPoints;
  cv::Size imageSize;
  std::vector<cv::Point2f> corners;
  while (images.size() > 0){
    // Find the board
    bool found = cv::findChessboardCorners(images.back(), boardSize, corners);
    if(found){
      imageSize = images.back().size();
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
  // cv::Mat intrinsicMatrix, distortionCoeffs;
  double err = cv::calibrateCamera(objectPoints, imagePoints, imageSize, intrinsicMatrix, distortionCoefficients, cv::noArray(), cv::noArray(), cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT);  
  if((camerDriverDebuglevel == Debuglevel::verbose) || (camerDriverDebuglevel == Debuglevel::all)){
    std::cout << "# CameraDriver::calibrate: Found intrinsic matrix: "<< std::endl;
    std::cout << intrinsicMatrix << std::endl;
    std::cout << "# CameraDriver::calibrate: Found distortion coefficients: "<< std::endl;
    std::cout << distortionCoefficients << std::endl;
  }
}

void CameraDriver::getIntrinsicMatrix(cv::Mat& matrix){
  if((camerDriverDebuglevel == Debuglevel::verbose) || (camerDriverDebuglevel == Debuglevel::all)){
    std::cout << "# CameraDriver::getIntrinsicMatrix: Returning the intrinsic matrix:" << std::endl;
    std::cout << this->intrinsicMatrix << std::endl;
  }
  matrix = this->intrinsicMatrix;
}

void CameraDriver::getDistortionCoefficients(cv::Mat& matrix){
  if((camerDriverDebuglevel == Debuglevel::verbose) || (camerDriverDebuglevel == Debuglevel::all)){
    std::cout << "# CameraDriver::getDistortionCoefficients: Returning the distortion coefficients:" << std::endl;
    std::cout << this->distortionCoefficients << std::endl;
  }
  matrix = this->distortionCoefficients;
}

void CameraDriver::getRawImage(cv::Mat& destination){
  destination = rawImage;
}

void CameraDriver::getUndistortedImage(cv::Mat& destination){// using (a reference to) the camera's intrinsic matrix & distortion-coefficients this method undistorts the source image
  if((camerDriverDebuglevel == Debuglevel::verbose) || (camerDriverDebuglevel == Debuglevel::all)){
    std::cout << "# CameraDriver::getUndistortedImage called." << std::endl;
  }
  cv::undistort(rawImage, destination, intrinsicMatrix, distortionCoefficients);  
}