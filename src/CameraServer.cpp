#include "CameraServer.h"
#include <string>

CameraServer::CameraServer(Debuglevel cameraServerDebugLevel) : calibrationDataIsLoaded(false) {
  debugLevel = cameraServerDebugLevel;
  printToConsole("CameraServer::CameraServer called.");
}

CameraServer::CameraServer() : calibrationDataIsLoaded(false) {
  debugLevel = Debuglevel::none;
}

void CameraServer::run(){
  printToConsole("CameraServer::run called.");
  threads.emplace_back(std::thread(&CameraServer::manageStateSwitches, this));
}

void CameraServer::calibrate(){
  printToConsole("CameraServer::calibrate called. Reading 17 images from ../calibration/ to compute the intrinsic matrix & the distortion coefficients.");
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
    std::string message = "CameraDriver::calibrate: Using the following image in calibration now: " + imageFileName;
    printToConsole(message);
    images.push_back(cv::imread(imageFileName, cv::IMREAD_COLOR));
  }
  int numberOfTiles = boardWidth * boardHeight;
  cv::Size boardSize = cv::Size(boardWidth, boardHeight);
  std::vector<std::vector<cv::Point2f>> imagePoints;
  std::vector<std::vector<cv::Point3f>> objectPoints;
  //cv::Size imageSize;
  std::vector<cv::Point2f> corners;
  while (images.size() > 0){
    // Find the board
    bool found = cv::findChessboardCorners(images.back(), boardSize, corners);
    if(found){
      imageSize = images.back().size();
      drawChessboardCorners(images.back(), boardSize, corners, found);
      images.pop_back();
      imagePoints.push_back(corners);
      objectPoints.push_back(std::vector<cv::Point3f>());
      std::vector<cv::Point3f> &opts = objectPoints.back();
      opts.resize(numberOfTiles);
      for (int j = 0; j < numberOfTiles; j++) {
        opts[j] = cv::Point3f(static_cast<float>(j / boardWidth), static_cast<float>(j % boardWidth), 0.0f);
      }
    }else{
      printToConsole("CameraDriver::calibrate: Error, cannot find a chessboard here!");
      cv::imshow("Calibration", images.back());
      cv::waitKey(0);
      images.pop_back();
    }  
  }
  double err = cv::calibrateCamera(objectPoints, imagePoints, imageSize, intrinsicMatrix, distortionCoefficients, cv::noArray(), cv::noArray(), cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT);  
  printToConsole("CameraDriver::calibrate: Found the intrinsic matrix!");
  printToConsole("CameraDriver::calibrate: Found the distortion coefficients!");
  calibrationDataIsLoaded = true;
}

void CameraServer::writeCameraCalibrationDataToFile(){
  printToConsole("CameraServer::writeCameraCalibrationDataToFile called. Saving calibration date in intrinsics.xml.");
  if(calibrationDataIsLoaded){
    cv::FileStorage fs("intrinsics.xml", cv::FileStorage::WRITE);
    fs << "image_width" << imageSize.width << "image_height" << imageSize.height
     << "camera_matrix" << intrinsicMatrix << "distortion_coefficients"
     << distortionCoefficients;
    fs.release();
  }else{
    printToConsole("CameraServer::writeCameraCalibrationDataToFile called but this instance does not have populated the intrinsic matrix and the distortion coefficient yet. Run CameraServer::calibrate or CameraServer::readCameraCalibrationDataFromFile to make this instance ready.");
  }
}

void CameraServer::readCameraCalibrationDataFromFile(){
  printToConsole("CameraDriver::readCameraCalibrationDataFromFile called.");
  int imageWidth;
  int imageHeight;
  cv::FileStorage fs("intrinsics.xml", cv::FileStorage::READ);
  fs["camera_matrix"] >> intrinsicMatrix;
  fs["distortion_coefficients"] >> distortionCoefficients;
  fs["image_width"] >> imageWidth;
  fs["image_height"] >> imageHeight;
  imageSize = cv::Size(imageWidth, imageHeight);
  calibrationDataIsLoaded  = true;
}

void CameraServer::undistortImage(cv::Mat& source, cv::Mat& destination){
  if(calibrationDataIsLoaded){
    printToConsole("CameraServer::undistortImage called.");
    cv::undistort(source, destination, intrinsicMatrix, distortionCoefficients);
  }else{
    printToConsole("CameraServer::undistortImage called, but intrinsic matrix and distortion coefficients are not populated, returning the undistorted image now.");
    destination = source;
  }
}

void CameraServer::createBlackRecord(MovableTimestampedType<PositionServiceRecord>& record){
  if(calibrationDataIsLoaded){
    std::string message = "CameraDriver::createBlackRecord: Filling the raw & undistored image with black images of width " + std::to_string(imageSize.width) + " and height " + std::to_string(imageSize.height) + ".";
    printToConsole(message);
    PositionServiceRecord data;
    data.rawImage = cv::Mat::zeros(imageSize, CV_8UC3);
    data.undistortedImage = cv::Mat::zeros(imageSize, CV_8UC3);
    data.distanceToLeftLane = 0.0;
    data.distanceToRightLane = 0.0;
    data.angle = 0.0;
    data.deviation = 0.0;
    record.setData(data);
  }else{
    std::string message = "CameraDriver::createBlackRecord: Image size is not defined yet. Run CameraServer::calibrate or CameraServer::readCameraCalibrationDataFromFile to make this instance ready.";
    printToConsole(message);
  }
}

void CameraServer::getImageSize(cv::Size& size){
  if(calibrationDataIsLoaded){
    std::string message = "CameraDriver::getImageSize: Serving images of width " + std::to_string(imageSize.width) + " and height " + std::to_string(imageSize.height) + ".";
    printToConsole(message);
    size = imageSize;
  }else{
    std::string message = "CameraDriver::getImageSize: Image size is not defined yet. Run CameraServer::calibrate or CameraServer::readCameraCalibrationDataFromFile to make this instance ready.";
    printToConsole(message);
  }
}

void CameraServer::manageStateSwitches(){
  while(true){
    if(currentState == initializing){
      printToConsole("CameraServer::manageStateSwitches is called: Instance is waiting in state initializing.");
      runInInitializingState();
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
    if(currentState == running){
      if(ready){
        printToConsole("CameraServer::manageStateSwitches is called: Instance is in state running.");
        runInRunningState();
      }else{
        printToConsole("CameraServer::manageStateSwitches is called: Instance is not ready & calling the initializing routine.");
        runInInitializingState();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
    if(currentState == terminated){
      printToConsole("CameraServer::manageStateSwitches is called: Instance has reached state terminated.");
      runInTerminatedState();
      break;
    }
    if(currentState == freezed){
      printToConsole("CameraServer::manageStateSwitches is called: Instance is waiting in state freezed.");
      runInFreezedState();
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
  }
}
    
void CameraServer::runInRunningState(){
  cv::Mat rawImage = cv::imread("test/test01.jpg" ,cv::IMREAD_COLOR);
  cv::Mat undistortedImage;
  undistortImage(rawImage, undistortedImage);
  PositionServiceRecord newRecord;
  newRecord.rawImage = std::move(rawImage);
  newRecord.undistortedImage = std::move(undistortedImage);
  MovableTimestampedType<PositionServiceRecord> newMovableTimestampedRecord(newRecord, debugLevel);
  std::unique_lock<std::mutex> uniqueLock(protection);
  condition.wait(uniqueLock); 
  record = std::move(newMovableTimestampedRecord);
  isCurrent = true;
  uniqueLock.unlock();
  condition.notify_one();
}
    
void CameraServer::runInInitializingState(){
  isCurrent = false;
  ready = calibrationDataIsLoaded;
  if(!ready){
    printToConsole("CameraServer::runInInitializingState is called but instance is not ready yet.");
  }else{
    PositionServiceRecord newRecord;
    MovableTimestampedType<PositionServiceRecord> newMovableTimestampedRecord(newRecord, debugLevel);
    createBlackRecord(newMovableTimestampedRecord);
    std::unique_lock<std::mutex> uniqueLock(protection);
    condition.wait(uniqueLock); 
    record = std::move(newMovableTimestampedRecord);
    uniqueLock.unlock();
    condition.notify_one();
  }
}
    
void CameraServer::runInFreezedState(){
  isCurrent = false;
}
    
void CameraServer::runInTerminatedState(){
  isCurrent = false;
  PositionServiceRecord newRecord;
  MovableTimestampedType<PositionServiceRecord> newMovableTimestampedRecord(newRecord, debugLevel);
  createBlackRecord(newMovableTimestampedRecord);
  std::unique_lock<std::mutex> uniqueLock(protection);
  condition.wait(uniqueLock); 
  record = std::move(newMovableTimestampedRecord);
  uniqueLock.unlock();
  condition.notify_one();
}