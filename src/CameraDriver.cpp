#include "CameraDriver.h"

CameraDriver::CameraDriver() : newRecordIsCurrent(false) {
  debugLevel = Debuglevel::none;
}

CameraDriver::CameraDriver(Debuglevel cameraDriverDebugLevel) : newRecordIsCurrent(false) {
  debugLevel = cameraDriverDebugLevel;
  printToConsole("CameraDriver::CameraDriver called.");
}

void CameraDriver::run(){
  printToConsole("CameraDriver::run called.");
  threads.emplace_back(std::thread(&CameraDriver::manageStateSwitches, this));
}
    
void CameraDriver::calibrate(){
  printToConsole("CameraDriver::calibrate called. Reading 17 images from ../calibration/ to compute the intrinsic matrix & the distortion coefficients. Hit enter to go from one image to the next.");
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
  ready = true;
}

void CameraDriver::writeCameraCalibrationDataToFile(){
  printToConsole("CameraDriver::writeCameraCalibrationDataToFile called. Saving calibration date in intrinsics.xml.");
  if(ready){
    cv::FileStorage fs("intrinsics.xml", cv::FileStorage::WRITE);
    fs << "image_width" << imageSize.width << "image_height" << imageSize.height
     << "camera_matrix" << intrinsicMatrix << "distortion_coefficients"
     << distortionCoefficients;
    fs.release();
  }else{
    printToConsole("CameraDriver::writeCameraCalibrationDataToFile called but this instance does not have populated the intrinsic matrix and the distortion coefficient yet. Run CameraDriver::calibrate or CameraDriver::readCameraCalibrationDataFromFile to make this instance ready.");
  }
}

void CameraDriver::readCameraCalibrationDataFromFile(){
  printToConsole("CameraDriver::readCameraCalibrationDataFromFile called.");
  cv::FileStorage fs("intrinsics.xml", cv::FileStorage::READ);
  fs["camera_matrix"] >> intrinsicMatrix;
  fs["distortion_coefficients"] >> distortionCoefficients;
  ready = true;
}

void CameraDriver::undistortImage(cv::Mat& source, cv::Mat& destination){
  if(ready){
    printToConsole("CameraDriver::undistortImage called.");
    cv::undistort(source, destination, intrinsicMatrix, distortionCoefficients);
  }else{
    printToConsole("CameraDriver::undistortImage called, but intrinsic matrix and distortion coefficients are not populated, returning the undistorted image now.");
    destination = source;
  }
}

bool CameraDriver::recordIsCurrent(){
  printToConsole("CameraDriver::newRecordIsCurrent called.");
  return newRecordIsCurrent;
}

void CameraDriver::receiveRecord(std::promise<MovableTimestampedType<PositionServiceRecord>> &&recordPromise){
  printToConsole("CameraDriver::receiveRecord is called.");
  std::unique_lock<std::mutex> lock(newRecordProtection);
  while(currentState == freezed){
    condition.wait(lock); 
  }
  while(!newRecordIsCurrent){
    condition.wait(lock); 
  }
  recordPromise.set_value(newRecord);
  lock.unlock();
  condition.notify_one();
  newRecordIsCurrent = false;
  printToConsole("CameraDriver::receiveRecord send a record and newRecord is not longer current.");
}

MovableTimestampedType<PositionServiceRecord> CameraDriver::getRecord(){
  printToConsole("CameraDriver::getRecord is called.");
  std::unique_lock<std::mutex> lock(newRecordProtection);
  while(currentState == freezed){
    condition.wait(lock); 
  }
  while(!newRecordIsCurrent){
    condition.wait(lock); 
  }
  MovableTimestampedType<PositionServiceRecord> movableTimestampedRecord = std::move(newRecord);
  lock.unlock();
  condition.notify_one();
  newRecordIsCurrent = false;
  printToConsole("CameraDriver::getRecord returned a record and newRecord is not longer current.");
  return movableTimestampedRecord;
}
  
void CameraDriver::setNewRecord(cv::Mat &&rawImage){
  printToConsole("CameraDriver::setNewRecord is called.");
  cv::Mat undistortedImage;
  undistortImage(rawImage, undistortedImage);
  PositionServiceRecord record;
  record.rawImage = std::move(rawImage);
  record.undistortedImage = std::move(undistortedImage);
  MovableTimestampedType<PositionServiceRecord> movableTimestampedRecord(record, debugLevel);
  std::unique_lock<std::mutex> lock(newRecordProtection);
  newRecord = std::move(movableTimestampedRecord);
  lock.unlock();
  newRecordIsCurrent = true;
}

void CameraDriver::runInRunningState(){
  printToConsole("CameraDriver::runInRunningState is called.");
    cv::Mat rawImage = cv::imread("test/test01.jpg" ,cv::IMREAD_COLOR); // source an image
    setNewRecord(std::move(rawImage)); // undistort and provide to clients, the main responsibility of the camera driver
}

void CameraDriver::runInInitializingState(){
  printToConsole("CameraDriver::runInInitializingState is called.");
  if(!ready){ // Camera is not calibrated, hence we must read from file
    readCameraCalibrationDataFromFile();
  }
}

void CameraDriver::runInFreezedState(){
  printToConsole("CameraDriver::runInFreezedState is called. Blocking changes to attributes now.");
}

void CameraDriver::runInTerminatedState(){
  printToConsole("CameraDriver::runInTerminatedState is called. Record is not longer updated.");
  newRecordIsCurrent = false;
}

void CameraDriver::manageStateSwitches(){
  while(true){
    if(currentState == initializing){
      printToConsole("CameraDriver::manageStateSwitches is called, instance is waiting in state initializing.");
      runInInitializingState();
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
    if(currentState == running){
      printToConsole("CameraDriver::manageStateSwitches, instance is in state running.");
      runInRunningState();
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
    if(currentState == terminated){
      printToConsole("CameraDriver::manageStateSwitches is called, instance has reached state terminated.");
      runInTerminatedState();
      break;
    }
    if(currentState == freezed){
      printToConsole("CameraDriver::manageStateSwitches is called, instance is waiting in state freezed.");
      runInFreezedState();
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
  }
}