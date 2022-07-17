#include "PositionServer.h"

PositionServer::PositionServer(){
  std::shared_ptr<CameraServer> sharedPointerToCameraServer(new CameraServer(debugLevel)); // create an instance + shared pointer to a CamerDriver, later moving this to the member variable.
  accessCameraServer = std::move(sharedPointerToCameraServer);
  std::shared_ptr<ImageTransformer> sharedPointerToImageTransformer(new ImageTransformer(debugLevel)); // create an instace + shared pointer to an ImageTransformer, later moving this to the member variable.
  accessImageTransformer = std::move(sharedPointerToImageTransformer);
  std::shared_ptr<PositionEstimator> sharedPointerToPositionEstimator(new PositionEstimator(debugLevel)); // create an instace + shared pointer to an ImageTransformer, later moving this to the member variable.
  accessPositionEstimator = std::move(sharedPointerToPositionEstimator);
  imposeSleepForMillisecondsOnResources(sleepForMilliseconds);
  accessImageTransformer->mountCameraServer(accessCameraServer);
  accessPositionEstimator->mountImageTransformer(accessImageTransformer);
}

PositionServer::PositionServer(Debuglevel positionServerDebuglevel){
  debugLevel = positionServerDebuglevel;
  printToConsole("PositionServer::PositionServer called.");
  std::shared_ptr<CameraServer> sharedPointerToCameraServer(new CameraServer(debugLevel)); // create an instance + shared pointer to a CamerDriver, later moving this to the member variable.
  accessCameraServer = std::move(sharedPointerToCameraServer);
  std::shared_ptr<ImageTransformer> sharedPointerToImageTransformer(new ImageTransformer(debugLevel)); // create an instace + shared pointer to an ImageTransformer, later moving this to the member variable.
  accessImageTransformer = std::move(sharedPointerToImageTransformer);
  std::shared_ptr<PositionEstimator> sharedPointerToPositionEstimator(new PositionEstimator(debugLevel)); // create an instace + shared pointer to an ImageTransformer, later moving this to the member variable.
  accessPositionEstimator = std::move(sharedPointerToPositionEstimator);
  imposeSleepForMillisecondsOnResources(sleepForMilliseconds);
  accessImageTransformer->mountCameraServer(accessCameraServer);
  accessPositionEstimator->mountImageTransformer(accessImageTransformer);
}
   
void PositionServer::run(){
  printToConsole("PositionServer::run called. Note: This the the main thread controlling servers & drivers.");
  if(ready){
    currentState = State::running;
    accessCameraServer->setCurrentState(currentState);
    std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    accessImageTransformer->setCurrentState(currentState);
    std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    accessPositionEstimator->setCurrentState(currentState);
  } else {
    printToConsole("PositionServer::run called but service is not ready yet.");
  }
}

void PositionServer::initialize(){
  printToConsole("PositionServer::initialize called. Note: This the the main thread controlling servers & drivers.");
  currentState = State::initializing;
  accessImageTransformer->setCurrentState(currentState);
  accessCameraServer->setCurrentState(currentState);
  accessPositionEstimator->setCurrentState(currentState);
  
  //std::async(std::launch::async, &CameraDriver::run, accessCameraDriver); // starting :CameraDriver's thread, per definition this is a call to its "run()".
  //std::async(std::launch::async, &ImageTransformer::run, accessImageTransformer); // starting :ImageTransformer's thread, per definition this is a call to its "run()".
  threads.emplace_back(std::thread(&CameraServer::run, accessCameraServer)); // starting :CameraDriver's thread, per definition this is a call to its "run()".
  threads.emplace_back(std::thread(&ImageTransformer::run, accessImageTransformer));// starting :ImageTransformer's thread, per definition this is a call to its "run()".
  threads.emplace_back(std::thread(&PositionEstimator::run, accessPositionEstimator));// starting :PositionEstimator's thread, per definition this is a call to its "run()".
  ready = false;
  while(!ready){
     printToConsole("PositionServer::initialize called: Waiting for resources to be ready.");
     ready = ((accessCameraServer->isReady() && accessImageTransformer->isReady()) && accessPositionEstimator->isReady());
     std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
  }
}

void PositionServer::terminate(){
  printToConsole("PositionServer::terminate called. Note: This the the main thread controlling servers & drivers.");
  currentState = State::terminated;
  accessPositionEstimator->setCurrentState(currentState);
  std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
  accessImageTransformer->setCurrentState(currentState);
  std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
  accessCameraServer->setCurrentState(currentState);
}

void PositionServer::freeze(){
  printToConsole("PositionServer::freeze called.");
  currentState = State::freezed;
  accessCameraServer->setCurrentState(currentState);
  accessImageTransformer->setCurrentState(currentState);
  accessPositionEstimator->setCurrentState(currentState);
}

void PositionServer::getRecord(PositionServiceRecord& record){
}
    
void PositionServer::getPosition(Position& position){
}
    
void PositionServer::runCameraCalibration(bool saveResults){
  printToConsole("PositionServer::runCameraCalibration called.");
  if (currentState == State::terminated){
    accessCameraServer->calibrate();
    if(saveResults){
      accessCameraServer->writeCameraCalibrationDataToFile(); 
    }
  } else {
    printToConsole("PositionServer::runCameraCalibration called but is rejected: This request is available only in terminated state.");
  }
}
      
void PositionServer::imposeSleepForMillisecondsOnResources(unsigned int time){
  std::string message = "PositionServer::imposeSleepForMillisecondsOnResources called, setting cycle time in drivers & servers to " + std::to_string(time);
  printToConsole(message);
  accessImageTransformer->setSleepForMilliseconds(time);
  accessCameraServer->setSleepForMilliseconds(time);
  accessPositionEstimator->setSleepForMilliseconds(time);
}