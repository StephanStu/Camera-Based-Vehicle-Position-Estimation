#include "PositionService.h"

PositionService::PositionService(){
  debugLevel = Debuglevel::none;
  currentState = State::terminated;
  std::shared_ptr<CameraDriver> sharedPointerToCameraDriver(new CameraDriver(debugLevel)); // create an instance + shared pointer to a CamerDriver, later moving this to the member variable.
  accessCameraDriver = std::move(sharedPointerToCameraDriver);
  std::shared_ptr<ImageTransformer> sharedPointerToImageTransformer(new ImageTransformer(debugLevel)); // create an instace + shared pointer to an ImageTransformer, later moving this to the member variable.
  accessImageTransformer = std::move(sharedPointerToImageTransformer);
  std::shared_ptr<PositionEstimator> sharedPointerToPositionEstimator(new PositionEstimator(debugLevel)); // create an instace + shared pointer to an ImageTransformer, later moving this to the member variable.
  accessPositionEstimator = std::move(sharedPointerToPositionEstimator);
  imposeSleepForMillisecondsOnResources(sleepForMilliseconds);
  accessImageTransformer->mountCamerDriver(accessCameraDriver);
  accessPositionEstimator->mountImageTransformer(accessImageTransformer);
}

PositionService::PositionService(Debuglevel positionEstimationServiceDebuglevel) {
  debugLevel = positionEstimationServiceDebuglevel;
  printToConsole("PositionService::PositionService called.");
  currentState = State::terminated;
  std::shared_ptr<CameraDriver> sharedPointerToCameraDriver(new CameraDriver(debugLevel)); // create an instance + shared pointer to a CamerDriver, later moving this to the member variable.
  accessCameraDriver = std::move(sharedPointerToCameraDriver);
  std::shared_ptr<ImageTransformer> sharedPointerToImageTransformer(new ImageTransformer(debugLevel)); // create an instace + shared pointer to an ImageTransformer, later moving this to the member variable.
  accessImageTransformer = std::move(sharedPointerToImageTransformer);
  std::shared_ptr<PositionEstimator> sharedPointerToPositionEstimator(new PositionEstimator(debugLevel)); // create an instace + shared pointer to an ImageTransformer, later moving this to the member variable.
  accessPositionEstimator = std::move(sharedPointerToPositionEstimator);
  imposeSleepForMillisecondsOnResources(sleepForMilliseconds);
  accessImageTransformer->mountCamerDriver(accessCameraDriver);
  accessPositionEstimator->mountImageTransformer(accessImageTransformer);
}

void PositionService::run(){
  printToConsole("PositionService::run called. Note: This the the main thread controlling servers & drivers.");
  if(ready){
    currentState = State::running;
    accessCameraDriver->setCurrentState(currentState);
    std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    accessImageTransformer->setCurrentState(currentState);
    std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    accessPositionEstimator->setCurrentState(currentState);
  } else {
    printToConsole("PositionService::run called but service is not ready yet.");
  }
}

void PositionService::initialize(){
  printToConsole("PositionService::initialize called. Note: This the the main thread controlling servers & drivers.");
  currentState = State::initializing;
  accessImageTransformer->setCurrentState(currentState);
  accessCameraDriver->setCurrentState(currentState);
  accessPositionEstimator->setCurrentState(currentState);
  
  //std::async(std::launch::async, &CameraDriver::run, accessCameraDriver); // starting :CameraDriver's thread, per definition this is a call to its "run()".
  //std::async(std::launch::async, &ImageTransformer::run, accessImageTransformer); // starting :ImageTransformer's thread, per definition this is a call to its "run()".
  threads.emplace_back(std::thread(&CameraDriver::run, accessCameraDriver)); // starting :CameraDriver's thread, per definition this is a call to its "run()".
  threads.emplace_back(std::thread(&ImageTransformer::run, accessImageTransformer));// starting :ImageTransformer's thread, per definition this is a call to its "run()".
  threads.emplace_back(std::thread(&PositionEstimator::run, accessPositionEstimator));// starting :PositionEstimator's thread, per definition this is a call to its "run()".
  ready = false;
  while(!ready){
     printToConsole("PositionService::initialize called: Waiting for resources to be ready.");
     ready = ((accessCameraDriver->isReady() && accessImageTransformer->isReady()) && accessPositionEstimator->isReady());
     std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
  }
}

void PositionService::terminate(){
  printToConsole("PositionService::terminate called. Note: This the the main thread controlling servers & drivers.");
  currentState = State::terminated;
  accessPositionEstimator->setCurrentState(currentState);
  std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
  accessImageTransformer->setCurrentState(currentState);
  std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
  accessCameraDriver->setCurrentState(currentState);
}

void PositionService::freeze(){
  printToConsole("PositionService::freeze called.");
  currentState = State::freezed;
  accessCameraDriver->setCurrentState(currentState);
  accessImageTransformer->setCurrentState(currentState);
  accessPositionEstimator->setCurrentState(currentState);
}

void PositionService::imposeSleepForMillisecondsOnResources(unsigned int time){
  std::string message = "PositionService::imposeSleepForMillisecondsOnResources called, setting cycle time in drivers & servers to " + std::to_string(time);
  printToConsole(message);
  accessImageTransformer->setSleepForMilliseconds(time);
  accessCameraDriver->setSleepForMilliseconds(time);
  accessPositionEstimator->setSleepForMilliseconds(time);
}

void PositionService::runCameraCalibration(bool saveResults){
  printToConsole("PositionService::runCameraCalibration called.");
  if (currentState == State::terminated){
    accessCameraDriver->calibrate();
    if(saveResults){
      accessCameraDriver->writeCameraCalibrationDataToFile(); 
    }
  } else {
    printToConsole("PositionService::runCameraCalibration called but is rejected: This request is available only in terminated state.");
  }
}

void PositionService::getRecord(PositionServiceRecord& record){
  printToConsole("PositionService::getRecord called.");
  record = accessPositionEstimator->record;
}