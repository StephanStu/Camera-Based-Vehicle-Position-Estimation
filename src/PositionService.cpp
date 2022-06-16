#include "PositionService.h"

PositionService::PositionService(){
  debugLevel = Debuglevel::none;
  printToConsole("PositionService::PositionService called.");
  currentState = State::terminated;
  std::shared_ptr<CameraDriver> sharedPointerToCameraDriver(new CameraDriver(debugLevel)); // create an instance + shared pointer to a CamerDriver, later moving this to the member variable.
  accessCameraDriver = std::move(sharedPointerToCameraDriver);
  std::shared_ptr<ImageTransformer> sharedPointerToImageTransformer(new ImageTransformer(debugLevel)); // create an instace + shared pointer to an ImageTransformer, later moving this to the member variable.
  accessImageTransformer = std::move(sharedPointerToImageTransformer);
  std::shared_ptr<PositionServer> sharedPointerToPositionServer(new PositionServer(debugLevel)); // create an instace + shared pointer to an ImageTransformer, later moving this to the member variable.
  accessPositionServer = std::move(sharedPointerToPositionServer);
  imposeStateOnResources(State::terminated);
}

PositionService::PositionService(Debuglevel positionEstimationServiceDebuglevel){
  debugLevel = positionEstimationServiceDebuglevel;
  printToConsole("PositionService::PositionService called.");
  currentState = State::terminated;
  std::shared_ptr<CameraDriver> sharedPointerToCameraDriver(new CameraDriver(debugLevel)); // create an instance + shared pointer to a CamerDriver, later moving this to the member variable.
  accessCameraDriver = std::move(sharedPointerToCameraDriver);
  std::shared_ptr<ImageTransformer> sharedPointerToImageTransformer(new ImageTransformer(debugLevel)); // create an instace + shared pointer to an ImageTransformer, later moving this to the member variable.
  accessImageTransformer = std::move(sharedPointerToImageTransformer);
  std::shared_ptr<PositionServer> sharedPointerToPositionServer(new PositionServer(debugLevel)); // create an instace + shared pointer to an ImageTransformer, later moving this to the member variable.
  accessPositionServer = std::move(sharedPointerToPositionServer);
  imposeStateOnResources(State::terminated);
}

void PositionService::run(){
  printToConsole("PositionService::run called. Note: This the the main thread controlling servers & drivers.");
  imposeStateOnResources(State::running);
}

void PositionService::initialize(){
  printToConsole("PositionService::initialize called. Note: This the the main thread controlling servers & drivers.");
  imposeStateOnResources(State::initializing);
  imposeSleepForMillisecondsOnResources(50);
  accessImageTransformer->mountCamerDriver(accessCameraDriver);
  //accessCameraDriver->calibrate();
  accessCameraDriver->isReady();
  //accessCameraDriver->writeCameraCalibrationDataToFile();
  
  accessCameraDriver->readCameraCalibrationDataFromFile();
  accessCameraDriver->isReady();
  
  //std::async(std::launch::async, &CameraDriver::run, accessCameraDriver); // starting :CameraDriver's thread, per definition this is a call to its "run()".
  //std::async(std::launch::async, &ImageTransformer::run, accessImageTransformer); // starting :ImageTransformer's thread, per definition this is a call to its "run()".
  threads.emplace_back(std::thread(&CameraDriver::run, accessCameraDriver)); // starting :CameraDriver's thread, per definition this is a call to its "run()".
  threads.emplace_back(std::thread(&ImageTransformer::run, accessImageTransformer));// starting :ImageTransformer's thread, per definition this is a call to its "run()".
}

void PositionService::terminate(){
  printToConsole("PositionService::terminate called. Note: This the the main thread controlling servers & drivers.");
  imposeStateOnResources(State::terminated);
}

void PositionService::imposeStateOnResources(State targetState){
  printToConsole("PositionService::imposeStateOnResources called.");
  accessPositionServer->setCurrentState(targetState);
  accessImageTransformer->setCurrentState(targetState);
  accessCameraDriver->setCurrentState(targetState);
}

void PositionService::imposeSleepForMillisecondsOnResources(unsigned int time){
  std::string message = "PositionService::imposeSleepForMillisecondsOnResources called, setting cycle time in drivers & servers to " + std::to_string(time);
  printToConsole(message);
  accessPositionServer->setSleepForMilliseconds(time);
  accessImageTransformer->setSleepForMilliseconds(time);
  accessCameraDriver->setSleepForMilliseconds(time);
}