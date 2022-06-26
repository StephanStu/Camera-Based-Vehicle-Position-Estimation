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
  imposeStateOnResources(currentState);
  imposeSleepForMillisecondsOnResources(sleepForMilliseconds);
  accessImageTransformer->mountCamerDriver(accessCameraDriver);
  accessPositionServer->mountImageTransformer(accessImageTransformer);
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
  imposeStateOnResources(currentState);
  imposeSleepForMillisecondsOnResources(sleepForMilliseconds);
  accessImageTransformer->mountCamerDriver(accessCameraDriver);
  accessPositionServer->mountImageTransformer(accessImageTransformer);
}

void PositionService::run(){
  printToConsole("PositionService::run called. Note: This the the main thread controlling servers & drivers.");
  currentState = State::running;
  imposeStateOnResources(currentState);
}

void PositionService::initialize(){
  printToConsole("PositionService::initialize called. Note: This the the main thread controlling servers & drivers.");
  currentState = State::initializing;
  imposeStateOnResources(currentState);
  //std::async(std::launch::async, &CameraDriver::run, accessCameraDriver); // starting :CameraDriver's thread, per definition this is a call to its "run()".
  //std::async(std::launch::async, &ImageTransformer::run, accessImageTransformer); // starting :ImageTransformer's thread, per definition this is a call to its "run()".
  threads.emplace_back(std::thread(&CameraDriver::run, accessCameraDriver)); // starting :CameraDriver's thread, per definition this is a call to its "run()".
  threads.emplace_back(std::thread(&ImageTransformer::run, accessImageTransformer));// starting :ImageTransformer's thread, per definition this is a call to its "run()".
  threads.emplace_back(std::thread(&PositionServer::run, accessPositionServer));// starting :PositionServer's thread, per definition this is a call to its "run()".
  ready = false;
  while(!ready){
     printToConsole("PositionService::initialize called: Waiting for resources to be ready.");
     ready = ((accessCameraDriver->isReady() && accessImageTransformer->isReady()) && accessPositionServer->isReady());
     std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
  }
}

void PositionService::terminate(){
  printToConsole("PositionService::terminate called. Note: This the the main thread controlling servers & drivers.");
  currentState = State::terminated;
  imposeStateOnResources(currentState);
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

LanePosition PositionService::getLanePosition(){
  printToConsole("PositionService::getPositionServiceRecord called.");
  LanePosition position;
  position.angle = 0.0;
  position.deviation = 0.0;
  if (currentState == State::running){
    // run the filter to return the state estimate
  } else {
    printToConsole("PositionService::getPositionServiceRecord called but is rejected: This request is available only in running state.");
  }
  return position;
}

PositionServiceRecord PositionService::getPositionServiceRecord(){
  printToConsole("PositionService::getPositionServiceRecord called.");
  PositionServiceRecord record;
  if(currentState == State::running){ 
    // run the filter to return the full record including state and measurements
  } else {
    printToConsole("PositionService::getPositionServiceRecord called but is rejected: This request is available only in running state.");
  }
  return record;
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