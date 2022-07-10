#include <iostream>
#include "PositionEstimator.h"

PositionEstimator::PositionEstimator(){
  debugLevel = Debuglevel::none;
  printToConsole("PositionEstimator::PositionServer called.");
}

PositionEstimator::PositionEstimator(Debuglevel positionEstimatorDebuglevel) {
  debugLevel = positionEstimatorDebuglevel;
  printToConsole("PositionEstimator::PositionServer called.");
}

void PositionEstimator::run(){
  printToConsole("PositionEstimator::run called.");
  threads.emplace_back(std::thread(&PositionEstimator::manageStateSwitches, this));
} 

void PositionEstimator::mountImageTransformer(std::shared_ptr<ImageTransformer> pointerToImageTransformer){
  printToConsole("PositionEstimator::mountImageTransformer called, instance of ImageTransformer is mounted via saving the shared pointer.");
  accessImageTransformer = pointerToImageTransformer;
  imageTransformerIsMounted = true;
}

MovableTimestampedType<PositionServiceRecord> PositionEstimator::receiveRecordFromMountedImageTransformer(){
  printToConsole("PositionEstimator::getRecordFromMountedCameraDriver called, pulling a record with promise-future-mechanism from mounted camera driver.");
  std::promise<MovableTimestampedType<PositionServiceRecord>> prms;
  std::future<MovableTimestampedType<PositionServiceRecord>> ftr = prms.get_future();
  std::thread t(&ImageTransformer::receiveRecord, accessImageTransformer, std::move(prms));
  printToConsole("PositionEstimator::getRecordFromMountedCameraDriver started thread.");
  t.join();
  MovableTimestampedType<PositionServiceRecord> movableTimestampedRecord = ftr.get();
  printToConsole("PositionEstimator::getRecordFromMountedCameraDriver finished thread.");
  return movableTimestampedRecord;
}

void PositionEstimator::updatePosition(){
  printToConsole("PositionEstimator::updatePosition is called.");
  std::promise<MovableTimestampedType<PositionServiceRecord>> prms;
  std::future<MovableTimestampedType<PositionServiceRecord>> ftr = prms.get_future();
  std::thread t(&ImageTransformer::receiveRecord, accessImageTransformer, std::move(prms));
  printToConsole("PositionEstimator::getRecordFromMountedCameraDriver started thread.");
  t.join();
  MovableTimestampedType<PositionServiceRecord> movableTimestampedRecord = ftr.get();
  record = movableTimestampedRecord.getData();
  std::cout << movableTimestampedRecord.getAge() << std::endl;
  position.angle = 0.0;
  position.deviation = 0.0;
  record.angle = 0.0;
  record.deviation = 0.0;
}

void PositionEstimator::runInRunningState(){
  printToConsole("PositionEstimator::runInRunningState is called.");
  /*Do the filter update here: Pull from queue, evalualte points on image, comoute distances,...*/
  //std::this_thread::sleep_for(std::chrono::milliseconds(100));
  //MovableTimestampedType<PositionServiceRecord> movableTimestampedRecord = receiveRecordFromMountedImageTransformer();
  updatePosition();
  std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
}

void PositionEstimator::runInInitializingState(){
  if(imageTransformerIsMounted){
    printToConsole("PositionEstimator::runInInitializingState is called; instance is ready to enter the running state.");
    ready = true;
  } else {
    printToConsole("PositionEstimator::runInInitializingState is called but image transformer is not mounted yet.");
  }
}

void PositionEstimator::runInFreezedState(){
  printToConsole("PositionEstimator::runInFreezedState is called. Blocking changes to attributes now.");
}

void PositionEstimator::runInTerminatedState(){
  printToConsole("PositionEstimator::runInTerminatedState is called.");
}

void PositionEstimator::manageStateSwitches(){
  while(true){
    if(currentState == initializing){
      printToConsole("PositionEstimator::manageStateSwitches is called, instance is waiting in state initializing.");
      runInInitializingState();
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
    if(currentState == running){
      printToConsole("PositionEstimator::manageStateSwitches, instance is in state running.");
      runInRunningState();
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
    if(currentState == terminated){
      printToConsole("PositionEstimator::manageStateSwitches is called, instance has reached state terminated.");
      runInTerminatedState();
      break;
    }
    if(currentState == freezed){
      printToConsole("PositionEstimator::manageStateSwitches is called, instance is waiting in state freezed.");
      runInFreezedState();
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
  }
}