#include "PositionServer.h"

PositionServer::PositionServer() : lanePositionUpdateFinished(false) {
  debugLevel = Debuglevel::none;
  printToConsole("PositionServer::PositionServer called.");
}

PositionServer::PositionServer(Debuglevel positionServerDebuglevel) : lanePositionUpdateFinished(false) {
  debugLevel = positionServerDebuglevel;
  printToConsole("PositionServer::PositionServer called.");
}

void PositionServer::run(){
  printToConsole("PositionServer::run called.");
  threads.emplace_back(std::thread(&PositionServer::manageStateSwitches, this));
} 

void PositionServer::mountImageTransformer(std::shared_ptr<ImageTransformer> pointerToImageTransformer){
  printToConsole("PositionServer::mountImageTransformer called, instance of ImageTransformer is mounted via saving the shared pointer.");
  accessImageTransformer = pointerToImageTransformer;
  ready = true;
}

MovableTimestampedType<PositionServiceRecord> PositionServer::receiveRecordFromMountedImageTransformer(){
  printToConsole("PositionServer::getRecordFromMountedCameraDriver called, pulling a record with promise-future-mechanism from mounted camera driver.");
  std::promise<MovableTimestampedType<PositionServiceRecord>> prms;
  std::future<MovableTimestampedType<PositionServiceRecord>> ftr = prms.get_future();
  std::thread t(&ImageTransformer::receiveRecord, accessImageTransformer, std::move(prms));
  printToConsole("PositionServer::getRecordFromMountedCameraDriver started thread.");
  t.join();
  MovableTimestampedType<PositionServiceRecord> movableTimestampedRecord = ftr.get();
  printToConsole("PositionServer::getRecordFromMountedCameraDriver finished thread.");
  return movableTimestampedRecord;
}

void PositionServer::runInRunningState(){
  printToConsole("PositionServer::runInRunningState is called.");
  lanePositionUpdateFinished = false;
  /*Do the filter update here: Pull from queue, evalualte points on image, comoute distances,...*/
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  MovableTimestampedType<PositionServiceRecord> movableTimestampedRecord = receiveRecordFromMountedImageTransformer();
  lanePositionUpdateFinished = true;
  printToConsole("PositionServer::runInRunningState finished update of the state estimator.");
}

void PositionServer::runInInitializingState(){
  printToConsole("PositionServer::runInInitializingState is called.");
}

void PositionServer::runInFreezedState(){
  printToConsole("PositionServer::runInFreezedState is called.");
}

void PositionServer::runInTerminatedState(){
  printToConsole("PositionServer::runInTerminatedState is called.");
}

void PositionServer::manageStateSwitches(){
  while(true){
    if(currentState == initializing){
      printToConsole("PositionServer::manageStateSwitches is called, instance is waiting in state initializing.");
      runInInitializingState();
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
    if(currentState == running){
      printToConsole("PositionServer::manageStateSwitches, instance is in state running.");
      runInRunningState();
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
    if(currentState == terminated){
      printToConsole("PositionServer::manageStateSwitches is called, instance has reached state terminated.");
      runInTerminatedState();
      break;
    }
    if(currentState == freezed){
      printToConsole("PositionServer::manageStateSwitches is called, instance is waiting in state freezed.");
      runInFreezedState();
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
  }
}

void PositionServer::receiveRecord(std::promise<PositionServiceRecord> &&recordPromise){
  printToConsole("PositionServer::receiveRecord called.");
  std::unique_lock<std::mutex> lock(recordProtection);
  while(!lanePositionUpdateFinished){
    condition.wait(lock); 
  }
  recordPromise.set_value(currentRecord);
  lock.unlock();
  condition.notify_one();
  printToConsole("PositionServer::receiveRecord called finished; returning the most current record to requester.");  
}

void PositionServer::receiveLanePosition(std::promise<LanePosition> &&lanePositionPromise){
  printToConsole("PositionServer::receiveRecord called.");
  std::unique_lock<std::mutex> lock(recordProtection);
  while(!lanePositionUpdateFinished){
    condition.wait(lock); 
  }
  LanePosition lanePosition;
  lanePosition.angle = currentRecord.angle;
  lanePosition.deviation = currentRecord.deviation;
  lanePositionPromise.set_value(lanePosition);
  lock.unlock();
  condition.notify_one();
  printToConsole("PositionServer::receiveRecord called finished; returning the most current record to requester."); 
}