#include "RecordServer.h"

RecordServer::RecordServer() : isCurrent(false) {
  printToConsole("RecordServer::RecordServer called.");
}

void RecordServer::sendRecord(std::promise<MovableTimestampedType<PositionServiceRecord>> &&briefcase){
  printToConsole("RecordServer::sendRecord called.");
  std::unique_lock<std::mutex> uniqueLock(protection);
  while(!isCurrent){
    printToConsole("RecordServer::sendRecord is waiting for record to be updated.");
    condition.wait(uniqueLock);
  }
  briefcase.set_value(std::move(record));
  uniqueLock.unlock();
  condition.notify_one();
  isCurrent = false;
}

MovableTimestampedType<PositionServiceRecord> RecordServer::getRecord(){
  printToConsole("RecordServer::getRecord called.");
  MovableTimestampedType<PositionServiceRecord> value;
  std::unique_lock<std::mutex> uniqueLock(protection);
  while(!isCurrent){
    condition.wait(uniqueLock);
  }
  value = std::move(record);
  uniqueLock.unlock();
  condition.notify_one();
  isCurrent = false;
  return value;
}

void RecordServer::stopService(){
  printToConsole("RecordServer::stopService called.");
  condition.notify_all();
}


