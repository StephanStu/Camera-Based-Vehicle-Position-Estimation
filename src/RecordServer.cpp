#include "RecordServer.h"

RecordServer::RecordServer() : isCurrent(false) {
  printToConsole("RecordServer::RecordServer called.");
}

void RecordServer::sendRecord(std::promise<MovableTimestampedType<PositionServiceRecord>> &&briefcase){
  printToConsole("RecordServer::sendRecord called.");
  std::unique_lock<std::mutex> uniqueLock(protection);
  condition.wait(uniqueLock, [this] { return !isCurrent; }); 
  briefcase.set_value(std::move(record));
  uniqueLock.unlock();
  condition.notify_one();
  isCurrent = false;
}

MovableTimestampedType<PositionServiceRecord> RecordServer::getRecord(){
  printToConsole("RecordServer::getRecord called.");
  MovableTimestampedType<PositionServiceRecord> value;
  std::unique_lock<std::mutex> uniqueLock(protection);
  condition.wait(uniqueLock, [this] { return !isCurrent; }); 
  value = std::move(record);
  uniqueLock.unlock();
  condition.notify_one();
  isCurrent = false;
  return value;
}



