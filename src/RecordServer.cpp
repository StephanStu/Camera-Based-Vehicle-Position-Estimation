#include "RecordServer.h"
#include <string>

void RecordServer::addRecord(MovableTimestampedType<PositionServiceRecord> &&record){
  std::unique_lock<std::mutex> lock(queueProtection);
  queue.push_back(std::move(record));
  lock.unlock();
  condition.notify_one(); 
  printToConsole("RecordServer::addRecordToQueue called; ingested a record.");
}

unsigned int RecordServer::getQueueLength(){
  unsigned int size = queue.size();
  std::string message = "RecordServer::getQueueOfRecordsLength called, queue holds " + std::to_string(size) + " items.";
  printToConsole(message);
  return size;
}

void RecordServer::clearQueue(){
  unsigned int size = queue.size();
  if(size > 0){
    std::string message = "RecordServer::clearQueue called, popping " + std::to_string(size) + " records.";
    printToConsole(message);
    std::unique_lock<std::mutex> lock(queueProtection);
    while(!queue.empty()){
      queue.pop_back();
    }
    lock.unlock();
    condition.notify_one(); 
  } else {
    std::string message = "RecordServer::clearQueue called but queue does not hold any records.";
    printToConsole(message);
  }
}

void RecordServer::popOneRecord(){
  printToConsole("RecordServer::popRecord called.");
  if(getQueueLength() > 0){
    std::unique_lock<std::mutex> lock(queueProtection);
    MovableTimestampedType<PositionServiceRecord> movableTimestampedRecord = std::move(queue.back());
    queue.pop_back();
    lock.unlock();
    condition.notify_one();
  } else {
    printToConsole("RecordServer::getRecord called but queue was empty: Nothing to pop here.");
  }
}