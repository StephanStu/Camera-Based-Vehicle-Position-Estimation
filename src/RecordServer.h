#ifndef RECORDSERVER_H
#define RECORDSERVER_H

#include <future>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include "RunnableEntity.h"
#include "Types.h"
#include "MovableTimestampedType.h"

/*
RecordServer:
A record server holds records of type std::deque<MovableTimestampedType<PositionServiceRecord>> in a thread safe manner in a member variable. It gives thread-safe access to the variable if needed, both simple "adding" and "getting" as well as "sending via promise-future" is offered to clients of this server.
*/

class RecordServer : public RunnableEntity {
  public:
    // constructor / desctructor
    RecordServer();
    // other
    void sendRecord(std::promise<MovableTimestampedType<PositionServiceRecord>> &&briefcase); // sends the record with a promise-future-instance
    MovableTimestampedType<PositionServiceRecord> getRecord(); // gets the record from the queue via return value; this is useful when testing childs of the RecordServer
  protected:
    std::mutex protection; // this mutex protects the record when clients want to access in a simultaneous fashion
    std::condition_variable condition; // condition varibale is needed to notify clients
    MovableTimestampedType<PositionServiceRecord> record; // queue is the implementation for holding the records
    bool isCurrent; // this is true if the record is current and this is false when the record is consumed by a client
};

#endif