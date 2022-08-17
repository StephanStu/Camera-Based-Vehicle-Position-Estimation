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
A record server holds a record of type MovableTimestampedType<PositionServiceRecord> in a thread safe manner in a member variable. It gives thread-safe access to the variable if needed, both simple "adding" and "getting" as well as "sending via promise-future" is offered to clients of this server.
*/

class RecordServer : public RunnableEntity {
  public:
    // constructor / desctructor
    RecordServer();
    // other
    void sendRecord(std::promise<MovableTimestampedType<PositionServiceRecord>> &&briefcase); // sends the record with a promise-future-instance
    MovableTimestampedType<PositionServiceRecord> getRecord(); // gets the record from the queue via return value; this is useful when testing childs of the RecordServer
  protected:
    void stopService(); // notifies all to ensure no client is waiting forever to be served with current records
    std::mutex protection; // this mutex protects the record when clients want to access in a simultaneous fashion
    std::condition_variable condition; // condition varibale is needed to notify clients
    MovableTimestampedType<PositionServiceRecord> record; // queue is the implementation for holding the records
    bool isCurrent; // this is true if the record is current and this is false when the record is consumed by a client
    bool recordUpdated; // this is true if updating the record is finished and then breaks the wait in sending-method and getting-method
};

#endif