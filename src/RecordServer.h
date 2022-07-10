#ifndef RECORDSERVER_H
#define RECORDERVER_H

#include <future>
#include <mutex>
#include <deque>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include "RunnableEntity.h"
#include "Types.h"
#include "MovableTimestampedType.h"

/*
RecordServer:
A record server holds records of type std::deque<MovableTimestampedType<PositionServiceRecord>> in a queue. It gives thread-safe access to the items if needed, both simple "adding" and "getting" as well as "receiving via promise-future" (as part of the position servive) is offered to clients of this server with public interfaces. This server is needed to queue the data that is produced & processed in the position service.
*/

class RecordServer : public RunnableEntity {
  public:
    // constructor / desctructor

    // other
    virtual void receiveRecord(std::promise<MovableTimestampedType<PositionServiceRecord>> &&recordPromise) = 0; // this method allows consumers to receive the latest record with a promise-future-instance
    unsigned int getQueueLength(); // returns the length of the queue of records to e.g. observe if the queue is growing and growing due to resource shortage or if it stays close to zero due to consumers constantly pulling images
    void addRecord(MovableTimestampedType<PositionServiceRecord> &&record); // adds a record to the queue
    virtual MovableTimestampedType<PositionServiceRecord> getRecord() = 0; // gets the record from the queue
  protected:
    void clearQueue(); // clears the queue of records (to free memory!)
    void popOneRecord(); // remove one (!) record from the queue, the oldest one.
    std::mutex queueProtection; // this mutex is shared by all objects for protecting access to the queues
    std::condition_variable condition; // condition varibale is needed to notify clients
    std::deque<MovableTimestampedType<PositionServiceRecord>> queue; // queue is the implementation for holding the records
};

#endif