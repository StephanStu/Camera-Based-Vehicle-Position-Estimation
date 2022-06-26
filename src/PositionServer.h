#ifndef POSITIONSERVER_H
#define POSITIONSERVER_H

#include <future>
#include <mutex>
#include "RunnableEntity.h"
#include "MovableTimestampedType.h"
#include "ImageTransformer.h"
#include "Types.h"

/*
PositionServer:
*/

class PositionServer : public RunnableEntity {
  public:
    // constructor / desctructor
    PositionServer();
    PositionServer(Debuglevel positionServerDebuglevel);
    // other
    void run(); // overrides virtual function "run" in base class
  private:
    void manageStateSwitches(); // this method manages switchs in States =  {initializing, running, freezed, terminated} and calls the appropriate methdos
    void runInRunningState(); // this method is called when State = runningand runs a kalman-filter to estimate the vehicle's distance to center line based on transformed images pulled from :ImageTransformer.
    void runInInitializingState(); // this method is called when State = initializing
    void runInFreezedState(); // this method is called when State = freezed
    void runInTerminatedState(); // this method is called when State = terminated
    MovableTimestampedType<PositionServiceRecord> receiveRecordFromMountedImageTransformer(); // get a record from the ImageTransformer-Instance that is mounted to this 
    void mountImageTransformer(std::shared_ptr<ImageTransformer> pointerToImageTransformer); // "mount" an instance of ImageTransformer via pasing a shared point, s.t. methods can access the ImageTransformer
    void receiveRecord(std::promise<PositionServiceRecord> &&recordPromise); //  receive a record via promise-future and pops the record just send from the queue
    void receiveLanePosition(std::promise<LanePosition> &&lanePositionPromise); //  receive a lane position via promise-future and pops the record just send from the queue
    std::shared_ptr<ImageTransformer> accessImageTransformer; // shared pointer to an instance of CameraDriver delivering images upond request
    PositionServiceRecord currentRecord; // the latest / most current, completed record that the position server protects in the sense of thread-save modification
    std::mutex recordProtection; // this mutex is protecing access to the currentRecord
    std::condition_variable condition; // condition varibale is needed to notify clients
    bool lanePositionUpdateFinished; // this must be false if the lane position is not current and/or an update is currently running
    friend class PositionService;
};

#endif