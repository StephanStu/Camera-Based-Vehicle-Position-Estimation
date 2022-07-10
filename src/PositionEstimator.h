#ifndef POSITIONESTIMATOR_H
#define POSITIONESTIMATOR_H

#include <future>
#include <mutex>
#include "RunnableEntity.h"
#include "MovableTimestampedType.h"
#include "ImageTransformer.h"
#include "Types.h"

/*
PositionEstimator:
*/

class PositionEstimator : public RunnableEntity {
  public:
    // constructor / desctructor
    PositionEstimator();
    PositionEstimator(Debuglevel positionEstimatorDebuglevel);
    // other
    void run(); // overrides virtual function "run" in base class
  private:
    friend class PositionService;
    void manageStateSwitches(); // this method manages switchs in States =  {initializing, running, freezed, terminated} and calls the appropriate methdos
    void runInRunningState(); // this method is called when State = runningand runs a kalman-filter to estimate the vehicle's distance to center line based on transformed images pulled from :ImageTransformer.
    void runInInitializingState(); // this method is called when State = initializing
    void runInFreezedState(); // this method is called when State = freezed
    void runInTerminatedState(); // this method is called when State = terminated
    MovableTimestampedType<PositionServiceRecord> receiveRecordFromMountedImageTransformer(); // get a record from the ImageTransformer-Instance that is mounted to this
    void updatePosition(); // this method updates the estimate of the position (member variable)
    void mountImageTransformer(std::shared_ptr<ImageTransformer> pointerToImageTransformer); // "mount" an instance of ImageTransformer via pasing a shared point, s.t. methods can access the ImageTransformer
    std::shared_ptr<ImageTransformer> accessImageTransformer; // shared pointer to an instance of CameraDriver delivering images upond request
    std::mutex recordProtection; // this mutex is protecing access to the currentRecord
    std::condition_variable condition; // condition varibale is needed to notify clients
    bool imageTransformerIsMounted; // this is true once an intance of ImageTransformer has been mounted
    Position position; // the most current estimate of the (lane-)position: angle and deviation from center
    PositionServiceRecord record; // the most current record inclduing the estimate of the (lane-)position: angle and deviation from center
};

#endif