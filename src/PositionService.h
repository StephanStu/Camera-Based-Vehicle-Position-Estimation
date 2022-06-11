#ifndef POSITIONSERVICE_H
#define POSITIONSERVICE_H

#include <memory>
#include "ImageTransformer.h"
#include "PositionServer.h"
#include "CameraDriver.h"
#include "RunnableEntity.h"
#include "Types.h"

/*
PositionService:
*/

class PositionService : public RunnableEntity {
  public:
    // constructor / desctructor
    PositionService();
    PositionService(Debuglevel positionEstimationServiceDebuglevel);
    // other
    void run(); // overrides virtual function "run" in base class
    void initialize(); // initializing routine must be called before run
    void terminate(); // routine terminating drivers & servers
  private:
    std::shared_ptr<CameraDriver> accessCameraDriver; // an instance (+ shared pointer to access the) camera driver, which is managed by the PositionEstimationService
    std::shared_ptr<ImageTransformer> accessImageTransformer; // an instance (+ shared pointer to access the) image transformer, which is managed by the PositionEstimationService
    std::shared_ptr<PositionServer> accessPositionServer; // an instance (+ shared pointer to access the) position server, which is managed by the PositionEstimationService
    void imposeStateOnResources(State targetState); // imposes the target state on all drivers & servers, makes the code more DRY
    void imposeSleepForMillisecondsOnResources(unsigned int time); // imposes the cycle time on all drivers & servers, makes the code more DRY
};

#endif