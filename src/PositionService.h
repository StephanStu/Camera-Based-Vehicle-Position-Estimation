#ifndef POSITIONSERVICE_H
#define POSITIONSERVICE_H

#include <memory>
#include "ImageTransformer.h"
#include "CameraDriver.h"
#include "PositionEstimator.h"
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
    void run(); // overrides virtual function "run" in base class; implements a launch sequence for the components
    void initialize(); // initializing routine must be called before run
    void terminate(); // routine terminating drivers & servers; implements a termination sequence for the components
    void freeze(); // imposes the freezed state, resources stop to manipulate the attributes now
    void getRecord(PositionServiceRecord& record); // writes the PositionServiceRecord to the referred variable
    void runCameraCalibration(bool saveResults); // runs camera calibration and saves the results to the disc if the argument is true
  private:
    std::shared_ptr<CameraDriver> accessCameraDriver; // an instance (+ shared pointer to access the) camera driver, which is managed by the PositionEstimationService
    std::shared_ptr<ImageTransformer> accessImageTransformer; // an instance (+ shared pointer to access the) image transformer, which is managed by the PositionEstimationService
    std::shared_ptr<PositionEstimator> accessPositionEstimator; // an instance (+ shared pointer to access the) position server, which is managed by the PositionEstimationService
    void imposeSleepForMillisecondsOnResources(unsigned int time); // imposes the cycle time on all drivers & servers, makes the code more DRY
};

#endif