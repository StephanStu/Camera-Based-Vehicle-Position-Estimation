#ifndef POSITIONSERVER_H
#define POSITIONSERVER_H

#include <memory>
#include "CameraServer.h"
#include "ImageTransformer.h"
#include "PositionEstimator.h"
#include "RunnableEntity.h"
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
    void run(); // overrides virtual function "run" in base class; implements a launch sequence for the components
    void initialize(); // initializing routine must be called before run
    void terminate(); // routine terminating drivers & servers; implements a termination sequence for the components
    void freeze(); // imposes the freezed state, resources stop to manipulate the attributes now
    void runCameraCalibration(bool saveResults); // runs camera calibration and saves the results to the disc if the argument is true
    void mountImageSource(std::shared_ptr<ImageSource> pointerToImageSource); // "mounts" the instance of ImageSource to the instance of CameraDriver by saving the shared pointer in the member variables
    void mountVelocitySource(std::shared_ptr<VelocitySource> pointerToVelocitySource); // "mounts" the instance of ImageSource to the instance of CameraDriver by saving the shared pointer in the member variables
  private:
    std::shared_ptr<CameraServer> accessCameraServer; // an instance (+ shared pointer to access the) camera driver, which is managed by the PositionEstimationService
    std::shared_ptr<ImageTransformer> accessImageTransformer; // an instance (+ shared pointer to access the) image transformer, which is managed by the PositionEstimationService
    std::shared_ptr<PositionEstimator> accessPositionEstimator; // an instance (+ shared pointer to access the) position server, which is managed by the PositionEstimationService
    void imposeSleepForMillisecondsOnResources(unsigned int time); // imposes the cycle time on all drivers & servers, makes the code more DRY
};

#endif