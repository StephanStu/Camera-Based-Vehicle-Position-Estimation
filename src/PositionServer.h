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
This software component is responsible to schedule execution of the application end:end. It must launch the threads needed and control the execution by switching the modes as requested by external interfaces "run()", "initialize()", "freeze()" and "terminate()". This software component coordinates launching and killing threads in e.g. the PositionEstimator. The user uses only the external interfaces and should not worry about how to use the service in detail. This services can be created and used as follows:

  std::unique_ptr<PositionServer> srv(new PositionServer(Debuglevel::verbose)); // create it
  
  std::shared_ptr<VelocitySource> accessVelocitySource(new VelocitySource(MEANVELOCITY, VARIANCEVELOCITY)); // create the velocity-source and mount it
  srv->mountVelocitySource(accessVelocitySource);
  
  std::shared_ptr<ImageSource> accessImageSource(new ImageSource(fileName)); // create the image source and mount it
  srv->mountImageSource(accessImageSource);
  
  // use the external interface to initialize
  srv->initialize();
  // you may wait a while...
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  // use the external interface to run
  srv->run();
  std::this_thread::sleep_for(std::chrono::milliseconds(RUNTIME));
  // use the external interface to stop
  srv->terminate();
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