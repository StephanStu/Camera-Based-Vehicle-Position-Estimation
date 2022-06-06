#ifndef POSITIONESTIMATIONSERVICE_H
#define POSITIONESTIMATIONSERVICE_H

#include <memory>
#include "ImageTransformer.h"
#include "PositionServer.h"
#include "CameraDriver.h"
#include "RunnableEntity.h"
#include "Types.h"

/*
PositionEstimationService:
*/

class PositionEstimationService : public RunnableEntity {
  public:
    // constructor / desctructor
    PositionEstimationService();
    PositionEstimationService(Debuglevel positionEstimationServiceDebuglevel);
    // other
    void run(); // overrides virtual function "run" in base class
    void initialize();
    void terminate();
  private:
    std::shared_ptr<CameraDriver> accessCameraDriver; // an instance (+ shared pointer to access the) camera driver, which is managed by the PositionEstimationService
    std::shared_ptr<ImageTransformer> accessImageTransformer; // an instance (+ shared pointer to access the) image transformer, which is managed by the PositionEstimationService
    std::shared_ptr<PositionServer> accessPositionServer; // an instance (+ shared pointer to access the) position server, which is managed by the PositionEstimationService
};

#endif