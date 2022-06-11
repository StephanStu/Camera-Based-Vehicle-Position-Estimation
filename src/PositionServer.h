#ifndef POSITIONSERVER_H
#define POSITIONSERVER_H

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
    void run(); // overrides virtual function "run" in base class
  private:
    void estimatePosition(); // runs a kalman-filter to estimate the vehicle's distance to center line based on transformed images pulled from :ImageTransformer.
    friend class PositionService;
};

#endif