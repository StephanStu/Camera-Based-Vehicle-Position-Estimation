#include <iostream>

#include "CameraDriver.h"
#include "Types.h"

/* Implementation of class "CameraDriver" */

CameraDriver::CameraDriver(Debuglevel debuglevel){
  camerDriverDebuglevel = debuglevel;
  if((debuglevel == Debuglevel::verbose) || (debuglevel == Debuglevel::all)){
    std::cout << " CameraDriver constructor called." << std::endl;
  }
}

CameraDriver::CameraDriver(){
  camerDriverDebuglevel = Debuglevel::none;
}

void CameraDriver::calibrate(){
  
}
