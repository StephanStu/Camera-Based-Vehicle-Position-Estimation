#ifndef IMAGESERVER_H
#define IMAGESERVER_H

#include "MessageQueue.h"

class ImageServer{
  public:
  // constructor / desctructor
  ImageServer();
  // getters / setters
  
  // typical behaviour methods
  
  private:
  // data
  MessageQueue<MovableImageData> messageQueue;
  // typical behaviour methods
  void pullImageFromCameraDriver(CameraDriver& camera);
};

#endif