#ifndef IMAGESERVER_H
#define IMAGESERVER_H

#include <mutex>
#include <deque>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include "RunnableEntity.h"
#include "Types.h"

/*
ImageServer:
*/

class ImageServer : public RunnableEntity {
  public:
    // constructor / desctructor
    //ImageServer(Debuglevel imageServerDebugLevel);
    //ImageServer();
    // other

  protected:
    void addImageToQueue(cv::Mat &&image); // adds an image to the queue using std::move()
    cv::Mat getImageFromQueue(); // pops an image from the queue
	std::mutex queueProtection; // this mutex is shared by all objects for protecting access to the queues
    std::condition_variable condition; // condition varibale is needed to notify clients
    std::deque<cv::Mat> queue; // queue is our implementaion of the image queue
};

#endif