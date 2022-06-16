#ifndef IMAGESERVER_H
#define IMAGESERVER_H

#include <future>
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
    void receiveImageFromQueue(std::promise<cv::Mat> &&imagePromise); //  receive an image via promise-future and pops the image just send from the queue
    unsigned int getQueueLength(); // returns the length of the queue to e.g. observe if the queue is growing and growing due to resource shortage or if it stays close to zero due to consumers constantly pulling images
  protected:
    void addImageToQueue(cv::Mat &&image); // adds an image to the queue using std::move()
    void clearQueue(); // clears the queue of images (to free memory!)
    cv::Mat getImageFromQueue(); // pops an image from the queue and returns
	std::mutex queueProtection; // this mutex is shared by all objects for protecting access to the queues
    std::condition_variable condition; // condition varibale is needed to notify clients
    std::deque<cv::Mat> queue; // queue is our implementaion of the image queue
  private:
    friend class PositionService;
};

#endif