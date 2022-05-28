#include <iostream>
#include <future>
#include <mutex>
#include "ImageServer.h"

/*ImageServer::ImageServer(Debuglevel imageServerDebugLevel){
  debugLevel = imageServerDebugLevel;
  printToConsole("ImageServer::ImageServer called.");
}*/

void ImageServer::addImageToQueue(cv::Mat &&image){
  std::lock_guard<std::mutex> lock(queueProtection);
  queue.push_back(std::move(image));
  condition.notify_one(); 
  printToConsole("ImageServer::addImageToQueue called; ingesting an image.");
}

cv::Mat ImageServer::getImageFromQueue(){
  std::unique_lock<std::mutex> lock(queueProtection);
  condition.wait(lock, [this] {return !queue.empty();}); 
  cv::Mat image = std::move(queue.back());
  queue.pop_back();
  lock.unlock();
  printToConsole("ImageServer::getImageFromQueue called; returning an image to requester.");
  return image;
}
/*
void ImageServer::run(){
  printToConsole("ImageServer::run called.");
}*/