#include <iostream>
#include <future>
#include <mutex>
#include <string>
#include "ImageServer.h"

void ImageServer::addImageToQueue(cv::Mat &&image){
  std::unique_lock<std::mutex> lock(queueProtection);
  queue.push_back(std::move(image));
  lock.unlock();
  condition.notify_one(); 
  printToConsole("ImageServer::addImageToQueue called; ingesting an image.");
}

cv::Mat ImageServer::getImageFromQueue(){
  printToConsole("ImageServer::getImageFromQueue called.");
  std::unique_lock<std::mutex> lock(queueProtection);
  //condition.wait(lock, [this] {return !queue.empty();}); 
  while(queue.empty()){
    condition.wait(lock); 
  }
  cv::Mat image = std::move(queue.back());
  queue.pop_back();
  lock.unlock();
  condition.notify_one();
  printToConsole("ImageServer::getImageFromQueue finished; returning an image to requester.");
  return image;
}

void ImageServer::receiveImageFromQueue(std::promise<cv::Mat> &&imagePromise){
  printToConsole("ImageServer::receiveImageFromQueue called.");
  std::unique_lock<std::mutex> lock(queueProtection);
  while(queue.empty()){
    condition.wait(lock); 
  }
  cv::Mat image = std::move(queue.back());
  queue.pop_back();
  imagePromise.set_value(image);
  lock.unlock();
  condition.notify_one();
  /*try{
    if (queue.empty()){
      throw std::runtime_error("ImageServer::sendImageFromQueue called with error: Queue is empty.");
      printToConsole("ImageServer::sendImageFromQueue: Queue is empty.");
    }else{
      cv::Mat image = std::move(queue.back());
      queue.pop_back();
      imagePromise.set_value(image);
    }
  }catch (...){
    imagePromise.set_exception(std::current_exception());
  }
  lock.unlock();*/
  printToConsole("ImageServer::receiveImageFromQueue finished; returning an image to requester.");
}

unsigned int ImageServer::getQueueLength(){
  unsigned int size = queue.size();
  std::string message = "ImageServer::getQueueLength called, queue is of size " + std::to_string(size) + ".";
  printToConsole(message);
  return size;
}