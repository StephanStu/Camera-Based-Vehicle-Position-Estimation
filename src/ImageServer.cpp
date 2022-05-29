#include <iostream>
#include <future>
#include <mutex>
#include "ImageServer.h"

void ImageServer::addImageToQueue(cv::Mat &&image){
  std::lock_guard<std::mutex> lock(queueProtection);
  queue.push_back(std::move(image));
  condition.notify_one(); 
  printToConsole("ImageServer::addImageToQueue called; ingesting an image.");
}

cv::Mat ImageServer::returnImageFromQueue(){
  std::unique_lock<std::mutex> lock(queueProtection);
  condition.wait(lock, [this] {return !queue.empty();}); 
  cv::Mat image = std::move(queue.back());
  queue.pop_back();
  lock.unlock();
  printToConsole("ImageServer::returnImageFromQueue called; returning an image to requester.");
  return image;
}

void ImageServer::sendImageFromQueue(std::promise<cv::Mat> &&imagePromise){
  printToConsole("ImageServer::sendImageFromQueue called.");
  std::unique_lock<std::mutex> lock(queueProtection);
  try{
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
  lock.unlock();
}