#include <iostream>
#include <future>
#include <mutex>

#include "MessageQueue.h"

template <typename T> 
void MessageQueue<T>::addItemToQueue(T &&item){
  std::lock_guard<std::mutex> uLock(mutex);
  queue.push_back(std::move(item));
  condition.notify_one(); 
}

template <typename T>
T MessageQueue<T>::getItemFromQueue(){
  std::unique_lock<std::mutex> uLock(mutex);
  condition.wait(uLock, [this] {return !queue.empty();}); 
  T item = std::move(queue.back());
  queue.pop_back();
  return item;
}