#include <iostream>
#include <future>
#include <mutex>

#include "MessageQueue.h"

template <typename T>
T MessageQueue<T>::receive()
{
    // The method receive should use std::unique_lock<std::mutex> and _condition.wait() 
    // to wait for and receive new messages and pull them from the queue using move semantics. 
    // The received object should then be returned by the receive function. 
  
  	// perform queue modification under the lock
    std::unique_lock<std::mutex> uLock(mutex);
    /*
    If the vector is empty, wait is called. 
    When the thread wakes up again, the condition is immediately re-checked 
    and - in case it has not been a spurious wake-up we can continue with 
    our job and retrieve the vector.
    */
    condition.wait(uLock, [this] { return !queue.empty(); }); // pass unique lock to condition variable
  	// remove last vector element from queue
    T msg = std::move(queue.back());
    queue.pop_back();
	return msg; // will not be copied due to return value optimization (RVO) in C++
}

template <typename T>
void MessageQueue<T>::send(T &&msg)
{
    // FP.4a : The method send should use the mechanisms std::lock_guard<std::mutex> 
    // as well as _condition.notify_one() to add a new message to the queue and afterwards send a notification.
  	
  	// perform vector modification under the lock
    std::lock_guard<std::mutex> uLock(mutex);
  	// add vector to queue
    queue.push_back(std::move(msg));
    condition.notify_one(); // notify client after pushing new msg into vector
}