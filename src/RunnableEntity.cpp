#include <algorithm>
#include <iostream>
#include <string>
#include "RunnableEntity.h"

std::mutex RunnableEntity::consoleProtection;

RunnableEntity::RunnableEntity() : sleepForMilliseconds(100), currentState(State::terminated), debugLevel(Debuglevel::none), ready(false) {
}

RunnableEntity::~RunnableEntity(){
  // set up thread barrier before this object is destroyed
  std::for_each(threads.begin(), threads.end(), [](std::thread &t) {
    t.join();
  });
}

State RunnableEntity::getCurrentState(){
  std::string message;
  switch (currentState){
    case initializing: message = "RunnableEntity::getCurrentState is called, state is currently initializing.";
    case running: message = "RunnableEntity::getCurrentState is called, state is currently running.";
    case freezed: message = "RunnableEntity::getCurrentState is called, state is currently freezed.";
    case terminated: message = "RunnableEntity::getCurrentState is called, state is currently terminated.";
    default: message = "RunnableEntity::getCurrentState is called, state is currently terminated.";
    }
  printToConsole(message);
  return currentState;
}

void RunnableEntity::setCurrentState(State targetState){
  std::string message;
  if (targetState == initializing){
    message = "RunnableEntity::setCurrentState is called, state is set to initializing.";
  }
  if (targetState == running){
    message = "RunnableEntity::setCurrentState is called, state is set to running";
  }
  if (targetState == terminated){
    message = "RunnableEntity::setCurrentState is called, state is set to terminated.";
  }
  if (targetState == freezed){
    message = "RunnableEntity::setCurrentState is called, state is set to freezed.";
  }
  printToConsole(message);
  currentState = targetState;
} 

void RunnableEntity::printToConsole(std::string message){
  if(debugLevel == Debuglevel::verbose){
    std::unique_lock<std::mutex> lock(consoleProtection);
    std::cout << "# From thread with id:" << std::this_thread::get_id() << ": " << message << std::endl;
    lock.unlock();
  }
}

 void RunnableEntity::setSleepForMilliseconds(unsigned int time){
   std::string message;
   message = "RunnableEntity::setSleepForMilliseconds is called, setting sleepForMillisecons equal to " + std::to_string(time) + ".";
   printToConsole(message);
   sleepForMilliseconds = time;
 }

bool RunnableEntity::isReady(){
  std::string message;
  if(ready){
    message = "RunnableEntity::isReady called, instance is ready to execute the run.";
  }else{
    message = "RunnableEntity::isReady called, instance is not yet ready to execute the run.";
  }
  printToConsole(message);
  return ready;
}