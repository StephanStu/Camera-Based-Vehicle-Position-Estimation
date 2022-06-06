#include <algorithm>
#include <iostream>
#include <string>
#include "RunnableEntity.h"

std::mutex RunnableEntity::consoleProtection;
State currentState = State::terminated;

RunnableEntity::RunnableEntity()
{
    debugLevel = Debuglevel::none;
}

RunnableEntity::~RunnableEntity()
{
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
  switch (targetState){
    case initializing: message = "RunnableEntity::setCurrentState is called, state is set to initializing.";
    case running: message = "RunnableEntity::setCurrentState is called, state is set to running.";
    case freezed: message = "RunnableEntity::setCurrentState is called, state is set to freezed.";
    case terminated: message = "RunnableEntity::setCurrentState is called, state is set to terminated.";
    default: message = "RunnableEntity::setCurrentState is called, state is set to terminated.";
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