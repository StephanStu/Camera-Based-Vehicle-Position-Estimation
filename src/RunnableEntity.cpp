#include <algorithm>
#include <iostream>
#include "RunnableEntity.h"

std::mutex RunnableEntity::consoleProtection;

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

void RunnableEntity::printToConsole(std::string message){
  if(debugLevel == Debuglevel::verbose){
    std::unique_lock<std::mutex> lock(consoleProtection);
    std::cout << "# From thread with id:" << std::this_thread::get_id() << ": " << message << std::endl;
    lock.unlock();
  }
}