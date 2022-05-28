#ifndef RUNNABLEENTITY_H
#define RUNNABLEENTITY_H

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include "Types.h"

/*
RunnableEnitity:
*/

class RunnableEntity{
  public:
    // constructor / desctructor
    RunnableEntity();
    ~RunnableEntity();
    // more
  	virtual void run() = 0; // must be implemented by childs and is defined as the command to be called to make the service 'spin forever'
  	void printToConsole(std::string message); // this method prints to console in a thread-safe way
  protected:
  	Debuglevel debugLevel; // this variabel is set to "none" or "verbose" to allow messages from methods to be printed or not
  	std::vector<std::thread> threads; // this vector holds all threads that have been launched within this object
    static std::mutex consoleProtection; // this mutex is shared by all objects for protecting access to console 
};

#endif