#ifndef RUNNABLEENTITY_H
#define RUNNABLEENTITY_H

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include "Types.h"

/*
RunnableEnitity:
This is the base class for all entities that must launch threads to provide their services.
It protects the console and allows thread-safe printing of messages.
It keeps the sate of the component and offers an interface to manipulate it.
*/

class RunnableEntity{
  public:
    // constructor / desctructor
    RunnableEntity();
    ~RunnableEntity();
    // more
  	virtual void run() = 0; // must be implemented by childs and is defined as the command to be called to make the service 'spin forever until termination' and manage the mode-switch events
    State getCurrentState(); // returns the current state
    bool isReady(); // return the ready-variable to a requester
  protected:
  	void printToConsole(std::string message); // this method prints to console in a thread-safe way
    State currentState; // this variable is the current state of the software component (as defined in the enumerated type)
  	Debuglevel debugLevel; // this variabel is set to "none" or "verbose" to allow messages from methods to be printed or not
  	std::vector<std::thread> threads; // this vector holds all threads that have been launched within this object
    static std::mutex consoleProtection; // this mutex is shared by all objects for protecting access to console
    void setCurrentState(State targetState); // this function sets the current state of the software component
    unsigned int sleepForMilliseconds; // this is the timing event of the thread that implements the service, i.e. the amount of time the thread sleeps to save resources in the operating system
    void setSleepForMilliseconds(unsigned int time); // this method sets the time the service is allowed to sleep in the thread
    bool ready; // if the runnable entity is ready to execute the run state, this is true, otherwise it is false
};

#endif