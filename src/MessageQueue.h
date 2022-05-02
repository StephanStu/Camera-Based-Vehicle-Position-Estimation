#ifndef MESSAGEQUEUE_H
#define MESSAGEQUEUE_H

#include <mutex>
#include <deque>
#include <condition_variable>


// Class „MessageQueue“ has the public methods send and receive. 
// Send takes an rvalue reference of a given type whereas receive returns this type. 
// This class defines an std::dequeue called _queue, which stores objects of a given type. 

template <class T>
class MessageQueue
{
public:
	T receive();
    void send(T &&msg);

private:
	std::mutex mutex;
    std::condition_variable condition;
    std::deque<T> queue;    
};

#endif