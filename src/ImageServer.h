#ifndef MESSAGEQUEUE_H
#define MESSAGEQUEUE_H

#include <mutex>
#include <deque>
#include <condition_variable>

template <class T>
class MessageQueue
{
public:
    void addItemToQueue(T &&item);
    T getItemFromQueue();
    
private:
	std::mutex mutex;
    std::condition_variable condition;
    std::deque<T> queue;  
};

#endif