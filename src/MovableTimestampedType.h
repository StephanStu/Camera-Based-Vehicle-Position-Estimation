#ifndef MOVABLETIMESTAMPED_H
#define MOVABLETIMESTAMPED_H
#include <chrono>
#include <opencv2/opencv.hpp>
#include "Types.h"

/*
MovableTimestampedType:
*/

template <typename T>
class MovableTimestampedType{
  public:
    MovableTimestampedType(); // simple constructor
    MovableTimestampedType(const T argument); // constructor with argument
    MovableTimestampedType(const T argument, Debuglevel typeDebuglevel); // constructor with argument + debug-level
    MovableTimestampedType(MovableTimestampedType<T> &&source); // move constructor
    MovableTimestampedType<T> &operator=(MovableTimestampedType<T> &&source); // move assignment operator
    ~MovableTimestampedType(); // destructor
    MovableTimestampedType(const MovableTimestampedType<T> &source); // copy constructor
    MovableTimestampedType<T> &operator=(const MovableTimestampedType<T> &source); // copy assignment operator
    T getData(); // get the data
    void setData(T argument); // change the data
    long int getAge();
  private:
    T *data; // (pointer to )payload
    std::chrono::system_clock::time_point birth; // (pointer to) point in time when instance is created
    Debuglevel debugLevel; // (pointer to) variable which is set to "none" or "verbose" to allow messages from methods to be printed or not
};

template class MovableTimestampedType<int>; // tells the compiler to make this class available for datatype T = int
template class MovableTimestampedType<cv::Mat>; // tells the compiler to make this class available for datatype T = cv::Mat
template class MovableTimestampedType<PositionServiceRecord>; // tells the compiler to make this class available for datatype T = PositionServiceRecord

#endif