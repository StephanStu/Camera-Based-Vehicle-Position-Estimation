#ifndef MOVABLETIMESTAMPED_H
#define MOVABLETIMESTAMPED_H
#include <chrono>
#include <opencv2/opencv.hpp>

/*
MovableTimestampedType:
*/
template <typename T>
class MovableTimestampedType{
  public:
    MovableTimestampedType(const T argument); // constructor
    MovableTimestampedType(MovableTimestampedType<T> &&source); // move constructor
    MovableTimestampedType<T> &operator=(MovableTimestampedType<T> &&source); // move assignment operator
    ~MovableTimestampedType(); // destructor
    MovableTimestampedType(const MovableTimestampedType<T>& source); // copy constructor
    MovableTimestampedType<T> &operator=(const MovableTimestampedType<T> &source); // copy assignment operator
    T getContent(); // get the data
    void setContent(T argument); // change the data
    long int getAge();
  private:
    T *data; // payload
    const std::chrono::system_clock::time_point birth; // point in time when instance is created
};

template class MovableTimestampedType<int>; // tells the compiler to make this class available for T = int
template class MovableTimestampedType<cv::Mat>; // tells the compiler to make this class available for T = cv::Mat

#endif