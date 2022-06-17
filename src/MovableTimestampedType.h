#ifndef MOVABLETIMESTAMPED_H
#define MOVABLETIMESTAMPED_H
#include <chrono>

/*
MovableTimestampedType:
*/

class MovableTimestampedType{
  public:
    MovableTimestampedType(const int arg); // constructor
    MovableTimestampedType(MovableTimestampedType &&source); // move constructor
    MovableTimestampedType &operator=(MovableTimestampedType &&source); // move assignment operator
    ~MovableTimestampedType(); // destructor
    MovableTimestampedType(const MovableTimestampedType &source); // copy constructor
    MovableTimestampedType &operator=(const MovableTimestampedType &source); // copy assignment operator
    int getContent(); // get the data
    void setContent(int arg); // change the data
    long int getAge();
  private:
    int *_data; // payload
    const std::chrono::system_clock::time_point birth; // point in time when instance is created
};

#endif