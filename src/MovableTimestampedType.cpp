#include <chrono>
#include <ctime>
#include <stdlib.h>
#include <iostream>
#include "MovableTimestampedType.h"

MovableTimestampedType::MovableTimestampedType(const int arg) : birth(std::chrono::system_clock::now()) {
  _data = new int;
  *_data = arg;
  std::cout << "CREATING instance of MyMovableClass at " << this << " allocated with size = " << sizeof(int)  << " bytes" << std::endl;
}

MovableTimestampedType::MovableTimestampedType(MovableTimestampedType &&source){
  std::cout << "MOVING (c’tor) instance " << &source << " to instance " << this << std::endl;
  _data = source._data;
  source._data = nullptr;
}

MovableTimestampedType &MovableTimestampedType::operator=(MovableTimestampedType &&source){
  std::cout << "MOVING (assign) instance " << &source << " to instance " << this << std::endl;
  if (this == &source){
    return *this;
  }
  delete _data;
  _data = source._data;
  source._data = nullptr;
  return *this;
}

MovableTimestampedType::~MovableTimestampedType(){
  std::cout << "DELETING instance of MyMovableClass at " << this << std::endl;
  delete _data;
}

MovableTimestampedType::MovableTimestampedType(const MovableTimestampedType &source){
  _data = new int;
  *_data = *source._data;
  std::cout << "COPYING content of instance " << &source << " to instance " << this << std::endl;
}
    
MovableTimestampedType &MovableTimestampedType::operator=(const MovableTimestampedType &source){
  std::cout << "ASSIGNING content of instance " << &source << " to instance " << this << std::endl;
  if (this == &source){
    return *this;
  }
  delete _data;
  _data = new int;
  *_data = *source._data;
  return *this;
}

int MovableTimestampedType::getContent(){
  int arg = *_data;
  return arg;
}

void MovableTimestampedType::setContent(int arg){
  *_data = arg;
}

 long int MovableTimestampedType::getAge(){
   long int now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch()).count();
   long int birth_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::time_point_cast<std::chrono::milliseconds>(birth).time_since_epoch()).count();
   long int age = now_ms - birth_ms;
   return age;
}