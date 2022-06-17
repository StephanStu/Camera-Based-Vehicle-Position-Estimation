#include <chrono>
#include <ctime>
#include <stdlib.h>
#include <iostream>
#include "MovableTimestampedType.h"

template <typename T>
MovableTimestampedType<T>::MovableTimestampedType(const T argument) : birth(std::chrono::system_clock::now()) {
  data = new T;
  *data = argument;
  std::cout << "CREATING instance of MyMovableClass at " << this << " allocated with size = " << sizeof(T)  << " bytes" << std::endl;
}

template <typename T>
MovableTimestampedType<T>::~MovableTimestampedType(){
  std::cout << "DELETING instance of MyMovableClass at " << this << std::endl;
  delete data;
}

template <typename T>
MovableTimestampedType<T>::MovableTimestampedType(MovableTimestampedType<T> &&source){
  std::cout << "MOVING (c’tor) instance " << &source << " to instance " << this << std::endl;
  data = source.data;
  source.data = nullptr;
}

template <typename T>
MovableTimestampedType<T> &MovableTimestampedType<T>::operator=(MovableTimestampedType<T> &&source){
  std::cout << "MOVING (assign) instance " << &source << " to instance " << this << std::endl;
  if (this == &source){
    return *this;
  }
  delete data;
  data = source.data;
  source.data = nullptr;
  return *this;
}

template <typename T>
MovableTimestampedType<T> &MovableTimestampedType<T>::operator=(const MovableTimestampedType<T>& source){
  std::cout << "ASSIGNING content of instance " << &source << " to instance " << this << std::endl;
  if (this == &source){
    return *this;
  }
  delete data;
  data = new T;
  *data = *source.data;
  return *this;
}

template <typename T>
MovableTimestampedType<T>::MovableTimestampedType(const MovableTimestampedType<T> &source){
  data = new T;
  *data = *source.data;
  std::cout << "COPYING content of instance " << &source << " to instance " << this << std::endl;
}

template <typename T>
T MovableTimestampedType<T>::getContent(){
  T arg = *data;
  return arg;
}

template <typename T>
void MovableTimestampedType<T>::setContent(T argument){
  *data = argument;
}

template <typename T>
 long int MovableTimestampedType<T>::getAge(){
   long int now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch()).count();
   long int birth_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::time_point_cast<std::chrono::milliseconds>(birth).time_since_epoch()).count();
   long int age = now_ms - birth_ms;
   return age;
}