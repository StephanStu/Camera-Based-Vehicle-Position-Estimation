#include <chrono>
#include <ctime>
#include <stdlib.h>
#include <iostream>
#include "MovableTimestampedType.h"

template <typename T>
MovableTimestampedType<T>::MovableTimestampedType(const T argument) : birth(std::chrono::system_clock::now()), debugLevel(Debuglevel::none) {
  data = new T;
  *data = argument;
}

template <typename T>
MovableTimestampedType<T>::MovableTimestampedType(const T argument, Debuglevel typeDebuglevel) : birth(std::chrono::system_clock::now()) {
  data = new T;
  debugLevel = typeDebuglevel;
  *data = argument;
  if(debugLevel==Debuglevel::verbose){
    std::cout << "# Creating instance of MovableTimestampedType at " << this << " allocated with size = " << sizeof(T)  << " bytes" << std::endl;
  }
}

template <typename T>
MovableTimestampedType<T>::~MovableTimestampedType(){
  if(debugLevel==Debuglevel::verbose){
    std::cout << "# Deleting instance of MovableTimestampedType at " << this << std::endl;
  }
  if(data != NULL){delete data;}
  /*delete data;*/
}

template <typename T>
MovableTimestampedType<T>::MovableTimestampedType(MovableTimestampedType<T> &&source){
  if(debugLevel==Debuglevel::verbose){
    std::cout << "# Moving (c’tor) instance " << &source << " to instance " << this << std::endl;
  }
  data = source.data;
  birth = source.birth;
  debugLevel = source.debugLevel;
  source.data = NULL;
}

template <typename T>
MovableTimestampedType<T>::MovableTimestampedType(const MovableTimestampedType<T> &source){
  if(debugLevel==Debuglevel::verbose){
    std::cout << "# Copying content of instance " << &source << " to instance " << this << std::endl;
  }
  data = new T;
  *data = *source.data;
  birth = source.birth;
  debugLevel = source.debugLevel;
}

template <typename T>
MovableTimestampedType<T> &MovableTimestampedType<T>::operator=(MovableTimestampedType<T> &&source){
  if(debugLevel==Debuglevel::verbose){
    std::cout << "# Moving (assign) instance " << &source << " to instance " << this << std::endl;
  }
  if (this == &source){
    return *this;
  }
  delete data;
  data = source.data;
  birth = source.birth;
  debugLevel = source.debugLevel;
  source.data = NULL;
  return *this;
}

template <typename T>
MovableTimestampedType<T> &MovableTimestampedType<T>::operator=(const MovableTimestampedType<T>& source){
  if(debugLevel==Debuglevel::verbose){
    std::cout << "# Assigning content of instance " << &source << " to instance " << this << std::endl;
  }
  if (this == &source){
    return *this;
  }
  delete data;
  data = new T;
  *data = *source.data;
  birth = source.birth;
  debugLevel = source.debugLevel;
  return *this;
}

template <typename T>
T MovableTimestampedType<T>::getData(){
  T arg = *data;
  return arg;
}

template <typename T>
void MovableTimestampedType<T>::setData(T argument){
  *data = argument;
}

template <typename T>
 long int MovableTimestampedType<T>::getAge(){
   long int now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch()).count();
   long int birth_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::time_point_cast<std::chrono::milliseconds>(birth).time_since_epoch()).count();
   long int age = now_ms - birth_ms;
   return age;
}