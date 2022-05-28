#include "PositionServer.h"

PositionServer::PositionServer(){
  debugLevel = Debuglevel::none;
  printToConsole("PositionServer::PositionServer called.");
}

PositionServer::PositionServer(Debuglevel positionServerDebuglevel){
  debugLevel = positionServerDebuglevel;
  printToConsole("PositionServer::PositionServer called.");
}

void PositionServer::run(){
  printToConsole("PositionServer::run called.");
  threads.emplace_back(std::thread(&PositionServer::estimatePosition, this));
} 

void PositionServer::estimatePosition(){
  printToConsole("PositionServer::estimatePosition called.");
  while(true){
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    printToConsole("PositionServer::estimatePosition is still idling.");
  }
}