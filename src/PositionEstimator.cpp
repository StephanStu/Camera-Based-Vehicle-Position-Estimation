#include "PositionEstimator.h"

PositionEstimator::PositionEstimator() : imageTransformerIsMounted(false) {
  debugLevel = Debuglevel::none;
}

PositionEstimator::PositionEstimator(Debuglevel positionEstimatorDebuglevel) : imageTransformerIsMounted(false) {
  debugLevel = positionEstimatorDebuglevel;
  printToConsole("PositionEstimator::PositionServer called.");
}

void PositionEstimator::run(){
  printToConsole("PositionEstimator::run called.");
  threads.emplace_back(std::thread(&PositionEstimator::manageStateSwitches, this));
}

void PositionEstimator::mountImageTransformer(std::shared_ptr<ImageTransformer> pointerToImageTransformer){
  printToConsole("PositionEstimator::mountImageTransformer called, instance of ImageTransformer is mounted by keeping the shared pointer.");
  accessImageTransformer = pointerToImageTransformer;
  imageTransformerIsMounted = true;
}

void PositionEstimator::manageStateSwitches(){
  while(true){
    if(currentState == initializing){
      printToConsole("PositionEstimator::manageStateSwitches is called, instance is waiting in state initializing.");
      runInInitializingState();
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
    if(currentState == running){
      if(ready){
        printToConsole("PositionEstimator::manageStateSwitches is called: Instance is in state running.");
        runInRunningState();
      }else{
        printToConsole("PositionEstimator::manageStateSwitches is called: Instance is not ready & calling the initializing routine.");
        runInInitializingState();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
    if(currentState == terminated){
      printToConsole("PositionEstimator::manageStateSwitches is called, instance has reached state terminated.");
      runInTerminatedState();
      break;
    }
    if(currentState == freezed){
      printToConsole("PositionEstimator::manageStateSwitches is called, instance is waiting in state freezed.");
      runInFreezedState();
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
  }
}

void PositionEstimator::runInRunningState(){
  /* get the record from :ImageTransformer */
  std::promise<MovableTimestampedType<PositionServiceRecord>> prms;
  std::future<MovableTimestampedType<PositionServiceRecord>> ftr = prms.get_future();
  std::thread t(&ImageTransformer::sendRecord, accessImageTransformer, std::move(prms));
  t.join();
  MovableTimestampedType<PositionServiceRecord> movableTimestampedRecord = std::move(ftr.get());
  /* apply Hough-Transform and update the estimate of the position */
  PositionServiceRecord newRecord = movableTimestampedRecord.getData();
  updatePosition(newRecord);
  movableTimestampedRecord.setData(newRecord);
  /* save the updated record in the member variable by "moving" it (in order to keep the timestamp!) */
  std::unique_lock<std::mutex> uniqueLock(protection);
  condition.wait(uniqueLock); 
  record = std::move(movableTimestampedRecord);
  isCurrent = true;
  uniqueLock.unlock();
  condition.notify_one();
}

void PositionEstimator::runInInitializingState(){
  isCurrent = false;
  ready = imageTransformerIsMounted;
  if(!ready){
    printToConsole("PositionEstimator::runInInitializingState is called; instance is not ready to enter the running state: Mount an :ImageTransformer.");
  }
}

void PositionEstimator::runInFreezedState(){
  isCurrent = false;
}

void PositionEstimator::runInTerminatedState(){
  isCurrent = false;
}

void PositionEstimator::updatePosition(const PositionServiceRecord& newRecord){}

void PositionEstimator::getHoughLines(const cv::Mat& image, std::vector<cv::Vec4i>& lines){
  printToConsole("PositionEstimator::getHoughLines is called.");
  cv::HoughLinesP(image, lines, 1, CV_PI/180, threshold, minLineLength, maxLineGap);
}

void PositionEstimator::createHoughLinesImage(const cv::Mat& source, cv::Mat destination){
  printToConsole("PositionEstimator::createHoughLinesImage is called.");
  std::vector<cv::Vec4i> lines;
  getHoughLines(source, lines);
  destination = cv::Mat::zeros(source.size(), CV_8UC3);
  /* draw lines on the image (if any are found) */
  if(lines.size()>0){
    for (size_t i=0; i<lines.size(); i++) {
      cv::Vec4i l = lines[i];
      cv::line(destination, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
    }
  }
}