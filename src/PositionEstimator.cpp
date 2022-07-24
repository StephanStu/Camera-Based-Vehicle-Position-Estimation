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
        std::string message = "PositionEstimator::manageStateSwitches is called: Instance is in state running with isCurrent = " + std::to_string(isCurrent) + ".";
        printToConsole(message);
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
  if(newRecordIsAvailable()){
    Position position;
    std::promise<MovableTimestampedType<PositionServiceRecord>> prms;
    std::future<MovableTimestampedType<PositionServiceRecord>> ftr = prms.get_future();
    std::thread t(&ImageTransformer::sendRecord, accessImageTransformer, std::move(prms));
    t.join();
    MovableTimestampedType<PositionServiceRecord> movableTimestampedRecord = std::move(ftr.get());
    /* apply Hough-Transform and update the estimate of the position */
    position = updatePosition(movableTimestampedRecord);
    /* use movableTimestampedRecord.setData(newRecord) to update the position here */
    
    /* save the updated record in the member variable by "moving" it (in order to keep the timestamp!) */
    std::unique_lock<std::mutex> uniqueLock(protection);
    record = std::move(movableTimestampedRecord);
    isCurrent = true;
    uniqueLock.unlock();
  }else{
    printToConsole("PositionEstimator::runInRunningState called: Wainting for :ImageTransformer to update it's record.");
  }
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


Position PositionEstimator::updatePosition(MovableTimestampedType<PositionServiceRecord>& movableTimestampedRecord){
  long int age = movableTimestampedRecord.getAge();
  std::string message = "PositionEstimator::updatePosition: Using a record with an age of " + std::to_string(age) + " milliseconds to compute angle & deviation.";
  printToConsole(message);
  std::vector<cv::Vec4i> lines;
  cv::Mat image = movableTimestampedRecord.getData().binaryBirdEyesViewImage;
  getHoughLines(image, lines);
  /* Print the lines found to console if suitable debugLevel is chosen */
  if(lines.size()>0){
    std::string message = "PositionEstimator::updatePosition: Found " + std::to_string(lines.size()) + " lines in the current image.";
    printToConsole(message);
    for (size_t i=0; i<lines.size(); i++) {
      cv::Vec4i l = lines[i];
      std::string message = "PositionEstimator::updatePosition: Line " + std::to_string(i+1) + " is [ " + std::to_string(l[0]) + " " +  std::to_string(l[1]) + " " + std::to_string(l[2]) + " " + std::to_string(l[3]) + " ].";
      printToConsole(message);
    }
  }
  /* Use age and lines to update the kalman filter and return the current position */
  Position position;
  position.angle = 0.0;
  position.deviation = 0.0;
  return position;
}

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

bool PositionEstimator::newRecordIsAvailable(){
  return accessImageTransformer->isCurrent;
}