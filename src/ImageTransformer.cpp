#include "ImageTransformer.h"

ImageTransformer::ImageTransformer(){
  debugLevel = Debuglevel::none;
  printToConsole("ImageTransformer::ImageTransformer called.");
}

ImageTransformer::ImageTransformer(Debuglevel imageTransformerDebugLevel){
  debugLevel = imageTransformerDebugLevel;
  printToConsole("ImageTransformer::ImageTransformer called.");
}
    
void ImageTransformer::run(){
  printToConsole("ImageTransformer::run called.");
  threads.emplace_back(std::thread(&ImageTransformer::manageStateSwitches, this));
}

void ImageTransformer::applyImageProcessingToRecord(PositionServiceRecord& record){
  printToConsole("ImageTransformer::applyImageProcessingToRecord called.");
  cv::Mat undistortedImage = record.undistortedImage;
  cv::Mat gaussianBlurredImage;
  cv::Mat grayImage;
  cv::Mat birdEyesViewImage;
  cv::Mat binaryBirdEyesViewImage;
  cv::Mat binaryEdgesDetcted;
  applyGausianBlurr(undistortedImage , gaussianBlurredImage);
  convertToGrayImage(gaussianBlurredImage, grayImage);
  convertToBirdEyesView(grayImage, birdEyesViewImage);
  //maskBirdEyesView();
  convertToBinaryImage(birdEyesViewImage, binaryBirdEyesViewImage);
  detectEdges(binaryBirdEyesViewImage, binaryEdgesDetcted);
  record.birdEyesViewImage = birdEyesViewImage;
  record.binaryBirdEyesViewImage = binaryEdgesDetcted;
}

void ImageTransformer::convertToBinaryImage(cv::Mat& source, cv::Mat& destination){
  printToConsole("ImageTransformer::convertToBinaryImage called.");
  cv::threshold(source, destination, binaryThresholdValue, maxBinaryValue, cv::THRESH_BINARY);
}

void ImageTransformer::convertToBirdEyesView(cv::Mat& source, cv::Mat& destination){
  printToConsole("ImageTransformer::convertToBirdEyesView called.");
  cv::Size birdsEyeImageSize(birdsEyeImageWidth, birdsEyeImageHeight);
  cv::warpPerspective(source,			// Source image
                        destination, 	// Destination image
                        birdsEyeTransformMatrix, // Transformation matrix
                        birdsEyeImageSize,   // Size for output image. Before: image.size()
                        cv::INTER_LINEAR, // cv::WARP_INVERSE_MAP | cv::INTER_LINEAR,
                        cv::BORDER_CONSTANT, cv::Scalar::all(0) // Fill border with black
                        );
}

void ImageTransformer::convertToGrayImage(cv::Mat& source, cv::Mat& destination){
  printToConsole("ImageTransformer::convertToGrayImage called.");
  cv::cvtColor(source, destination, cv::COLOR_BGR2GRAY);
}

void ImageTransformer::detectEdges(cv::Mat& source, cv::Mat& destination){
  printToConsole("ImageTransformer::detectEdges called.");
  cv::Canny(source, destination, 50, 200);
}
    
void ImageTransformer::applyGausianBlurr(cv::Mat& source, cv::Mat& destination){
  printToConsole("ImageTransformer::applyGausianBlurr called.");
  cv::Size size(kernelsize, kernelsize);
  cv::GaussianBlur(source, destination, size, 0, 0);
}
    
void ImageTransformer::maskBirdEyesView(cv::Mat& source, cv::Mat& destination){
  printToConsole("ImageTransformer::maskBirdEyesView called.");
  destination = source; // WOKR THIS OUT
}
    
void ImageTransformer::setBirdEyesTransformMatrix(){
  printToConsole("ImageTransformer::setBirdEyesTransformMatrix called.");
  cv::Point2f objPts[4], imgPts[4];
  objPts[0].x = objectPoint1XValue; objPts[0].y = objectPoint1YValue; 
  objPts[1].x = objectPoint2XValue; objPts[1].y = objectPoint2YValue;
  objPts[2].x = objectPoint3XValue; objPts[2].y = objectPoint3YValue; 
  objPts[3].x = objectPoint4XValue; objPts[3].y = objectPoint4YValue; 
  imgPts[0].x = 0; imgPts[0].y = 0;
  imgPts[1].x = birdsEyeImageWidth; imgPts[1].y = 0;
  imgPts[2].x = birdsEyeImageWidth; imgPts[2].y = birdsEyeImageHeight; 
  imgPts[3].x = 0; imgPts[3].y = birdsEyeImageHeight; 
  double Z = 1;
  cv::Mat H = cv::getPerspectiveTransform(objPts, imgPts);
  H.at<double>(2, 2) = Z;
  birdsEyeTransformMatrix = H;
  ready = true;
}

void ImageTransformer::mountCamerDriver(std::shared_ptr<CameraDriver> pointerToCameraDriver){
  printToConsole("ImageTransformer::mountCameraDriver called, instance of CamerDriver is mounted via saving the shared pointer.");
  accessCameraDriver = pointerToCameraDriver;
}

/*cv::Mat ImageTransformer::getImageFromMountedCameraDriver(){
  printToConsole("ImageTransformer::getImageFromMountedCameraDriver called, pulling an image with promise-future-mechanism.");
  std::promise<cv::Mat> prms;
  std::future<cv::Mat> ftr = prms.get_future();
  std::thread t(&CameraDriver::receiveImageFromQueue, accessCameraDriver, std::move(prms));
  t.join();
  cv::Mat image = ftr.get();
  return image;
}*/

/*MovableTimestampedType<PositionServiceRecord> ImageTransformer::getRecordFromMountedCameraDriver(){
  printToConsole("ImageTransformer::getRecordFromMountedCameraDriver called, pulling a record with promise-future-mechanism from mounted camera driver.");
  std::promise<MovableTimestampedType<PositionServiceRecord>> prms;
  std::future<MovableTimestampedType<PositionServiceRecord>> ftr = prms.get_future();
  std::thread t(&CameraDriver::receiveRecordFromQueue, accessCameraDriver, std::move(prms));
  t.join();
  MovableTimestampedType<PositionServiceRecord> movableTimestampedRecord = ftr.get();
  return movableTimestampedRecord;
}*/

void ImageTransformer::receiveRecord(std::promise<MovableTimestampedType<PositionServiceRecord>> &&recordPromise){
  printToConsole("ImageTransformer::receiveRecord called.");
  MovableTimestampedType<PositionServiceRecord> movableTimestampedRecord = getRecordFromQueue();
  PositionServiceRecord record = movableTimestampedRecord.getData();
  applyImageProcessingToRecord(record);
  movableTimestampedRecord.setData(record);
  recordPromise.set_value(movableTimestampedRecord);
}

void ImageTransformer::runInRunningState(){
  printToConsole("ImageTransformer::runInRunningState is called.");
  // monitor the queue length here and request reboot if the queue length does get too long!
}

void ImageTransformer::runInInitializingState(){
  printToConsole("ImageTransformer::runInInitializingState is called.");
  if(!ready){
    setBirdEyesTransformMatrix();
  }
}

void ImageTransformer::runInFreezedState(){
  printToConsole("ImageTransformer::runInFreezedState is called.");
}

void ImageTransformer::runInTerminatedState(){
  printToConsole("ImageTransformer::runInTerminatedState is called.");
  clearQueueOfRecords();
}

void ImageTransformer::manageStateSwitches(){
  while(true){
    if(currentState == initializing){
      printToConsole("ImageTransformer::manageStateSwitches is called, instance is waiting in state initializing.");
      runInInitializingState();
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
    if(currentState == running){
      printToConsole("ImageTransformer::manageStateSwitches, instance is in state running.");
      runInRunningState();
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
    if(currentState == terminated){
      printToConsole("ImageTransformer::manageStateSwitches is called, instance has reached state terminated.");
      runInTerminatedState();
      break;
    }
    if(currentState == freezed){
      printToConsole("ImageTransformer::manageStateSwitches is called, instance is waiting in state freezed.");
      runInFreezedState();
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
  }
}