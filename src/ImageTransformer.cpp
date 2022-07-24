#include "ImageTransformer.h"

ImageTransformer::ImageTransformer(Debuglevel imageTransformerDebugLevel)  : cameraServerIsMounted(false), birdsEyeTransformationMatrixIsReady(false) {
  debugLevel = imageTransformerDebugLevel;
  printToConsole("ImageTransformer::ImageTransformer called.");
}

ImageTransformer::ImageTransformer() : cameraServerIsMounted(false), birdsEyeTransformationMatrixIsReady(false) {
  debugLevel = Debuglevel::none;
}

void ImageTransformer::run(){
  printToConsole("ImageTransformer::run called.");
  threads.emplace_back(std::thread(&ImageTransformer::manageStateSwitches, this));
}

void ImageTransformer::mountCameraServer(std::shared_ptr<CameraServer> pointerToCameraServer){
  printToConsole("ImageTransformer::mountCameraServer called, instance of CameraServer is mounted by keeping the shared pointer.");
  accessCameraServer = pointerToCameraServer;
  cameraServerIsMounted = true;
}

void ImageTransformer::manageStateSwitches(){
  while(true){
    if(currentState == initializing){
      printToConsole("ImageTransformer::manageStateSwitches is called, instance is waiting in state initializing.");
      runInInitializingState();
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
    if(currentState == running){
      if(ready){
        std::string message = "ImageTransformer::manageStateSwitches is called: Instance is in state running with isCurrent = " + std::to_string(isCurrent) + ".";
        printToConsole(message);
        runInRunningState();
      }else{
        printToConsole("ImageTransformer::manageStateSwitches is called: Instance is not ready & calling the initializing routine.");
        runInInitializingState();
      }
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

void ImageTransformer::runInRunningState(){
  /* get the record from :CameraServer */
  if(newRecordIsAvailable()){
    std::promise<MovableTimestampedType<PositionServiceRecord>> prms;
    std::future<MovableTimestampedType<PositionServiceRecord>> ftr = prms.get_future();
    std::thread t(&CameraServer::sendRecord, accessCameraServer, std::move(prms));
    t.join();
    MovableTimestampedType<PositionServiceRecord> movableTimestampedRecord = std::move(ftr.get());
    /* process images and update the record */
    PositionServiceRecord newRecord = movableTimestampedRecord.getData();
    applyImageTransformations(newRecord);
    movableTimestampedRecord.setData(newRecord);
    /* save the updated record in the member variable by "moving" it (in order to keep the timestamp!) */
    std::unique_lock<std::mutex> uniqueLock(protection);
    record = std::move(movableTimestampedRecord);
    isCurrent = true;
    uniqueLock.unlock();
  }else{
    printToConsole("ImageTransformer::runInRunningState called: Wainting for :CameraServer to update it's record.");
  }
}

void ImageTransformer::runInInitializingState(){
  isCurrent = false;
  ready = birdsEyeTransformationMatrixIsReady && cameraServerIsMounted;
  if(!ready){
    if(!birdsEyeTransformationMatrixIsReady){
      printToConsole("ImageTransformer::runInInitializingState is called but Bird Eye's Transformation Matrix is not computed yet.");
      setBirdEyesTransformMatrix();
    }
    if(!cameraServerIsMounted){
      printToConsole("ImageTransformer::runInInitializingState is called but :CameraServer is not mounted yet.");
    }
  }
}

void ImageTransformer::runInFreezedState(){
  isCurrent = false;
  //cv::namedWindow( "Bird eye's view image from :ImageTransformer", cv::WINDOW_AUTOSIZE);
  //cv::imshow( "Bird eye's view image from :ImageTransformer", record.getData().birdEyesViewImage);
  //cv::waitKey(0);
}

void ImageTransformer::runInTerminatedState(){
  isCurrent = false;
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
  birdsEyeTransformationMatrixIsReady = true;
}

void ImageTransformer::applyImageTransformations(PositionServiceRecord& record){
  printToConsole("ImageTransformer::applyImageTransformations called.");
  cv::Mat undistortedImage = record.undistortedImage;
  cv::Mat gaussianBlurredImage;
  cv::Mat grayImage;
  cv::Mat birdEyesViewImage;
  cv::Mat binaryBirdEyesViewImage;
  cv::Mat binaryEdgesDetcted;
  applyGausianBlurr(undistortedImage , gaussianBlurredImage);
  convertToGrayImage(gaussianBlurredImage, grayImage);
  convertToBirdEyesView(grayImage, birdEyesViewImage);
  /* maskBirdEyesView() goes here if necessary */
  convertToBinaryImage(birdEyesViewImage, binaryBirdEyesViewImage);
  detectEdges(binaryBirdEyesViewImage, binaryEdgesDetcted);
  record.birdEyesViewImage = birdEyesViewImage;
  record.binaryBirdEyesViewImage = binaryEdgesDetcted;
}

bool ImageTransformer::newRecordIsAvailable(){
  return accessCameraServer->isCurrent;
}