#include "ImageTransformer.h"

ImageTransformer::ImageTransformer(){
  debugLevel = Debuglevel::none;
  printToConsole("ImageTransformer::ImageTransformer called.");
  setBirdEyesTransformMatrix();
}

ImageTransformer::ImageTransformer(Debuglevel imageTransformerDebugLevel){
  debugLevel = imageTransformerDebugLevel;
  printToConsole("ImageTransformer::ImageTransformer called.");
  setBirdEyesTransformMatrix();
}
    
void ImageTransformer::run(){
  printToConsole("ImageTransformer::run called.");
  threads.emplace_back(std::thread(&ImageTransformer::transformAndStoreImage, this));
}

void ImageTransformer::convertToBinaryImage(cv::Mat& source, cv::Mat& destination){
  printToConsole("ImageTransformer::convertToBinaryImage called.");
}

void ImageTransformer::convertToBirdEyesView(cv::Mat& source, cv::Mat& destination){
  printToConsole("ImageTransformer::convertToBirdEyesView called.");
}

void ImageTransformer::convertToGrayImage(cv::Mat& source, cv::Mat& destination){
  printToConsole("ImageTransformer::convertToGrayImage called.");
}

void ImageTransformer::detectEdges(cv::Mat& source, cv::Mat& destination){
  printToConsole("ImageTransformer::detectEdges called.");
}
    
void ImageTransformer::applyGausianBlurr(cv::Mat& source, cv::Mat& destination){
  printToConsole("ImageTransformer::applyGausianBlurr called.");
}
    
void ImageTransformer::maskBirdEyesView(cv::Mat& source, cv::Mat& destination){
  printToConsole("ImageTransformer::maskBirdEyesView called.");
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
}

void ImageTransformer::mountCamerDriver(std::shared_ptr<CameraDriver> pointerToCameraDriver){
  printToConsole("ImageTransformer::mountCameraDriver called, instance of CamerDriver is mounted via saving the shared pointer.");
  accessCameraDriver = pointerToCameraDriver;
}

cv::Mat ImageTransformer::getImageFromMountedCameraDriver(){
  printToConsole("ImageTransformer::getImageFromMountedCameraDriver called, pulling an image with promise-future-mechanism.");
  std::promise<cv::Mat> prms;
  std::future<cv::Mat> ftr = prms.get_future();
  std::thread t(&CameraDriver::receiveImageFromQueue, accessCameraDriver, std::move(prms));
  t.join();
  cv::Mat image = ftr.get();
  return image;
}

void ImageTransformer::transformAndStoreImage(){
  while(true){
    if(currentState == initializing){
      printToConsole("ImageTransformer::transformAndStoreImage is called, instance is waiting in state initializing.");
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
    if(currentState == running){
      printToConsole("ImageTransformer::transformAndStoreImage is running and pulling images from camera driver into its own queue.");
      // ADD PULL FROM CAMERA DRIVER HERE AND TRANSFORMING
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
    if(currentState == terminated){
      printToConsole("ImageTransformer::transformAndStoreImage is called, instance has reached state terminated. Cleaning up & quitting.");
      clearQueue();
      break;
    }
    if(currentState == freezed){
      printToConsole("ImageTransformer::transformAndStoreImage is called, instance is waiting in state freezed.");
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
  }
}