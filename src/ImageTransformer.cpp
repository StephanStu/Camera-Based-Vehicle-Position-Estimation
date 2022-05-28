#include "ImageTransformer.h"

ImageTransformer::ImageTransformer(Debuglevel imageTransformerDebugLevel){
  debugLevel = imageTransformerDebugLevel;
  printToConsole("ImageTransformer::ImageTransformer called.");
  setBirdEyesTransformMatrix();
}

ImageTransformer::ImageTransformer(){
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

void ImageTransformer::transformAndStoreImage(){
  while(true){
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    printToConsole("ImageTransformer::transformAndStoreImage is still running.");
  }
}