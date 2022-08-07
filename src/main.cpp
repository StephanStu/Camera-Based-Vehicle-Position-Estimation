#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string> 
#include <opencv2/core/types.hpp>
#include <thread>
#include <future>
#include <mutex>
#include <algorithm>  // std::for_each
#include <memory>
#include <ctime>


#include "Types.h"
#include "CameraServer.h"
#include "MovableTimestampedType.h"
#include "ImageTransformer.h"
#include "PositionEstimator.h"
#include "PositionServer.h"

using std::vector;
using std::cout;
using std::cerr;
using std::endl;
using std::string;

#define PI 3.14159265

int runHoughTransformationTest(cv::Mat image){
  cv::Mat grayImage; // used for the result of transfroming the RGB-Image to a grayscale-image
  cv::Mat edgesDetected; // used for the result of the Canny-Edge-Detection
  cv::Mat blurred;
  std::vector<cv::Vec4i> lines; // a vector to store lines of the image, using the cv::Vec4i-Datatype
  if(! image.data ) {
    std::cout <<  "Image not found or unable to open" << std::endl ;
    return -1;
  }else{
    /*
    Now convert this image to a gray image
    */
    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
    /*
    Apply a gaussian Blur
    */
    //cv::GaussianBlur(grayImage, image_blurred_with_3x3_kernel, cv::Size(3, 3), 0);
    cv::Size size(9,9);
    cv::GaussianBlur(grayImage, blurred, size, 0, 0);
    /*
    Run the canny-edge-detection 
    */
    cv::Canny(blurred, edgesDetected, 50, 200);
    /*
    Run the Hough-Transformation
    */
    cv::HoughLinesP(edgesDetected, lines, 1, CV_PI/180, 250, 100, 250);
    // Draw lines on the image
    for (size_t i=0; i<lines.size(); i++) {
      cv::Vec4i l = lines[i];
      cv::line(image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
    }

  }
  cv::namedWindow( "OpenCV Test Program", cv::WINDOW_AUTOSIZE );
  cv::imshow( "OpenCV Test Program", image );
  return 0;
}

int runVideoTest(cv::VideoCapture cap){
  // Create a VideoCapture object and open the input file
  // If the input is the web camera, pass 0 instead of the video file name
  cv::Mat frame;
  // Check if camera opened successfully
  if(!cap.isOpened()){
    std::cout << "Error opening video stream or file" << std::endl;
    return -1;
  }
  while(1){
    // Capture frame-by-frame
    cap >> frame;
    // If the frame is empty, break immediately
    if (frame.empty()){
      break;
    }
    // Display the resulting frame
    //imshow( "Frame", frame );
    // Run Hough Transformation and display frame
    runHoughTransformationTest(frame);
    // Press  ESC on keyboard to exit
    char c=(char)cv::waitKey(25);
    if(c==27){
      break;
    }
  }
  // When everything done, release the video capture object
  cap.release();
  // Closes all the frames
  cv::destroyAllWindows();
  return 0;
}

int testMovableTimestampedType(){
  PositionServiceRecord myRec, myNewRec, res;
  myRec.rawImage = cv::imread("SimpleRunwayTestImage.png" ,cv::IMREAD_COLOR);
  myRec.binaryBirdEyesViewImage = cv::imread("test/test01.jpg" ,cv::IMREAD_COLOR);
  myRec.distanceToLeftLane = 1.0;
  myRec.distanceToRightLane = 1.5;;
  myRec.deviation = -0.5;
  
  MovableTimestampedType<PositionServiceRecord> cmplxObj1(myRec, Debuglevel::verbose);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  myNewRec.rawImage = cv::imread("test/test02.jpg" ,cv::IMREAD_COLOR);
  myNewRec.binaryBirdEyesViewImage = cv::imread("test/test03.jpg" ,cv::IMREAD_COLOR);
  myNewRec.distanceToLeftLane = 0.0;
  myNewRec.distanceToRightLane = 0.5;;
  myNewRec.deviation = 1.5;
  std::cout << "# ob1 age: " << cmplxObj1.getAge() << std::endl;
  
  MovableTimestampedType<PositionServiceRecord> cmplxObj2(myNewRec, Debuglevel::verbose);
  cmplxObj1 = std::move(cmplxObj2);
  std::cout << "# ob1 age: " << cmplxObj1.getAge() << std::endl;
  res = cmplxObj1.getData();
  std::cout << res.binaryBirdEyesViewImage.size() << std::endl;
  
  return 0;
}

int testCameraDriver(){
  cv::Size myImageSize;
  PositionServiceRecord record;
  MovableTimestampedType<PositionServiceRecord> timestampedRecord(record, Debuglevel::verbose);
  
  std::shared_ptr<CameraServer> accessCamera(new CameraServer(Debuglevel::verbose));
  //accessCamera->calibrate();
  //accessCamera->writeCameraCalibrationDataToFile();
  accessCamera->getImageSize(myImageSize);
  accessCamera->readCameraCalibrationDataFromFile();
  accessCamera->getImageSize(myImageSize);
  accessCamera->createBlackRecord(timestampedRecord);
  PositionServiceRecord myRecord = timestampedRecord.getData();
  cv::namedWindow( "OpenCV Test Program", cv::WINDOW_AUTOSIZE );
  cv::imshow( "OpenCV Test Program", myRecord.rawImage );
  cv::waitKey(0);
  
  
  //accessCameraDriver->isReady();
  //accessCameraDriver->run();
  //MovableTimestampedType<PositionServiceRecord> cmplxObj1 = accessCameraDriver->getRecord(); // stuck, because no image is here
  return 0;
}

int testImageTransformer(){
  std::shared_ptr<CameraServer> accessCamera(new CameraServer(Debuglevel::verbose));
  std::shared_ptr<ImageTransformer> accessImageTransformer(new ImageTransformer(Debuglevel::verbose));
  accessImageTransformer->mountCameraServer(accessCamera);
  return 0;
}

int testPositionEstimator(){
  std::shared_ptr<CameraServer> accessCamera(new CameraServer(Debuglevel::verbose));
  std::shared_ptr<ImageTransformer> accessImageTransformer(new ImageTransformer(Debuglevel::verbose));
  std::shared_ptr<PositionEstimator> accessPositionEstimator(new PositionEstimator(Debuglevel::verbose));
  accessImageTransformer->mountCameraServer(accessCamera);
  accessPositionEstimator->mountImageTransformer(accessImageTransformer);
  return 0;
}

int testPositionServer(){
  std::unique_ptr<PositionServer> srv(new PositionServer(Debuglevel::verbose));
  //srv->runCameraCalibration(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  srv->initialize();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  srv->run();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  //srv->freeze();
  srv->terminate();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  srv->initialize();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  srv->run();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  //srv->freeze();
  srv->terminate();
  return 0;
}

void showImage(cv::Mat img){
  int delay = 0;
  cv::imshow("Your Image", img);
  cv::waitKey(delay);
}

cv::Mat mergeImages(const cv::Mat& src1, const cv::Mat& src2){
  int rows = cv::max(src1.rows, src2.rows);
  int cols = src1.cols + src2.cols;
  cv::Mat dest = cv::Mat::zeros(cv::Size(cols, rows), CV_8UC3);  
  std::vector<cv::Mat> imgs_for_concat_0 = {src1, src2};
  int start_col = 0;
  int start_row = 0;
  int curr_cols;
  int curr_rows;
  for (size_t i=0; i < imgs_for_concat_0.size(); ++i){
    curr_cols = imgs_for_concat_0[i].cols;
    curr_rows = imgs_for_concat_0[i].rows;
    imgs_for_concat_0[i].copyTo(dest(cv::Rect(start_col, start_row, curr_cols, curr_rows)));
    start_col += curr_cols;
  }
  return dest;
}

bool isVertical( cv::Vec4i line){
  if(line[0] == line[2]){
    //std::cout << "Found a vertical line." << std::endl;
    return true;
  }else{
    return false;
  }
}

bool findRightLaneLineInHoughLines(const std::vector<cv::Vec4i>& houghLines, cv::Vec4i& laneLine, float& deviation, float& angle){
  float laneWidth = 3.6576; // width of lane in metres
  float metresPerPixel = 0.02147443; // in x-direction of the image coordinates: How many metres per pixel?
  float lowerAngleThreshold = - 15.0 * PI / 180; // lower bound for a reasonable road vehicle angle crusing on highway
  float upperAngleThreshold = 15.0 * PI / 180; // lower bound for a reasonable road vehicle angle crusing on highway
  float deviationThreshold = 0.9144; // threshold for deviation from center line, quarter of road width
  int centerLineXCoordinate = 250;
  int yMax = 600; // bottom of the image
  int xMax = 500; // width of the image
  int closestToCenterXCoordinate = xMax;
  float slope;
  float slopeLowerBound; // bound for slope of lane line in x-y-coordintes of cv::Mat-Image; lane lines must be very steep - have large m - otherwise they are not lane lines but artifacts 
  float y0; // this is the value in the linear form y = y0 + slope * x; equation represnts a lane line
  float xStar; // the x value where the linear form hits the yMax (the bottom of the image)
  bool success = false;
  for (size_t i=0; i<houghLines.size(); i++) {
    cv::Vec4i line = houghLines[i];
    if(isVertical(line)){
      if((line[0] > centerLineXCoordinate) && (line[0] < closestToCenterXCoordinate)){
        closestToCenterXCoordinate = line[0];
        laneLine[0] = line[0];
        laneLine[2] = line[2];
        laneLine[1] = yMax;
        laneLine[3] = 0;
        angle = 0.0;
        deviation = (laneWidth / 2) - ((line[0] - centerLineXCoordinate) * metresPerPixel);
        /* 
           static check if measurements are within their bounds; otherwise
           the image processing is corrupted with artifacts or lines are not
           interpreted in the way the should, e.g. fences are mistaken as 
           lane lines. Way of checking is
           
           -deviationThreshold < deviation < deviationThreshold
           
           -lowerAngleThreshold < angle < upperAngleThreshold
        */
        if(abs(deviation) < deviationThreshold){
          success = true;
        }
      }
    }else{
      slope = (line[3]-line[1]) / (line[2]-line[0]);
      y0 = line[1] - (slope * line[0]);
      xStar = (yMax - y0) / slope;
      if((int(xStar) > centerLineXCoordinate) && (int(xStar) < closestToCenterXCoordinate)){
        closestToCenterXCoordinate = int(xStar);
        laneLine[0] = int(xStar);
        laneLine[1] = yMax;
        laneLine[3] = 0;
        laneLine[2] = int(-y0 / slope);
        angle = atan(yMax/(laneLine[2] - laneLine[0]));
        if(angle < 0){
          angle = 0.0 - angle;
          deviation = (laneWidth / 2) - ((line[0] - centerLineXCoordinate) * sin(angle) * metresPerPixel);
          angle = angle - (90.0 * PI / 180.0);
        }else{
          deviation = (laneWidth / 2) - ((line[0] - centerLineXCoordinate) * sin(angle) * metresPerPixel);
          angle = (90.0 * PI / 180.0) - angle;
        }
        /* 
           static check if measurements are within their bounds; otherwise
           the image processing is corrupted with artifacts or lines are not
           interpreted in the way the should, e.g. fences are mistaken as 
           lane lines. Way of checking is
           
           -deviationThreshold < deviation < deviationThreshold
           
           -lowerAngleThreshold < angle < upperAngleThreshold
        */
        if((angle > lowerAngleThreshold) && (angle < upperAngleThreshold)){
          if(abs(deviation) < deviationThreshold){
            success = true;
          }
        }
        //std::cout << "Detected right lane line with slope " << slope << " and offset " << y0 << " and angle " << angle * 180.0/PI <<  std::endl;
        //std::cout << "Updated closest to be equal to be " << closestToCenterXCoordinate << std::endl;
      }
    }
  }
  if(success){
    std::cout << "Detected right lane line with slope = " << slope << " and offset = " << y0 << ". Hence angle = " << angle * 180.0/PI << " deg and distance to center lane = " << deviation * 100 << " cm" << std::endl;
  }
  return success;
}

bool findLeftLaneLineInHoughLines(const std::vector<cv::Vec4i>& houghLines, cv::Vec4i& laneLine, float& deviation, float& angle){
  float laneWidth = 3.6576; // width of lane in metres
  float metresPerPixel = 0.0213895; // in x-direction of the image coordinates: How many metres per pixel? Taken from test02.jpg
  float lowerAngleThreshold = - 15.0 * PI / 180; // lower bound for a reasonable road vehicle angle crusing on highway
  float upperAngleThreshold = 15.0 * PI / 180; // lower bound for a reasonable road vehicle angle crusing on highway
  float deviationThreshold = 0.9144; // threshold for deviation from center line, quarter of road width
  int closestToCenterXCoordinate = 0;
  int centerLineXCoordinate = 250;
  int yMax = 600; // bottom of the image
  int xMax = 500; // width of the image
  float slope;
  float slopeLowerBound; // bound for slope of lane line in x-y-coordintes of cv::Mat-Image; lane lines must be very steep - have large m - otherwise they are not lane lines but artifacts 
  float y0; // this is the value in the linear form y = y0 + slope * x; equation represnts a lane line
  float xStar; // the x value where the linear form hits the yMax (the bottom of the image)
  bool success = false;
  for (size_t i=0; i<houghLines.size(); i++) {
    cv::Vec4i line = houghLines[i];
    if(isVertical(line)){
      if((line[0] < centerLineXCoordinate) && (line[0] > closestToCenterXCoordinate)){
        closestToCenterXCoordinate = line[0];
        laneLine[0] = line[0];
        laneLine[2] = line[2];
        laneLine[1] = yMax;
        laneLine[3] = 0;
        angle = 0.0;
        deviation = ((centerLineXCoordinate - line[0]) * metresPerPixel) - (laneWidth / 2);
        /* 
           static check if measurements are within their bounds; otherwise
           the image processing is corrupted with artifacts or lines are not
           interpreted in the way the should, e.g. fences are mistaken as 
           lane lines. Way of checking is
           
           -deviationThreshold < deviation < deviationThreshold
           
           -lowerAngleThreshold < angle < upperAngleThreshold
        */
        if(abs(deviation) < deviationThreshold){
          success = true;
        }
      }
    }else{
      slope = (line[3]-line[1]) / (line[2]-line[0]);
      y0 = line[1] - (slope * line[0]);
      xStar = (yMax - y0) / slope;
      if((int(xStar) < centerLineXCoordinate) && (int(xStar) > closestToCenterXCoordinate)){
        closestToCenterXCoordinate = int(xStar);
        laneLine[0] = int(xStar);
        laneLine[1] = yMax;
        laneLine[3] = 0;
        laneLine[2] = int(-y0 / slope);
        angle = atan(yMax/(laneLine[2] - laneLine[0]));
        if(angle < 0){
          angle = 0.0 - angle;
          deviation = ((centerLineXCoordinate - line[0]) * sin(angle) *  metresPerPixel) - (laneWidth / 2);
          angle = angle - (90.0 * PI / 180.0);
        }else{
          deviation = ((centerLineXCoordinate - line[0]) * sin(angle) *  metresPerPixel) - (laneWidth / 2);
          angle = (90.0 * PI / 180.0) - angle;
        }
        /* 
           static check if measurements are within their bounds; otherwise
           the image processing is corrupted with artifacts or lines are not
           interpreted in the way the should, e.g. fences are mistaken as 
           lane lines. Way of checking is
           
           -deviationThreshold < deviation < deviationThreshold
           
           -lowerAngleThreshold < angle < upperAngleThreshold
        */
        if((angle > lowerAngleThreshold) && (angle < upperAngleThreshold)){
          if(abs(deviation) < deviationThreshold){
            success = true;
          }
        }
        //std::cout << "Detected left lane line with slope " << slope << " and offset " << y0 << " and angle " << angle * 180.0/PI << std::endl;
        //std::cout << "Updated closest to be equal to be " << closestToCenterXCoordinate << std::endl;
      }
    }
  }
  if(success){
    std::cout << "Detected left lane line with slope = " << slope << " and offset = " << y0 << ". Hence angle = " << angle * 180.0/PI << " deg and distance to center lane = " << deviation * 100 << " cm" << std::endl;
  }
  return success;
}

int testLanePositionSensing(){
  /* resource initialization*/
  std::shared_ptr<CameraServer> accessCamera(new CameraServer(Debuglevel::verbose));
  accessCamera->readCameraCalibrationDataFromFile();
  std::shared_ptr<ImageTransformer> accessTransformer(new ImageTransformer(Debuglevel::verbose));
  accessTransformer->setBirdEyesTransformMatrix();
  std::shared_ptr<PositionEstimator> accessEstimator(new PositionEstimator(Debuglevel::verbose));
  /* laod test image & create empty images to hold results */
  cv::Mat undistortedImage;
  cv::Mat gaussianBlurredImage;
  cv::Mat grayImage;
  cv::Mat birdEyesViewImage;
  cv::Mat binaryBirdEyesViewImage;
  cv::Mat binaryEdgesDetected;
  cv::Mat result;
  cv::Mat rawImage = cv::imread("test/test05.jpg" , cv::IMREAD_COLOR);
  /* camera driver's operations are here */
  accessCamera->undistortImage(rawImage, undistortedImage);
  /* image transformer's operations are here */
  accessTransformer->applyGausianBlurr(undistortedImage , gaussianBlurredImage);
  accessTransformer->convertToGrayImage(gaussianBlurredImage, grayImage);
  accessTransformer->convertToBirdEyesView(grayImage, birdEyesViewImage);
  accessTransformer->convertToBinaryImage(birdEyesViewImage, binaryBirdEyesViewImage);
  accessTransformer->detectEdges(binaryBirdEyesViewImage, binaryEdgesDetected);
  /* position estimator's operations are here */
  std::vector<cv::Vec4i> lines;
  accessEstimator->getHoughLines(binaryEdgesDetected, lines);
  /* show the result */
  
  result = cv::Mat::zeros(binaryEdgesDetected.size(), CV_8UC3);
  if(lines.size()>0){
    for (size_t i=0; i<lines.size(); i++) {
    //for (size_t i=0; i<1; i++) {  
      cv::Vec4i l = lines[i];
      cv::line(result, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
      std::cout << "Adding a line with points: [ " << l[0] << " , " << l[1] << " ] to [ " << l[2] << " , " << l[3] << " ]" << std::endl;
    }
  }
  /* compute the "sensor readings": distances and angles */
  /* find the line that is closest to center but left of center = x < 250 */
  float rightDeviation;
  float leftDeviation;
  float leftAngle;
  float rightAngle;
  cv::Vec4i leftLaneLine;
  bool leftLaneDetected;
  cv::Vec4i rightLaneLine;
  bool rightLaneDetected;
  
  result = birdEyesViewImage;
  
  rightLaneDetected = findRightLaneLineInHoughLines(lines, rightLaneLine, leftDeviation, leftAngle);
  leftLaneDetected = findLeftLaneLineInHoughLines(lines, leftLaneLine, rightDeviation, rightAngle);
  if(leftLaneDetected){
    std::cout << "Left Lane detected: [ " << leftLaneLine[0] << " , " << leftLaneLine[1] << " ] to [ " << leftLaneLine[2] << " , " << leftLaneLine[3] << " ]" << std::endl;
    cv::line(result, cv::Point(leftLaneLine[0], leftLaneLine[1]), cv::Point(leftLaneLine[2], leftLaneLine[3]), cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
  }else{
    std::cout<<"Did not find a left lane line."<<std::endl;
  }
  if(rightLaneDetected){
    std::cout << "Right Lane detected: [ " << rightLaneLine[0] << " , " << rightLaneLine[1] << " ] to [ " << rightLaneLine[2] << " , " << rightLaneLine[3] << " ]" << std::endl;
    cv::line(result, cv::Point(rightLaneLine[0], rightLaneLine[1]), cv::Point(rightLaneLine[2], rightLaneLine[3]), cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
  }else{
    std::cout<<"Did not find a right lane line."<<std::endl;
  }
  
  
  cv::Size mat_size(500,600);
  cv::Mat blue(mat_size, CV_8UC3, cv::Scalar(255,0,0));
  cv::Mat green(mat_size, CV_8UC3, cv::Scalar(0,255,0));
  
  //cv::Mat birdEyesViewAndLaneLines = mergeImages(birdEyesViewImage, result);
  
  /* find the line that is closest to center but right of center = x > 250 */
  
  //int rows = cv::max(rawImage.rows, binaryEdgesDetcted.rows);
  //int cols = rawImage.cols + binaryEdgesDetcted.cols;
  //cv::Mat result(rows, cols,  CV_8UC3);
  //rawImage.copyTo(result(cv::Rect(0, 0, rawImage.cols, rawImage.rows)));
  //binaryEdgesDetcted.copyTo(result(cv::Rect(rawImage.cols, 0, binaryEdgesDetcted.cols, binaryEdgesDetcted.rows)));
  showImage(result);
  
  //int rows = cv::max(birdEyesViewImage.rows, binaryEdgesDetected.rows);
  //int cols = birdEyesViewImage.cols + binaryEdgesDetected.cols;
  //cv::Mat merged = cv::Mat::zeros(cv::Size(cols, rows), CV_8UC3);
  
  return 0;
}



int main(){
  int flag;
  cv::Mat image = cv::imread("SimpleRunwayTestImage.png" ,cv::IMREAD_COLOR); // sample image to test the transformation.
  cv::VideoCapture cap("testVideo002.mp4"); // sample video to test the video-processing
  //flag = testCameraDriver();
  //flag = testImageTransformer();
  //flag = testPositionEstimator();
  //flag = testPositionServer();
  flag = testLanePositionSensing();
  return 0;
}

