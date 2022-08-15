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
#include <eigen3/Eigen/Core>


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
#define MEANVELOCITY 88 // km/h
#define VARIANCEVELOCITY 5 // km/h

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
  return 0;
}

void runWithVideo(const std::string fileName){
  /* resource initialization*/
  std::unique_ptr<PositionServer> srv(new PositionServer(Debuglevel::verbose));
  std::shared_ptr<VelocitySource> accessVelocitySource(new VelocitySource(MEANVELOCITY, VARIANCEVELOCITY));
  srv->mountVelocitySource(accessVelocitySource);
  std::shared_ptr<ImageSource> accessImageSource(new ImageSource(fileName));
  srv->mountImageSource(accessImageSource);
  //srv->runCameraCalibration(true);
  srv->initialize();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  srv->run();
  std::this_thread::sleep_for(std::chrono::milliseconds(10000));
  srv->terminate();
}

void runWithStaticImage(const std::string fileName){
  /* resource initialization*/
  std::shared_ptr<CameraServer> accessCamera(new CameraServer(Debuglevel::verbose));
  accessCamera->readCameraCalibrationDataFromFile();
  std::shared_ptr<ImageTransformer> accessTransformer(new ImageTransformer(Debuglevel::verbose));
  accessTransformer->setBirdEyesTransformMatrix();
  std::shared_ptr<PositionEstimator> accessEstimator(new PositionEstimator(Debuglevel::verbose));
  std::shared_ptr<VelocitySource> accessVelocitySource(new VelocitySource(MEANVELOCITY, VARIANCEVELOCITY));
  /* laod test image & create empty images to hold results */
  cv::Mat undistortedImage;
  cv::Mat gaussianBlurredImage;
  cv::Mat grayImage;
  cv::Mat birdEyesViewImage;
  cv::Mat binaryBirdEyesViewImage;
  cv::Mat binaryEdgesDetected;
  cv::Mat result;
  cv::Mat rawImage = cv::imread(fileName , cv::IMREAD_COLOR);
  /* more variables are needed */
  float velocityMeasurement;
  float rightDeviation;
  float leftDeviation;
  float leftAngle;
  float rightAngle;
  cv::Vec4i leftLaneLine;
  bool leftLaneDetected;
  cv::Vec4i rightLaneLine;
  bool rightLaneDetected;
  std::vector<cv::Vec4i> lines;
  Eigen::VectorXd x(4);
  float time = 0;
  float px;
  float py;
  float vx;
  float vy;
  float phi;
  /* camera driver's operations are here */
  accessCamera->undistortImage(rawImage, undistortedImage);
  /* image transformer's operations are here */
  accessTransformer->applyGausianBlurr(undistortedImage , gaussianBlurredImage);
  accessTransformer->convertToGrayImage(gaussianBlurredImage, grayImage);
  accessTransformer->convertToBirdEyesView(grayImage, birdEyesViewImage);
  accessTransformer->convertToBinaryImage(birdEyesViewImage, binaryBirdEyesViewImage);
  accessTransformer->detectEdges(binaryBirdEyesViewImage, binaryEdgesDetected);
  /* position estimator's operations are here */
  accessEstimator->getHoughLines(binaryEdgesDetected, lines);
  rightLaneDetected = accessEstimator->findRightLaneLineInHoughLines(lines, rightLaneLine, leftDeviation, leftAngle);
  leftLaneDetected = accessEstimator->findLeftLaneLineInHoughLines(lines, leftLaneLine, rightDeviation, rightAngle);
  /* prepare the result */
  result = birdEyesViewImage;
  if(leftLaneDetected){
    std::cout << "Left Lane detected: [ " << leftLaneLine[0] << " , " << leftLaneLine[1] << " ] to [ " << leftLaneLine[2] << " , " << leftLaneLine[3] << " ]" << std::endl;
    cv::line(result, cv::Point(leftLaneLine[0], leftLaneLine[1]), cv::Point(leftLaneLine[2], leftLaneLine[3]), cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
  }else{
    std::cout<<"Did not find a left lane line."<<std::endl;
    leftDeviation = -99;
    leftAngle = PI;
  }
  if(rightLaneDetected){
    std::cout << "Right Lane detected: [ " << rightLaneLine[0] << " , " << rightLaneLine[1] << " ] to [ " << rightLaneLine[2] << " , " << rightLaneLine[3] << " ]" << std::endl;
    cv::line(result, cv::Point(rightLaneLine[0], rightLaneLine[1]), cv::Point(rightLaneLine[2], rightLaneLine[3]), cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
  }else{
    std::cout<<"Did not find a right lane line."<<std::endl;
    rightDeviation = -99;
    rightAngle = PI;
  }
  /* run the filter operatiosn to get the estimate of the position*/
  float timestep = 0.1; 
  Measurement measurement;
  if(leftLaneDetected){
    measurement.deviation = leftDeviation;
    measurement.angle = leftAngle;
    measurement.velocity = MEANVELOCITY * 1000/3600;
  }else{
    if(rightLaneDetected){
      measurement.deviation = rightDeviation;
      measurement.angle = rightAngle;
      measurement.velocity = MEANVELOCITY * 1000/3600;
    }
  }
  accessEstimator->initializeKalmanFilter(); 
  for (size_t i=0; i<100; i++){
    accessEstimator->getStateVector(x);
    accessEstimator->predict(timestep);
    accessVelocitySource->getNextVelocityMeasurement(measurement.velocity);
    accessEstimator->update(measurement);
    /* feed quantities to trip recorder */
    px = x(0);
    py = x(1);
    vx = x(2);
    vy = x(3);
    if(vx > 0){
      phi = atan(vy/vx);
    }else{
      phi = 0.0;
    }
    accessEstimator->feedToTripRecorderRecords(time, px, py, vx, vy, phi, leftDeviation, leftAngle, rightDeviation, rightAngle);
    time = time + timestep;
  }
  std::string resultFileName = "result.txt";
  accessEstimator->saveTripRecorderRecordsToFile(resultFileName);
  cv::imshow("Bird Eye's View with Lane Lines detected", result);
  cv::waitKey(0);
}

Filetype getFileType(const std::string fileName){
  Filetype filetype;
  filetype = Filetype::unknown;
  std::string jpg = "jpg";
  std::string mp4 = "mp4";
  int length = fileName.length();
  std::string fileExtension = fileName.substr(length-3,length);
  if(fileName.length()>4){
    if(jpg.compare(fileExtension) == 0){filetype = Filetype::jpg;}
    if(mp4.compare(fileExtension) == 0){filetype = Filetype::mp4;}
  } 
  return filetype;
}

int main(int argcount, const char** argvalue){
  int flag;
  //cv::Mat image = cv::imread("SimpleRunwayTestImage.png" ,cv::IMREAD_COLOR); // sample image to test the transformation.
  cv::VideoCapture cap("video01.mp4"); // sample video to test the video-processing
  //flag = testCameraDriver();
  //flag = testImageTransformer();
  //flag = testPositionEstimator();
  //flag = testPositionServer();
  //flag = testLanePositionSensing();
  //flag = testEigen();
  
  std::string fileName;
  Filetype filetype;
  bool validFileTypeProvided;
  if(argcount > 1){
    fileName = argvalue[1];
    filetype = getFileType(fileName);
    if(filetype == Filetype::mp4){
      std::cout << "# From thread with id:" << std::this_thread::get_id() << ": Running simulation with video-file '" << fileName << "'"<< std::endl;
      validFileTypeProvided = true;
      runWithVideo(fileName);
    }
    if(filetype == Filetype::jpg){
      std::cout << "# From thread with id:" << std::this_thread::get_id() <<  ": Running simulation with static image '" << fileName << "'"<< std::endl;
      validFileTypeProvided = true;
      runWithStaticImage(fileName);
    }
  }else{
    std::cout << "# From thread with id:" << std::this_thread::get_id() <<  ": Error: Please provide relative path to video-file (mp4) or an image (jpg) as argument, e.g. './VBRE vidoe01.mp4' " << std::endl;
    return -1;
  }
  if(!validFileTypeProvided){
    std::cout << "# From thread with id:" << std::this_thread::get_id() <<  ": Error: Please provide relative path to video-file (mp4) or an image (jpg) as argument, e.g. './VBRE vidoe01.mp4' " << std::endl;
    return -1;
  }
  return 0;
}


