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
#include "CameraDriver.h"
#include "ImageTransformer.h"
#include "MovableTimestampedType.h"
#include "PositionService.h"

using std::vector;
using std::cout;
using std::cerr;
using std::endl;
using std::string;

#define PI 3.1415926

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
  std::shared_ptr<CameraDriver> accessCameraDriver(new CameraDriver(Debuglevel::verbose));
  accessCameraDriver->calibrate();
  if (accessCameraDriver->recordIsCurrent()){
    std::cout << "# Current." << std::endl;
  } else {
    std::cout << "# Not current." << std::endl;
  }
  accessCameraDriver->isReady();
  accessCameraDriver->run();
  //MovableTimestampedType<PositionServiceRecord> cmplxObj1 = accessCameraDriver->getRecord(); // stuck, because no image is here
  return 0;
}

int testPositionService(){
  PositionServiceRecord record;
  std::vector<cv::Vec4i> lines; // a vector to store lines of the image, using the cv::Vec4i-Datatype
  PositionService positionService(Debuglevel::verbose);
  positionService.initialize();
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  std::cout << "##############################################" << std::endl;
  positionService.run();
  std::this_thread::sleep_for(std::chrono::milliseconds(10000));
  std::cout << "##############################################" << std::endl;
  positionService.terminate();
  positionService.getRecord(record);
  cv::imshow( "Raw Image", record.rawImage);
  cv::imshow( "Binary Bird Eye's View", record.binaryBirdEyesViewImage);
  /*
  threshold: The minimum number of intersections to "*detect*" a line
  minLineLength: The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
  maxLineGap: The maximum gap between two points to be considered in the same line.
  */
  cv::HoughLinesP(record.binaryBirdEyesViewImage, lines, 1, CV_PI/180, 100, 25, 25);
  cv::Mat output = cv::Mat::zeros(record.binaryBirdEyesViewImage.size(), CV_8UC3);
    // Draw lines on the image
    for (size_t i=0; i<lines.size(); i++) {
      cv::Vec4i l = lines[i];
      cv::line(output, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
    }
  cv::imshow( "Result", output);
  
  cv::waitKey(0);
  return 0;
}

int main(){
  int flag;
  cv::Mat image = cv::imread("SimpleRunwayTestImage.png" ,cv::IMREAD_COLOR); // sample image to test the transformation.
  cv::VideoCapture cap("testVideo002.mp4"); // sample video to test the video-processing
  //flag = testCameraDriver();
  flag = testPositionService();
  return 0;
}

