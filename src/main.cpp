#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string> 
#include <opencv2/core/types.hpp>

#include "CameraDriver.h"
#include "MovableImageData.h"
#include "Types.h"

using std::vector;
using std::cout;
using std::cerr;
using std::endl;
using std::string;
 
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

int calibrateCamera(cv::Mat image){
  // Taken from https://github.com/oreillymedia/Learning-OpenCV-3_examples/blob/master/example_19-01.cpp
  int n_boards = 1;           
  float image_sf = 0.5f;      // image scaling factor
  int board_w = 9;
  int board_h = 6; 
  int board_n = board_w * board_h;
  cv::Size board_sz = cv::Size(board_w, board_h);
  cv::Size image_size = image.size();
  vector<vector<cv::Point2f>> image_points;
  vector<vector<cv::Point3f>> object_points;
  vector<cv::Point2f> corners;
  bool found = cv::findChessboardCorners(image, board_sz, corners);
  cout << "Searchign the chessboard corners:" << endl;
  cout << found << endl;
  drawChessboardCorners(image, board_sz, corners, found);
  /*cv::imshow("Calibration", image);
  cv::waitKey(0);
  cv::destroyWindow("Calibration");*/
  image_points.push_back(corners);
  object_points.push_back(vector<cv::Point3f>());
  vector<cv::Point3f> &opts = object_points.back();
  opts.resize(board_n);
  for (int j = 0; j < board_n; j++) {
    opts[j] = cv::Point3f(static_cast<float>(j / board_w), static_cast<float>(j % board_w), 0.0f);
  }
  cv::Mat intrinsic_matrix, distortion_coeffs;
  double err = cv::calibrateCamera(object_points, image_points, image_size, intrinsic_matrix, distortion_coeffs, cv::noArray(), cv::noArray(), cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT);
  cout << " *** DONE!\n\nReprojection error is " << err << "\n Saving results in CalibrationData.xml...\n";
  cv::FileStorage fs("calibrationData.xml", cv::FileStorage::WRITE);
  fs << "image_width" << image_size.width << "image_height" << image_size.height << "camera_matrix" << intrinsic_matrix << "distortion_coefficients" << distortion_coeffs;
  fs.release();
  // EXAMPLE OF LOADING THESE MATRICES BACK IN:
  fs.open("calibrationData.xml", cv::FileStorage::READ);
  cout << "\nimage width: " << static_cast<int>(fs["image_width"]);
  cout << "\nimage height: " << static_cast<int>(fs["image_height"]);
  cv::Mat intrinsic_matrix_loaded, distortion_coeffs_loaded;
  fs["camera_matrix"] >> intrinsic_matrix_loaded;
  fs["distortion_coefficients"] >> distortion_coeffs_loaded;
  cout << "\nintrinsic matrix:" << intrinsic_matrix_loaded;
  cout << "\ndistortion coefficients: " << distortion_coeffs_loaded << endl;
  cv::Mat map1, map2;
  cv::initUndistortRectifyMap(intrinsic_matrix_loaded, distortion_coeffs_loaded, cv::Mat(), intrinsic_matrix_loaded, image_size, CV_16SC2, map1, map2);
 // Just run the camera to the screen, now showing the raw and
 // the undistorted image.
 //
 cv::Mat undistortedImage;
 image = cv::imread("SimpleRunwayTestImage.png" ,cv::IMREAD_COLOR);
 cv::remap(image, undistortedImage, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
 //cv::imshow("Undistorted", undistortedImage);
 //cv::waitKey(0);
  
  
  // Bird Eye view starts here:
  
 cv::Point2f objPts[4], imgPts[4];
 objPts[0].x = 0; objPts[0].y = 0;
 objPts[1].x = board_w-1; objPts[1].y = 0;
 objPts[2].x = 0; objPts[2].y = board_h-1;
 objPts[3].x = board_w-1; objPts[3].y = board_h-1;
 imgPts[0] = corners[0];
 imgPts[1] = corners[board_w-1];
 imgPts[2] = corners[(board_h-1)*board_w];
 imgPts[3] = corners[(board_h-1)*board_w + board_w-1];
 cv::circle( image, imgPts[0], 9, cv::Scalar( 255, 0, 0), 3);
 cv::circle( image, imgPts[1], 9, cv::Scalar( 0, 255, 0), 3);
 cv::circle( image, imgPts[2], 9, cv::Scalar( 0, 0, 255), 3);
 cv::circle( image, imgPts[3], 9, cv::Scalar( 0, 255, 255), 3);
 //cv::drawChessboardCorners( image, board_sz, corners, found );
  cv::Mat H = cv::getPerspectiveTransform( objPts, imgPts );
 //cv::imshow( "Checkers", image );
 //  cv::waitKey(0);
  //cv::destroyWindow("Calibration");
  
 double Z = 25;
 cv::Mat birds_image;
 for(;;) { // escape key stops
   H.at<double>(2, 2) = Z;
   cv::warpPerspective( 
     image, // Source image
     birds_image, // Output image
     H, // Transformation matrix
     image.size(), // Size for output image
     cv::WARP_INVERSE_MAP | cv::INTER_LINEAR,
     cv::BORDER_CONSTANT,
     cv::Scalar::all(0) // Fill border with black
   );
   cv::imshow("Birds_Eye", birds_image);
   int key = cv::waitKey() & 255;
   if(key == 'u') Z += 0.5;
   if(key == 'd') Z -= 0.5;
   if(key == 27) break;
 }
 return 0;
}

int calibrate(){
  /*cv::Mat image;
  string path("calibration/");
  string file("calibration");
  string fileType(".jpg");  
  int number=12;
  string fileNumber = std::to_string(number);
  string imageFile = path + file + fileNumber + fileType;
  cout << imageFile << endl;
  //image = cv::imread("calibration/calibration01.jpg" ,cv::IMREAD_COLOR);
  image = cv::imread(imageFile ,cv::IMREAD_COLOR);
  cout << image.size() << endl;
  cv::imshow("Birds_Eye", image);
  cv::waitKey(0);
  */
  Debuglevel debuglevel = Debuglevel::verbose;
  CameraDriver cameraDriver(debuglevel);
  cameraDriver.calibrate();
  cv::Mat TestMat1, TestMat2;
  cameraDriver.getIntrinsicMatrix(TestMat1);
  cameraDriver.getDistortionCoefficients(TestMat2); 
  MovableImageData movableImage01(cv::imread("SimpleRunwayTestImage.png" ,cv::IMREAD_COLOR), debuglevel);
  MovableImageData movableImage02(cv::imread("SimpleRunwayTestImage.png" ,cv::IMREAD_COLOR), debuglevel);
  MovableImageData movableImage03(cv::imread("SimpleRunwayTestImage.png" ,cv::IMREAD_COLOR), debuglevel);
  
  unsigned long int myLongInt;
  
  movableImage02.getCounter(myLongInt);
  movableImage02.getIdentifier(myLongInt);
  
  cv::Mat imageFromCamera;
  cameraDriver.getRawImage(imageFromCamera);
  MovableImageData fromCamera(imageFromCamera, debuglevel);
  fromCamera.getCounter(myLongInt);
  fromCamera.getIdentifier(myLongInt);
  
  
  return 0;
}

int drawpoly(){  
    cv::Mat img(500, 500, CV_8U, cv::Scalar(0));  
  
    cv::Point root_points[1][4];  
    root_points[0][0] = cv::Point(215,220);  
    root_points[0][1] = cv::Point(460,225);  
    root_points[0][2] = cv::Point(466,450);  
    root_points[0][3] = cv::Point(235,465);  
  
    const cv::Point* ppt[1] = {root_points[0]};  
    int npt[] = {4};  
    //cv::Scalar scale(255);
    //cv::polylines(img, ppt, npt, 1, 1, scale, 1, 8, 0);  
    polylines(img, ppt, npt, 1, 1, cv::Scalar(255),1,8,0);  
    cv::imshow("Test", img);  
    cv::waitKey();  
    cv::fillPoly(img, ppt, npt, 1, cv::Scalar(255));  
    cv::imshow("Test", img);  
    cv::waitKey(); 
    return 0;
}  

int main(){
  int flag;
  cv::Mat image; // used to hold an RGB-image
  image = cv::imread("SimpleRunwayTestImage.png" ,cv::IMREAD_COLOR); // sample image to test the transformation.
  cv::VideoCapture cap("testVideo002.mp4"); // sample video to test the video-processing
  //flag = runHoughTransformationTest(image);  cv::waitKey(0);
  //flag = runVideoTest(cap);
  //flag = calibrateCamera(cv::imread("checkerboard9x6.png" ,cv::IMREAD_COLOR));
  // flag = calibrate();
  flag = drawpoly();
  return 0;
}