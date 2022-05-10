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
#include "ImageServer.h"
#include "Types.h"

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

int transform2BirdsEyeView(){
  /* Luiteture cited https://www.researchgate.net/publication/224195999_Distance_determination_for_an_automobile_environment_using_Inverse_Perspective_Mapping_in_OpenCV/link/00b4951c994745bf6b000000/download
  https://gist.github.com/anujonthemove/7b35b7c1e05f01dd11d74d94784c1e58
  */
  
  
  // mat container to receive images
  cv::Mat source, destination;
  source = cv::imread("SimpleRunwayTestImage.png" ,cv::IMREAD_COLOR); 
  int alpha_ = 90, beta_ = 90, gamma_ = 90;
  int f_ = 10, dist_ = 10;
  int frameWidth = 640; 
  int frameHeight = 480;
  cv::namedWindow("Result", 1);
  cv::resize(source, source, cv::Size(frameWidth, frameHeight));
  double focalLength, dist, alpha, beta, gamma; 
  alpha =((double)alpha_ -90) * PI/180;
  beta =((double)beta_ -90) * PI/180;
  gamma =((double)gamma_ -90) * PI/180;
  focalLength = (double)f_;
  dist = (double)dist_;
  cv::Size image_size = source.size();
  double w = (
              double)image_size.width, h = (double)image_size.height;
  // Projecion matrix 2D -> 3D
  cv::Mat A1 = (cv::Mat_<float>(4, 3)<< 
			1, 0, -w/2,
			0, 1, -h/2,
			0, 0, 0,
			0, 0, 1 );

	
	// Rotation matrices Rx, Ry, Rz
	cv::Mat RX = (cv::Mat_<float>(4, 4) << 
			1, 0, 0, 0,
			0, cos(alpha), -sin(alpha), 0,
			0, sin(alpha), cos(alpha), 0,
			0, 0, 0, 1 );
	cv::Mat RY = (cv::Mat_<float>(4, 4) << 
			cos(beta), 0, -sin(beta), 0,
			0, 1, 0, 0,
			sin(beta), 0, cos(beta), 0,
			0, 0, 0, 1	);
	cv::Mat RZ = (cv::Mat_<float>(4, 4) << 
			cos(gamma), -sin(gamma), 0, 0,
			sin(gamma), cos(gamma), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1	);
	// R - rotation matrix
	cv::Mat R = RX * RY * RZ;
	// T - translation matrix
	cv::Mat T = (cv::Mat_<float>(4, 4) << 
			1, 0, 0, 0,  
			0, 1, 0, 0,  
			0, 0, 1, dist,  
			0, 0, 0, 1); 
	// K - intrinsic matrix 
    cv::Mat K = (cv::Mat_<float>(3, 4) << 
			focalLength, 0, w/2, 0,
			0, focalLength, h/2, 0,
			0, 0, 1, 0
			); 
	cv::Mat transformationMat = K * (T * (R * A1));

	cv::warpPerspective(source, destination, transformationMat, image_size, cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);

	cv::imshow("Result", destination);
	cv::waitKey(0);
    return 0;
}

int calibratePerspectiveTransformation(){
  cv::Mat image = cv::imread("test/test01.jpg" ,cv::IMREAD_COLOR);
  cv::Size size;
  size = image.size();
  std::cout << size <<std::endl;
  cv::Point2f objPts[4], imgPts[4];
  // Calibrweted with 800 x 582 image
  objPts[0].x = 696; objPts[0].y = 455; 
  objPts[1].x = 1096; objPts[1].y = 719;
  objPts[2].x = 206; objPts[2].y = 719; 
  objPts[3].x = 587; objPts[3].y = 455; 
  
  // Calibrweted with 800 x 582 image
  imgPts[0].x = 930; imgPts[0].y = 0; // Top-Right
  imgPts[1].x = 930; imgPts[1].y = 719; // Bottom-Right
  imgPts[2].x = 350; imgPts[2].y = 719; // Bottom-Left
  imgPts[3].x = 350; imgPts[3].y = 0; // Top-Left
    
  /*imgPts[0].x = 
  imgPts[1].y = 
  imgPts[1].x = 
  imgPts[1].y = 
  imgPts[2].x = 
  imgPts[2].y = 
  imgPts[3].y = 
  imgPts[3].y = */
  // DRAW THE POINTS in order: B,G,R,YELLOW
  int objPtsRadius = 12;
  int imgPtsRadius = 9;
  cv::circle(image, objPts[0], objPtsRadius, cv::Scalar(255, 0, 0), 3);
  cv::circle(image, objPts[1], objPtsRadius, cv::Scalar(255, 0, 0), 3);
  cv::circle(image, objPts[2], objPtsRadius, cv::Scalar(255, 0, 0), 3);
  cv::circle(image, objPts[3], objPtsRadius, cv::Scalar(255, 0, 0), 3);
  cv::circle(image, imgPts[0], imgPtsRadius, cv::Scalar(255, 255, 255), 3);
  cv::circle(image, imgPts[1], imgPtsRadius, cv::Scalar(255, 255, 255), 3);
  cv::circle(image, imgPts[2], imgPtsRadius, cv::Scalar(255, 255, 255), 3);
  cv::circle(image, imgPts[3], imgPtsRadius, cv::Scalar(255, 255, 255), 3);
  
  /*
  
      if not src:
            src = np.float32([[696, 455],
                             [1096, 719],
                             [206, 719],
                             [587, 455]])
        if not dst:
            dst = np.float32([[930, 0],
                             [930, 719],
                             [350, 719],
                             [350, 0]])

        #get transformation matrix
        M = cv2.getPerspectiveTransform(src, dst)
        #get inverse transformation matrix
        Minv = cv2.getPerspectiveTransform(dst, src)
  
  */
  
  
  cv::Mat H = cv::getPerspectiveTransform(objPts, imgPts);
  cv::Mat HInv = cv::getPerspectiveTransform(imgPts, objPts);
  std::cout << H << std::endl;
  
  
  cv::imshow("Test-Image", image);
  cv::waitKey(0);
  
  cout << "\nPress 'd' for lower birdseye view, and 'u' for higher (it adjusts the apparent 'Z' height), Esc to exit" << endl;
  double Z = 15;
  cv::Mat birds_image;
  //cv::Size birds_image_size(280, 230);
  for (;;) {
    // escape key stops
    H.at<double>(2, 2) = Z;
    // USE HOMOGRAPHY TO REMAP THE VIEW
    //
    cv::warpPerspective(image,			// Source image
                        birds_image, 	// Output image
                        H,              // Transformation matrix
                        image.size(),   // Size for output image
                        cv::INTER_LINEAR, // cv::WARP_INVERSE_MAP | cv::INTER_LINEAR,
                        cv::BORDER_CONSTANT, cv::Scalar::all(0) // Fill border with black
                        );
    cv::imshow("Birds_Eye", birds_image);
    int key = cv::waitKey() & 255;
    if (key == 'u')
      Z += 0.5;
    if (key == 'd')
      Z -= 0.5;
    if (key == 27)
      break;
  }
  
  return 0;
}

int testMovableImageData(){
  cv::Mat image = cv::imread("test/test01.jpg" ,cv::IMREAD_COLOR);
  cv::Mat result;
  cv::Size size;
  size = image.size();
  Debuglevel debuglevel = Debuglevel::verbose;
  MovableImageData movableImage(image, debuglevel);
  movableImage.getRawImageSize(size); // TESTED::OK
  movableImage.getRawImage(result); // TESTED::OK
  imshow("Original", image);
  imshow("Processed", result);
  cv::waitKey(0);  
  return 0;
}

int testImageServer(){
  Debuglevel debuglevel = Debuglevel::verbose;
  ImageServer imageServer(debuglevel);
  cv::Mat image = cv::imread("test/test01.jpg" ,cv::IMREAD_COLOR);
  cv::Mat result;
  CameraDriver camera(debuglevel);
  camera.calibrate();
  //imageServer.convert2GrayImage(image, result); // TESTED::OK
  //imageServer.applyGausianBlurr(image, result); // TESTED::OK
  imageServer.undistortImage(camera, image, result); // TESTED::OK
  imshow("Original", image);
  imshow("Processed", result);
  cv::waitKey(0);
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
  //flag = drawpoly();
  //flag = transform2BirdsEyeView();
  //flag = calibratePerspectiveTransformation();
  flag = testMovableImageData();
  //flag = testImageServer();
  return 0;
}