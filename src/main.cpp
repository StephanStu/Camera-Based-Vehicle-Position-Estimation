#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include <vector>
#include <iostream>
 
int runHoughTransformationTest(cv::Mat image){
  cv::Mat grayImage; // used for the result of transfroming the RGB-Image to a grayscale-image
  cv::Mat edgesDetected; // used for the result of the Canny-Edge-Detection
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
    Run the canny-edge-detection 
    */
    cv::Canny(grayImage, edgesDetected, 50, 200);
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

int main(){
  int flag;
  cv::Mat image; // used to hold an RGB-image
  image = cv::imread("SimpleRunwayTestImage.png" ,cv::IMREAD_COLOR); // sample image to test the transformation.
  cv::VideoCapture cap("testVideo002.mp4"); // sample video to test the video-processing
  //flag = runHoughTransformationTest(image);  cv::waitKey(0);
  flag = runVideoTest(cap);
  return 0;
}