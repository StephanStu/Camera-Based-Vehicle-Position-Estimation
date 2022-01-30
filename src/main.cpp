#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include <vector>
#include <iostream>
 
int main( int argc, char** argv ) {
  cv::Mat image; // used to hold an RGB-image
  cv::Mat grayImage; // used for the result of transfroming the RGB-Image to a grayscale-image
  cv::Mat edgesDetected; // used for the result of the Canny-Edge-Detection
  std::vector<cv::Vec4i> lines; // a vector to store lines of the image, using the cv::Vec4i-Datatype


  
  image = cv::imread("SimpleRunwayTestImage.png" ,cv::IMREAD_COLOR);
  
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
  
  cv::waitKey(0);
  return 0;
}