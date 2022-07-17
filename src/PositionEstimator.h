#ifndef POSITIONESTIMATOR_H
#define POSITIONESTIMATOR_H

#include <opencv2/opencv.hpp>
#include "ImageTransformer.h"
#include "RecordServer.h"
#include "Types.h"

/*
PositionEstimator:
*/

class PositionEstimator : public RecordServer {
  public:
    // constructor / desctructor
    PositionEstimator();
    PositionEstimator(Debuglevel positionEstimatorDebuglevel);
    // other
    void run(); // overrides virtual function "run" in base class
    void mountImageTransformer(std::shared_ptr<ImageTransformer> pointerToImageTransformer); // "mount" an instance of ImageTransformer via pasing a shared point, s.t. methods can access the ImageTransformer
    void getHoughLines(const cv::Mat& image, std::vector<cv::Vec4i>& lines); // compute the lines in the image using opneCV's Hough-Transformation
    void createHoughLinesImage(const cv::Mat& source, cv::Mat destination); // draw a black image of size source with Houg Lines on it and return it at referenced destination
  private:
    friend class PositionServer;
    void manageStateSwitches(); // this method manages switchs in States =  {initializing, running, freezed, terminated} and calls the appropriate methdos
    void runInRunningState(); // this method is called when State = runningand runs a kalman-filter to estimate the vehicle's distance to center line based on transformed images pulled from :ImageTransformer.
    void runInInitializingState(); // this method is called when State = initializing
    void runInFreezedState(); // this method is called when State = freezed
    void runInTerminatedState(); // this method is called when State = terminated
    void updatePosition(const PositionServiceRecord& newRecord); // this method updates the estimate of the position (member variable)
    std::shared_ptr<ImageTransformer> accessImageTransformer; // shared pointer to an instance of CameraDriver delivering images upond request
    bool imageTransformerIsMounted; // this is true once an intance of ImageTransformer has been mounted
    int threshold = 100; // parameter in Hough-Transformation, the minimum number of intersections to "*detect*" a line
    int minLineLength = 25; // parameter in Hough-Transformation, the minimum number of points that can form a line. Lines with less than this number of points are disregarded
    int maxLineGap = 25; // parameter in Hough-Transformation, the maximum gap between two points to be considered in the same line
};

#endif