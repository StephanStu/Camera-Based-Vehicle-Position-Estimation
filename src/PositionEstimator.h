#ifndef POSITIONESTIMATOR_H
#define POSITIONESTIMATOR_H

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include "ImageTransformer.h"
#include "RecordServer.h"
#include "Types.h"

#define PI 3.14159265

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
    bool findLeftLaneLineInHoughLines(const std::vector<cv::Vec4i>& houghLines, cv::Vec4i& laneLine, float& deviation, float& angle); // returns true, if a valid (!) left lane line has been found in a set of lines (computed with the Hough-Transformation) and saves the resulting lane line in laneLine, the deviation from the center in deviation and the angle between the vehicle and the center lane in angle
    bool findRightLaneLineInHoughLines(const std::vector<cv::Vec4i>& houghLines, cv::Vec4i& laneLine, float& deviation, float& angle); // returns true, if a valid (!) right lane line has been found in a set of lines (computed with the Hough-Transformation) and saves the resulting lane line in laneLine, the deviation from the center in deviation and the angle between the vehicle and the center lane in angle
    void initializeKalmanFilter(); // initializes the Kalmanfilter for 2D-Motion Prediction
    void predict(const float timestep); // predicts the state and the state covariance using the equations of motion
    void update(const Measurement& measurement); // Updates the state by using standard Kalman Filter equations
    void computeStateTransitionMatrix(const float timestep); // compute the time-dependent state transition matrix A (member attribute) 
    void computeProcessCovariancenMatrix(const float timestep); // compute the time-dependent process covariance matrix Q (member attribute)
    Eigen::VectorXd mapState2Outputs(const Eigen::VectorXd& state); // compute the return of the output-equation "h(x)" the vector [phi deviation]^T
    Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& state); //  compute the Jacobian of the output euqation H = dh(x)/dx
    void getStateVector(Eigen::VectorXd& state); // write the state vector to the referenced variable
  private:
    friend class PositionServer;
    void manageStateSwitches(); // this method manages switchs in States =  {initializing, running, freezed, terminated} and calls the appropriate methdos
    void runInRunningState(); // this method is called when State = runningand runs a kalman-filter to estimate the vehicle's distance to center line based on transformed images pulled from :ImageTransformer.
    void runInInitializingState(); // this method is called when State = initializing
    void runInFreezedState(); // this method is called when State = freezed
    void runInTerminatedState(); // this method is called when State = terminated
    PositionServiceRecord updatePosition(MovableTimestampedType<PositionServiceRecord>& movableTimestampedRecord); // this method updates the estimate of the state (runs the Kalman-Filter and catches exceptions if necessary)
    std::shared_ptr<ImageTransformer> accessImageTransformer; // shared pointer to an instance of CameraDriver delivering images upond request
    bool newRecordIsAvailable(); // verifies, that the server has a new record (before requesting to send it)
    bool isVertical( cv::Vec4i line); // returns true if the line is vertical (in the image coordinate system)
    bool imageTransformerIsMounted; // this is true once an intance of ImageTransformer has been mounted
    bool kalmanFilterIsInitialized; // this is true once the Kalman Filter has been initialized
    int threshold = 100; // parameter in Hough-Transformation, the minimum number of intersections to "*detect*" a line
    int minLineLength = 25; // parameter in Hough-Transformation, the minimum number of points that can form a line. Lines with less than this number of points are disregarded
    int maxLineGap = 50; // parameter in Hough-Transformation, the maximum gap between two points to be considered in the same line
    const float laneWidth = 3.6576; // width of lane in metres
    const float metresPerPixel = 0.02147443; // in x-direction of the image coordinates: How many metres per pixel?
    const float lowerAngleThreshold = - 15.0 * PI / 180; // lower bound for a reasonable road vehicle angle crusing on highway
    const float upperAngleThreshold = 15.0 * PI / 180; // lower bound for a reasonable road vehicle angle crusing on highway
    const float deviationThreshold = 0.9144; // threshold for deviation from center line, quarter of road width
    const int centerLineXCoordinate = 250; // center line coordinate of the bird eye's image
    const int birdEyesViewHeight = 600; // bottom of the bird eye's view image
    const int birdEyesViewWidth = 500; // width of the bird eye's view image
    Eigen::VectorXd x; // state vector
    Eigen::VectorXd z; // measured output vector
    Eigen::VectorXd y; // predicted output vector
    Eigen::MatrixXd P; // state covariance matrix
    Eigen::MatrixXd A; // state transition matrix
    Eigen::MatrixXd Q; // process covariance matrix
    const float deviationVariance = 0.02; // measurement noise variance in deviation in metres
    const float angleVariance = 0.034; // measurement noise variance in deviation in rad = deg * PI / 180
    const float velocityVariance = 2.78; // measurement noise variance in velocity in m/s
    const float accelerationYNoise = 0.25; // variance of acceleration assuming it is a stochastic process with zero mean 
    const float accelerationXNoise = 2.0; // variance of acceleration assuming it is a stochastic process with zero mean 
    const float initialVelocity = 88 * 1000/3600; // initial assumption on vehicle velocity = 55 miles per hour
};

#endif