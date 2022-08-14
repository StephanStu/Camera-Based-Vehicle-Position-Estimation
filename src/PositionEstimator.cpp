#include <eigen3/Eigen/Dense>
#include "PositionEstimator.h"

#define PI 3.14159265

PositionEstimator::PositionEstimator() : imageTransformerIsMounted(false) {
  debugLevel = Debuglevel::none;
}

PositionEstimator::PositionEstimator(Debuglevel positionEstimatorDebuglevel) : imageTransformerIsMounted(false) {
  debugLevel = positionEstimatorDebuglevel;
  printToConsole("PositionEstimator::PositionServer called.");
}

void PositionEstimator::run(){
  printToConsole("PositionEstimator::run called.");
  threads.emplace_back(std::thread(&PositionEstimator::manageStateSwitches, this));
}

void PositionEstimator::mountImageTransformer(std::shared_ptr<ImageTransformer> pointerToImageTransformer){
  printToConsole("PositionEstimator::mountImageTransformer called, instance of ImageTransformer is mounted by keeping the shared pointer.");
  accessImageTransformer = pointerToImageTransformer;
  imageTransformerIsMounted = true;
}

void PositionEstimator::manageStateSwitches(){
  while(true){
    if(currentState == initializing){
      printToConsole("PositionEstimator::manageStateSwitches is called, instance is waiting in state initializing.");
      runInInitializingState();
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
    if(currentState == running){
      if(ready){
        std::string message = "PositionEstimator::manageStateSwitches is called: Instance is in state running with isCurrent = " + std::to_string(isCurrent) + ".";
        printToConsole(message);
        runInRunningState();
      }else{
        printToConsole("PositionEstimator::manageStateSwitches is called: Instance is not ready & calling the initializing routine.");
        runInInitializingState();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
    if(currentState == terminated){
      printToConsole("PositionEstimator::manageStateSwitches is called, instance has reached state terminated.");
      runInTerminatedState();
      break;
    }
    if(currentState == freezed){
      printToConsole("PositionEstimator::manageStateSwitches is called, instance is waiting in state freezed.");
      runInFreezedState();
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepForMilliseconds));
    }
  }
}

void PositionEstimator::runInRunningState(){
  /* get the record from :ImageTransformer */
  if(newRecordIsAvailable()){
    PositionServiceRecord newRecord;
    std::promise<MovableTimestampedType<PositionServiceRecord>> prms;
    std::future<MovableTimestampedType<PositionServiceRecord>> ftr = prms.get_future();
    std::thread t(&ImageTransformer::sendRecord, accessImageTransformer, std::move(prms));
    t.join();
    MovableTimestampedType<PositionServiceRecord> movableTimestampedRecord = std::move(ftr.get());
    /* apply Hough-Transform and update the estimate of the position */
    newRecord = updatePosition(movableTimestampedRecord);
    /* use movableTimestampedRecord.setData(newRecord) to update the position here */
    movableTimestampedRecord.setData(newRecord);
    /* save the updated record in the member variable by "moving" it (in order to keep the timestamp!) */
    std::unique_lock<std::mutex> uniqueLock(protection);
    record = std::move(movableTimestampedRecord);
    isCurrent = true;
    uniqueLock.unlock();
  }else{
    printToConsole("PositionEstimator::runInRunningState called: Wainting for :ImageTransformer to update it's record.");
  }
}

void PositionEstimator::runInInitializingState(){
  isCurrent = false;
  ready = imageTransformerIsMounted && kalmanFilterIsInitialized;
  if(!ready){
    printToConsole("PositionEstimator::runInInitializingState is called; instance is not ready to enter the running state: Mount an :ImageTransformer.");
  }
  if(!kalmanFilterIsInitialized){
    initializeKalmanFilter();
  }
}

void PositionEstimator::runInFreezedState(){
  isCurrent = false;
}

void PositionEstimator::runInTerminatedState(){
  isCurrent = false;
}


PositionServiceRecord PositionEstimator::updatePosition(MovableTimestampedType<PositionServiceRecord>& movableTimestampedRecord){
  PositionServiceRecord record;
  float rightDeviation;
  float leftDeviation;
  float leftAngle;
  float rightAngle;
  cv::Vec4i leftLaneLine;
  bool leftLaneDetected;
  cv::Vec4i rightLaneLine;
  bool rightLaneDetected;
  bool someLaneDetected = false;
  std::vector<cv::Vec4i> lines;
  long int age = movableTimestampedRecord.getAge();
  std::string message = "PositionEstimator::updatePosition: Using a record with an age of " + std::to_string(age) + " milliseconds to compute angle & deviation.";
  printToConsole(message);
  cv::Mat image = movableTimestampedRecord.getData().binaryBirdEyesViewImage;
  getHoughLines(image, lines);
  /* Print the lines found to console if suitable debugLevel is chosen */
  if(lines.size()>0){
    std::string message = "PositionEstimator::updatePosition: Found " + std::to_string(lines.size()) + " lines in the current image.";
    printToConsole(message);
    for (size_t i=0; i<lines.size(); i++) {
      cv::Vec4i l = lines[i];
      std::string message = "PositionEstimator::updatePosition: Line " + std::to_string(i+1) + " is [ " + std::to_string(l[0]) + " " +  std::to_string(l[1]) + " " + std::to_string(l[2]) + " " + std::to_string(l[3]) + " ].";
      printToConsole(message);
    }
  }
  /* Use age and lines to update the kalman filter and return the current position */
  rightLaneDetected = findRightLaneLineInHoughLines(lines, rightLaneLine, leftDeviation, leftAngle);
  leftLaneDetected = findLeftLaneLineInHoughLines(lines, leftLaneLine, rightDeviation, rightAngle);
  Measurement measurement;
  if(leftLaneDetected){
    measurement.deviation = leftDeviation;
    measurement.angle = leftAngle;
    measurement.velocity = movableTimestampedRecord.getData().velocity; // To Do: Ship velocity here
    someLaneDetected = true;
  }else{
    if(rightLaneDetected){
      measurement.deviation = rightDeviation;
      measurement.angle = rightAngle;
      measurement.velocity = movableTimestampedRecord.getData().velocity; // To Do: Ship velocity here
      someLaneDetected = true;
    }
  }
  if(someLaneDetected){
    predict((age/1000.0));
    update(measurement);
  }else{
    predict((age/1000.0));
  }
  Eigen::VectorXd x(4);
  getStateVector(x);
  float py = x(1);
  float vx = x(2);
  float vy = x(3);
  float phi;
  if(vx > 0){
    printToConsole(message);
    phi = atan(vy/vx);
  }else{
    phi = 0.0;
    printToConsole("PositionEstimator::updatePosition: is called with vx < 0; using replacement value for angle now.");
  }
  record.rawImage = movableTimestampedRecord.getData().rawImage;
  record.undistortedImage = movableTimestampedRecord.getData().undistortedImage;
  record.birdEyesViewImage = movableTimestampedRecord.getData().birdEyesViewImage;
  record.binaryBirdEyesViewImage = movableTimestampedRecord.getData().binaryBirdEyesViewImage;
  if(leftLaneDetected){
    record.distanceToLeftLane = leftDeviation;
  }else{
    record.distanceToLeftLane = -99;
  }
  if(rightLaneDetected){
    record.distanceToRightLane = rightDeviation;
  }else{
    record.distanceToRightLane = -99;
  }
  record.deviation = py;
  record.angle = phi;
  record.velocity = sqrt(vx*vx + vy*vy);
  return record;
}

void PositionEstimator::getHoughLines(const cv::Mat& image, std::vector<cv::Vec4i>& lines){
  printToConsole("PositionEstimator::getHoughLines is called.");
  cv::HoughLinesP(image, lines, 1, CV_PI/180, threshold, minLineLength, maxLineGap);
}

void PositionEstimator::createHoughLinesImage(const cv::Mat& source, cv::Mat destination){
  printToConsole("PositionEstimator::createHoughLinesImage is called.");
  std::vector<cv::Vec4i> lines;
  getHoughLines(source, lines);
  destination = cv::Mat::zeros(source.size(), CV_8UC3);
  /* draw lines on the image (if any are found) */
  if(lines.size()>0){
    for (size_t i=0; i<lines.size(); i++) {
      cv::Vec4i l = lines[i];
      cv::line(destination, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
    }
  }
}

bool PositionEstimator::newRecordIsAvailable(){
  return accessImageTransformer->isCurrent;
}


bool PositionEstimator::isVertical( cv::Vec4i line){
  if(line[0] == line[2]){
    return true;
  }else{
    return false;
  }
}

bool PositionEstimator::findRightLaneLineInHoughLines(const std::vector<cv::Vec4i>& houghLines, cv::Vec4i& laneLine, float& deviation, float& angle){
  int yMax = birdEyesViewHeight; // bottom of the image
  int xMax = birdEyesViewWidth; // width of the image
  int closestToCenterXCoordinate = xMax;
  float slope; // slope
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
      }
    }
  }
  if(success){
    std::string message = "PositionEstimator::findRightLaneLineInHoughLines: Detected right lane line, derived angle = " + std::to_string(angle * 180.0/PI) + " deg and distance = " + std::to_string(deviation * 100) + " cm"; 
    printToConsole(message);
  }
  return success;
}

bool PositionEstimator::findLeftLaneLineInHoughLines(const std::vector<cv::Vec4i>& houghLines, cv::Vec4i& laneLine, float& deviation, float& angle){
  int closestToCenterXCoordinate = 0;
  int yMax = birdEyesViewHeight; // bottom of the image
  int xMax = birdEyesViewWidth; // width of the image
  float slope;
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
      }
    }
  }
  if(success){
    std::string message = "PositionEstimator::findLefttLaneLineInHoughLines: Detected left lane line, derived angle = " + std::to_string(angle * 180.0/PI) + " deg and distance = " + std::to_string(deviation * 100) + " cm"; 
    printToConsole(message);
  }
  return success;
}

void PositionEstimator::initializeKalmanFilter(){
  x = Eigen::VectorXd(4);
  x << 0.0, 0.0, initialVelocity, 0.0;
  computeStateTransitionMatrix(0.0);
  computeProcessCovariancenMatrix(0.0);
  P = Eigen::MatrixXd(4, 4);
  P <<  0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0;
  kalmanFilterIsInitialized = true;  
}

void PositionEstimator::predict(const float timestep){
  std::string message = "PositionEstimator::predict is called with timestep = " + std::to_string(timestep*1000) + " ms"; 
  printToConsole(message);
  computeStateTransitionMatrix(timestep);
  computeProcessCovariancenMatrix(timestep);
  x = A * x;
  Eigen::MatrixXd At = A.transpose();
  P = A * P * At + Q;
}

void PositionEstimator::update(const Measurement& measurement){
  std::string message = "PositionEstimator::update is called with measured deviation = " + std::to_string(measurement.deviation * 100) + " cm, angle = " + std::to_string(measurement.angle * 180/PI)  + " deg and velocity = " + std::to_string(measurement.velocity * 3600/1000) + " km/h"; 
  printToConsole(message);
  z = Eigen::VectorXd(3);
  z << measurement.deviation, measurement.angle, measurement.velocity;
  Eigen::MatrixXd H(3, 4); // TO DO
  H = computeJacobian(x);
  Eigen::MatrixXd R(3, 3);
  R << deviationVariance, 0,  0, // variance in deviation (m)
       0, angleVariance, 0, // variance in angle (rad)
       0, 0, velocityVariance; // variance in velocity (m/s)
  Eigen::VectorXd y = z - mapState2Outputs(x);
  // ensure the angle to be in between -pi to pi
  while(y(1) > PI){
      y(1) -= 2 * PI;
    }
  while(y(1) < -PI){
    y(1) += 2 * PI;
  }
  Eigen::MatrixXd Ht = H.transpose();
  Eigen::MatrixXd S = H * P * Ht + R;
  Eigen::MatrixXd Si = S.inverse();
  Eigen::MatrixXd K = P * Ht * Si;
  x = x + (K * y);
  int dimension_of_x = x.size();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dimension_of_x, dimension_of_x);
  P = (I - K * H) * P;
}

void PositionEstimator::computeStateTransitionMatrix(const float timestep){
  printToConsole("PositionEstimator::computeStateTransitionMatrix is called.");
  A = Eigen::MatrixXd(4, 4);
  A << 1.0, 0.0, timestep, 0.0,
             0.0, 1.0, 0.0, timestep,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0;
}

void PositionEstimator::computeProcessCovariancenMatrix(const float timestep){
  printToConsole("PositionEstimator::computeProcessCovariancenMatrix is called.");
  float dt = timestep;
  float dt_2 = dt   * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  Q = Eigen::MatrixXd(4, 4);
  Q << dt_4 / 4.0 * accelerationXNoise, 0.0, dt_3 / 2.0 * accelerationXNoise, 0.0,
             0.0, dt_4 / 4.0 * accelerationYNoise, 0.0, dt_3 / 2.0 * accelerationYNoise,
             dt_3 / 2.0 * accelerationXNoise, 0.0, dt_2 * accelerationXNoise, 0.0,
             0.0, dt_3 / 2.0 * accelerationYNoise, 0.0, dt_2 * accelerationYNoise;
}

Eigen::VectorXd PositionEstimator::mapState2Outputs(const Eigen::VectorXd& state){
  Eigen::VectorXd z(3);
  float py = state(1);
  float vx = state(2);
  float vy = state(3);
  float phi;
  if(vx > 0){
    std::string message = "PositionEstimator::mapState2Outputs is called with vx = " + std::to_string(vx * 3600/1000) + " km/h"; 
    printToConsole(message);
    phi = atan(vy/vx);
  }else{
    phi = 0.0;
    printToConsole("PositionEstimator::mapState2Outputs is called with vx < 0; using replacement value for angle now.");
  }
  float v = sqrt(vx*vx + vy*vy);
  z << py, phi, v;
  return z;
}

Eigen::MatrixXd PositionEstimator::computeJacobian(const Eigen::VectorXd& state){
  printToConsole("PositionEstimator::computeJacobian is called.");
  float vx = state(2);
  float vy = state(3);
  float temp = (vx*vx) + (vy*vy);
  Eigen::MatrixXd H(3, 4);
  H << 0, 1, 0, 0, // [dh1(x)/dpx dh1(x)/dpy dh1(x)/dvx dh1(x)/dvy] with h1 = py = distance to center lane (normal to roadcenter line)
       0, 0, -vy/temp, vx/temp, // [dh2(x)/dpx dh2(x)/dpy dh2(x)/dvx dh2(x)/dvy] with h2 = phi = angle between direction of traveling & center lane ("+" = twist to the right)
       0, 0, vx/sqrt(temp), vy/sqrt(temp); // [dh3(x)/dpx dh3(x)/dpy dh3(x)/dvx dh3(x)/dvy] with h3 = velocity = length of the velocity vector
  return H; 
}

 void PositionEstimator::getStateVector(Eigen::VectorXd& state){
   float px = x(0);
   float py = x(1);
   float vx = x(2);
   float vy = x(3);
   std::string message = "PositionEstimator::getStateVector is called with state =  [ " + std::to_string(px * 100)  + 
     " cm " +
     std::to_string(py * 100)  + 
     " cm " +
     std::to_string(vx * 3600/1000)  + 
     " km/h " +
     std::to_string(vy * 3600/1000)  +  " km/h ]"; 
   printToConsole(message);
   state << px, py, vx, vy;
 }
