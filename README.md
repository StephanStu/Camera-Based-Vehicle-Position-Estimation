# Camera-based Vehicle-Position-Estimation
This is my capstone project in Udacity's C++ Nanodegree Program. This project contains an application that estimates the position of a road vehicle as it travels on a highway. The application generates an estimate of the state of the vehicle that may be fed to a driver assistant system or may be part of an autonomous driving system: An extended Kalman Filter estimates the position on the road and the velocities using "measurements" taken from a simple camera image. The architecture of this application is service-oriented, suitable for (Ubuntu) Linux and uses OpenCV and the Eigen-Library.

The application is shipped with sample images and sample videos for

* static testing of the Kalman Filter
* calibration of the camera
* videos to test the performance of the Kalman Filter with real-world data

in a save (cloud or desktop) Ubuntu-Linux Environment.

## Requirements
The following requirements hold:
* 00: The position of the vehicle on the road must be updated when a new image becomes available at the source of images of the road in front of the vehicle.
* 01: An image of the road in front of the vehicle must be read from an image source and at the same time the vehicle velocity must be captured. These two entities must be saved in a structure at the same time.
* 02: An image of the road in front of the vehicle must be undistorted before it is used in downstream image processing.
* 03: The undistorted image must be used to detect lanes in the image.
* 04: If lanes can be detected in the image, the distance to the center line of the road and the angle of the vehicle w.r.t. the center line of the road must be computed.
* 05: Distances and angles computed from images of the road in front of the vehicle must not be used directly by clients of this service; they must but fed into a filter minimizing the effects of noise, processing time and uncertainty in the accuracy of the image processing algorithm. Clients of this service consume the estimated state only.
* 06: The trip must be recorded in a file; the file must contain the estimated state, the timestamp and the distances & angles computed with image processing to arrive at the state estimate.

## Architecture & Mapping of Requirements to Software

## Dependencies for Running Locally
This section lists the dependencies of the application &amp how to get them.

* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* Eigen-Library
  * Linux: Run chmod u+x installEigenLibrary.sh && ./installEigenLibrary.sh 

## Basic Build Instructions
This section explains how to build &amp deploy the application.  

1. Clone this repo.
2. Change the directory to the build directory: `cd build`
3. Compile: `cmake .. && make`
4. Run it: `./VBRPE`.

## Running the Application
The application can be run by calling it with a relative path to a suitable video or a relative path to a suitable image.
Suitable images and videos are part of this repository:

* Find suitable images in /build/test, e.g. /test/test01.jpg
* Find suitable videos in /build/, e.g. video01.mp4

The application runs in a multi-threaded mode when parsing a suitable video, otherwise it runs in a single-threaded mode. Parsing a suitable image is usefull to test convergence of the filter with "perfect" road images.

### Running the Application with a Suitable Image
To run with a suitable image, switch to /build/ and

TO DO: ADD COMMANDS HERE

A trip is simulated assuming the same image appears again and again in the camera of the vehicle. 
Find the resulting record of this trip in the same directory as name-of-the-file-passed.csv
Note: Roughly constant vehicle velocity is assumed, as defined in main.cpp:

ADD DEFINED HERE

### Running the Application with a Suitable Video
To run with a suitable image, switch to /build/ and

TO DO: ADD COMMANDS HERE

A trip is simulated drawing video frames at a frequency simulating the camera of the vehicle. 
Find the resulting record of this trip in the same directory as name-of-the-file-passed.csv
Note: Roughly constant vehicle velocity is assumed, as defined in main.cpp:

ADD DEFINED HERE

## Literature cited
[1] B.Anbarasu  &amp G.Anitha, Vision-based runway recognition and position estimation for UAV
autonomous landing, find it [here](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=2792686)
