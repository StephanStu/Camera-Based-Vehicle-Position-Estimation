# Vision-Based-Runway-Position-Estimation
This is my capstone project in Udacity's C++ Nanodegree Program. As I work in the urban air mobility (UAM) industry which targets to launch production &amp operation of electrical vertical take-off &amp landing (eVTOL) aircrafts to offer new ways of moving in congested urban areas I am keen to accelerate this venture with easier aircaft handling and cheap equipment that is utilized well with contemporary software technology. This project contaions an application that estimates the position of an aircaft as it taxis on a runway. The appliaction generates an estimate of the deivation of the vehicle from the center lane that may be fed to the avionics in order to offer a better bearing to the pilot during taxi or fed to an automatic taxi- &amp take-off algorithm that steers the vehicle along the runway to it's final take-off-position.

## Requirements

## Architecture &amp Mapping of Requirements to Software

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

## Basic Build Instructions
This section explains how to build &amp deploy the application.  

1. Clone this repo.
2. Change the directory to the build directory: `cd build`
3. Compile: `cmake .. && make`
4. Run it: `./VBRPE`.

## Literature cited
[1] B.Anbarasu  &amp G.Anitha, Vision-based runway recognition and position estimation for UAV
autonomous landing, find it [here](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=2792686)
