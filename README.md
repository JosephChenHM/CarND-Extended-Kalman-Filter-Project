# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

![](https://i.imgur.com/HNm3OLl.png)


---
## EKF System Design
![](https://i.imgur.com/4qxCeUL.png)

* **Initialize EFK matrices** is implemented in line `20 ~ 53` in `FusionEKF.cpp`
* **Initialize state x, and covariance matrices** is implemented in line `68 ~ 108` in `FusionEKF.cpp`
* **Predict block** is implemented in line `121 ~ 146` in `FusionEKF.cpp`
* **Predict function** is implemented in line `29 ~ 38` in `kalman_filter.cpp`
* **Update block** is implemented in line `148 ~ 168` in `FusionEKF.cpp`
* **Update laser function** is implemented in line `40 ~ 54` in `kalman_filter.cpp`
* **Update radar function** is implemented in line `56 ~ 94` in `kalman_filter.cpp`
* All the **Process Measurement** will be evalutae by **RMSE**(Root-Mean-Square-Error). It is implemented in line `12 ~ 57` in `tools.cpp`
---
```
CarND-Extended-Kalman-Filter-Project
│   README.md
│   install-ubuntu.sh
│   install-mac.sh
│   CMakeLists.txt
│   cmakepatch.txt
│    ...
└───Docs
│    ...
└───build
│    ...
└───data
│    ...
└───ide_profiles
│    ...
└───src
│   │   main.cpp
│   │   measurement_package.h
│   │   FusionEKF.cpp // Implement ProcessMeasurement
│   │   FusionEKF.h
│   │   kalman_filter.cpp // Define predict & updateKF % updateEKF function
│   │   kalman_filter.h
│   │   tools.cpp    // Calculate RMSE & Jacobian Matrix
│   │   tools.h
│   │
│   └───Eigen    // matrix library
│       │   ...
```

Here is the main protcol that main.cpp uses for `uWebSocketIO` in communicating with the simulator.

* INPUT: values provided by the simulator to the c++ program

> ["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


* OUTPUT: values provided by the c++ program to the simulator

> ["estimate_x"] <= kalman filter estimated position x
 ["estimate_y"] <= kalman filter estimated position y
["rmse_x"] <= Root Mean Square Error between ground truth and estimation of position x
["rmse_y"] <= Root Mean Square Error between ground truth and estimation of position y
["rmse_vx"] <= Root Mean Square Error between ground truth and estimation of velocity x
["rmse_vy"] <= Root Mean Square Error between ground truth and estimation of velocity y
---
## Build Dependencies
This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions
Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `
5. Excute [Udacity Term2 simulator](https://github.com/udacity/self-driving-car-sim/releases): `term2_sim`
6. Select: `Project 1/2: EKF and UKF`

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the [utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.