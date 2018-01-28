# README #



## Project Overview ##


Packages used for Low-level control of electronics used for a variety of robotic applications, as well as algorithms for data fusion and controls.


### Version Summary ###

Version 1.0.0

Language: C++

Dependencies:

- pigpiod
- Boost
- OpenCV

### Developer Notes ###


#### CMakelists to make ####

     - Actuators
          - DC Motor
          - Servo
     - Communications
          - UDP
          - Serial
     - Sensors
          - Encoder
          - IMU (MPU-9250)
          - CameraStreamer
     - Devices
          - RazorIMU
          - RoboClaw
     - Filters
          - EKF
     - Controls
          - PID
     - Profiles
          - 4WD

#### Build-able Drivers ####

     - Utilities

#### Drivers to Re-package ####

     - Actuators
          - DC Motor
          - Servo
     - Communications
          - UDP
          - Serial
     - Sensors
          - Encoder
          - IMU (MPU-9250)
          - CameraStreamer
     - Devices
          - RazorIMU
          - RoboClaw
     - Filters
          - EKF
     - Controls
          - PID
     - Profiles
          - 4WD

#### Drivers to Test ####


#### Drivers to create ####

     - Devices
          - MappyDot (LiDAR)
          - PWM Driver Board (PCA-9685)

     - Controls
          - LiDAR-based Row following
          - GPS waypoint-follower (pure pursuit)

     - Filters
          - RANSAC Line Smoother


### Developer Instructions ###

Currently the drivers are built specifically to target the Raspberry Pi 3

#### Install Dependencies ####

To Be Continued...


##### Building dependencies from source #####

#### Configure and Build ####

To Be Continued...

##### Cross-compiling on Ubuntu #####

#### Installation ####

To Be Continued...

#### Running ####

To Be Continued...

#### Testing ####

There are currently no testing procedures or programs defined or developed for this project.

#### Developer Notes ####

To Be Continued...

#### TODOs (Double Check if still valid) ####

     -

     - combine SerialDev class with the functionality of the pigpiod serial functions used in RoboClaw class
     - convert DC_motor class to modular class setup using class inheritance (model code structure similar to Motors class)




### Maintainer ###

Hunter Young (hunter.lw.young@gmail.com)
