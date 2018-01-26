# README #



### Project Overview ###


Packages used for Low-level control of electronics used for a variety of robotic applications, as well as algorithms for data fusion and controls.


#### Version Summary ####

Version 1.0.0

Language: C++

Dependencies:

1. pigpiod
2. OpenCV

#### Working Drivers ####

     1. PID
     2. Serial
     3. Utilities
     4. Encoder
     5. RazorIMU
     6. 4WD
     7. CameraStreamer
     8. UDP
     9. Motor
     10. Servo

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

     1. i2c base class for Rpi 3 system
     2. combine SerialDev class with the functionality of the pigpiod serial functions used in RoboClaw class
     3. convert DC_motor class to modular class setup using class inheritance (model code structure similar to Motors class)
     4. Combine all base class code into one area in include folder
     5. Organize filesystem for more modular project


### Maintainer ###

Hunter Young (hunter.lw.young@gmail.com)
