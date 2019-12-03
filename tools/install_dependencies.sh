#!/bin/bash

libs_root="$HOME/libraries"
echo "Libraries directory set to $libs_root"
mkdir -p $libs_root

############################################
#  Get RTIMULib library
############################################
read -n1 -p "Do you need to get the RTIMULib repo? Enter (y) or (n)" doit
echo
if [[ $doit == "Y" || $doit == "y" ]]; then
     cd $libs_root
     git clone https://github.com/RPi-Distro/RTIMULib.git

     sudo apt update
     sudo apt install cmake
     sudo apt install libqt4-dev

     cd RTIMULib/Linux
     mkdir build
     cd build

     cmake ..
     make -j4
     sudo make install
     sudo ldconfig
fi

read -n1 -p "Do you need to install the pigpiod library? Enter (y) or (n)" doit
echo
if [[ $doit == "Y" || $doit == "y" ]]; then
     cd $libs_root
     wget abyz.me.uk/rpi/pigpio/pigpio.zip
     unzip pigpio.zip
     cd PIGPIO
     make -j4
     sudo make install

     rm $libs_root/pigpio.zip
fi

read -n1 -p "Do you need to install other dependencies? Enter (y) or (n)" doit
echo
if [[ $doit == "Y" || $doit == "y" ]]; then
     sudo apt update
     sudo apt install libboost-all-dev
     sudo apt install libarmadillo-dev
fi

read -n1 -p "Do you need to install the GTSAM library? Enter (y) or (n)" doit
echo
if [[ $doit == "Y" || $doit == "y" ]]; then
     cd $libs_root
     git clone https://github.com/borglab/gtsam.git
     cd gtsam
     mkdir build
     cd build
     cmake ..
     make -j4
     make check
     sudo make install
fi

read -n1 -p "Do you need to install the librealsense2 library? Enter (y) or (n)" doit
echo
if [[ $doit == "Y" || $doit == "y" ]]; then
     sudo apt update
     sudo apt install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
     sudo apt install libglfw3-dev
     sudo apt install python python-dev python3 python3-dev

     cd $libs_root
     git clone https://github.com/IntelRealSense/librealsense.git

     cd librealsense
     sudo ./scripts/setup_udev_rules.sh
     mkdir build
     cd build
     cmake -DBUILD_PYTHON_BINDING=bool:true -DPYTHON_EXECUTABLE=/usr/bin/python2 -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_OPENMP=bool:true ..
     make -j4
     sudo make install
fi

pwd
