#!/bin/bash

libs_root="$HOME/libraries"
mkdir -p $libs_root

############################################
#  Get RTIMULib library
############################################
read -n1 -p "Do you need to get the RTIMULib repo? Enter (y) or (n)" doit
echo
if [[ $doit == "Y" || $doit == "y" ]]; then
     cd $libs_root
     git clone https://github.com/RPi-Distro/RTIMULib.git
fi
