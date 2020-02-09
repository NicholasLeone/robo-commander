cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# ===========================
#     Set Build Options
# ===========================
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-narrowing")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-narrowing")
# =========================================
#          Build Common Libraries
# =========================================
add_library(utils SHARED utilities/utils.cpp)
target_link_libraries(utils PUBLIC ${OpenCV_LIBS} ${EXTERNAL_PACKAGE_LIBRARIES})
set(RoboCommander_LIBRARIES utils)

add_library(cv_utils SHARED utilities/cv_utils.cpp)
target_link_libraries(cv_utils PUBLIC utils ${OpenCV_LIBS})
list(APPEND RoboCommander_LIBRARIES cv_utils)

add_library(plot_utils SHARED utilities/plot_utils.cpp)
target_link_libraries(plot_utils PUBLIC ${OpenCV_LIBS})
list(APPEND RoboCommander_LIBRARIES plot_utils)

add_library(img_utils SHARED utilities/image_utils.cpp)
target_link_libraries(img_utils PUBLIC ${OpenCV_LIBS})
list(APPEND RoboCommander_LIBRARIES img_utils)

# =========================================
#      Build Communication Libraries
# =========================================
add_library(serial SHARED communication/serial.cpp)
target_link_libraries(serial PUBLIC ${Boost_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES serial)

# i2c
add_library(i2c SHARED communication/i2c.cpp)
target_link_libraries(i2c PUBLIC ${pigpiod_if2_LIBRARY})
list(APPEND RoboCommander_LIBRARIES i2c)

# UDP
add_library(udp SHARED communication/udp.cpp)
list(APPEND RoboCommander_LIBRARIES udp)

# UART
add_library(uart SHARED communication/uart.cpp)
list(APPEND RoboCommander_LIBRARIES uart)

# =========================================
#         Build Actuator Libraries
# =========================================

# DC Motor
add_library(dcmotor SHARED actuators/dc_motor.cpp)
target_link_libraries(dcmotor PUBLIC utils ${pigpiod_if2_LIBRARY})
list(APPEND RoboCommander_LIBRARIES dcmotor)

# Servo
add_library(servo SHARED actuators/servo.cpp)
target_link_libraries(servo PUBLIC ${pigpiod_if2_LIBRARY})
list(APPEND RoboCommander_LIBRARIES servo)

# =========================================
#         Build Sensor Libraries
# =========================================

# Encoder
add_library(encoder SHARED sensors/encoder.cpp)
target_link_libraries(encoder PUBLIC ${pigpiod_if2_LIBRARY})
list(APPEND RoboCommander_LIBRARIES encoder)

# IMU
add_library(rtimu SHARED sensors/generic_rtimu.cpp)
target_link_libraries(rtimu PUBLIC ${RTIMULib})
list(APPEND RoboCommander_LIBRARIES rtimu)

# MPU-9250
add_library(mpu9250 SHARED sensors/mpu9250.cpp)
target_link_libraries(mpu9250 PUBLIC utils serial ${pigpiod_if2_LIBRARY})
list(APPEND RoboCommander_LIBRARIES mpu9250)

# BNO-055 - Serial (UART)
add_library(bno055_uart SHARED sensors/bno055.cpp)
target_link_libraries(bno055_uart PUBLIC uart)
list(APPEND RoboCommander_LIBRARIES bno055_uart)
# BNO-055 - I2C
add_library(bno055_i2c SHARED sensors/bno055_i2c.cpp)
target_link_libraries(bno055_i2c PUBLIC i2c tca9548a)
list(APPEND RoboCommander_LIBRARIES bno055_i2c)

# Camera - D415
if(WITH_LIBREALSENSE)
     add_library(camera_d415 SHARED sensors/camera_d415.cpp)
     target_link_libraries(camera_d415 img_utils cv_utils ${realsense2_LIBRARY})
     list(APPEND RoboCommander_LIBRARIES camera_d415)
endif(WITH_LIBREALSENSE)

# =========================================
#      Build Controller Libraries
# =========================================

# PID
add_library(pid SHARED controllers/pid.cpp)
list(APPEND RoboCommander_LIBRARIES pid)

# Pure Pursuit
add_library(purepursuit SHARED controllers/waypoint_follower.cpp)
list(APPEND RoboCommander_LIBRARIES purepursuit)

# Camera Gimbal Controller
add_library(camera_gimbal SHARED devices/camera_gimbal.cpp)
target_link_libraries(camera_gimbal PUBLIC pid bno055_uart bno055_i2c pca9685)
list(APPEND RoboCommander_LIBRARIES camera_gimbal)
# =========================================
#      Build Algorithm Libraries
# =========================================

# EKF
add_library(ekf SHARED algorithms/ekf.cpp)
target_link_libraries(ekf PUBLIC ${ARMADILLO_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES ekf)

# # iSAM
# add_library(isam SHARED algorithms/isam.cpp)
# target_link_libraries(isam PUBLIC ${LIBS})

# VBOATS - RGBD Obstacle Avoidance
add_library(uvmap_utils SHARED algorithms/vboats/uvmap_utils.cpp)
target_link_libraries(uvmap_utils PUBLIC utils img_utils cv_utils ${OpenCV_LIBS})
list(APPEND RoboCommander_LIBRARIES uvmap_utils)

add_library(vboats SHARED algorithms/vboats/vboats.cpp)
target_link_libraries(vboats PUBLIC utils img_utils cv_utils ${OpenCV_LIBS})
list(APPEND RoboCommander_LIBRARIES vboats)

# =========================================
#      Build Device Libraries
# =========================================

# RoboClaw
add_library(roboclaw SHARED devices/roboclaw.cpp)
target_link_libraries(roboclaw PUBLIC ${pigpiod_if2_LIBRARY})
list(APPEND RoboCommander_LIBRARIES roboclaw)

# PWM Driver - PCA-9685
add_library(pca9685 SHARED devices/pca9685.cpp)
target_link_libraries(pca9685 PUBLIC i2c ${pigpiod_if2_LIBRARY})
list(APPEND RoboCommander_LIBRARIES pca9685)

# I2C Multiplexer - TCA9548A
add_library(tca9548a SHARED devices/tca9548a.cpp)
target_link_libraries(tca9548a PUBLIC i2c)
list(APPEND RoboCommander_LIBRARIES tca9548a)

# =========================================
#      Build Interface Libraries
# =========================================

# Android app interface
add_library(android_app_interface SHARED interfaces/android_app_interface.cpp)
target_link_libraries(android_app_interface PUBLIC udp)
list(APPEND RoboCommander_LIBRARIES android_app_interface)

# =========================================
#         Build Drivetrain Profiles
# =========================================

# 4-wheel drive robot using two roboclaws
add_library(dualclaw SHARED drivetrains/dual_roboclaw.cpp)
target_link_libraries(dualclaw PUBLIC utils roboclaw ${pigpiod_if2_LIBRARY})
list(APPEND RoboCommander_LIBRARIES dualclaw)

# =========================================
#         Build Robot Profiles
# =========================================

# Swanson V2
add_library(swansonV2 SHARED robots/swansonV2.cpp)
target_link_libraries(swansonV2 PUBLIC dualclaw android_app_interface rtimu ${pigpiod_if2_LIBRARY})
list(APPEND RoboCommander_LIBRARIES swansonV2)

# =========================================
#  Combine everthing into single variable
# =========================================
set(RoboCommander_LIBRARIES ${RoboCommander_LIBRARIES} PARENT_SCOPE)

# ===========================
#          INSTALL
# ===========================
install(
     TARGETS ${RoboCommander_LIBRARIES}
     EXPORT RoboCommanderTargets
     DESTINATION "${PRJOECT_LIB_DEST}"
)

install(
     DIRECTORY "${INCS_DIR}/"
     DESTINATION "${INC_DEST_DIR}"
     FILES_MATCHING PATTERN "*.h"
)

# ===========================
#          DEBUG
# ===========================
# message(STATUS "Headers to install ------")
# foreach(dir ${RoboCommander_HEADERS})
#      message(STATUS "         included='${dir}'")
# endforeach()

# message(STATUS "Libraries to install ------")
# foreach(dir ${RoboCommander_LIBRARIES})
#      message(STATUS "         included='${dir}'")
# endforeach()

# foreach(dir ${LIBS})
#      message(STATUS "         included='${dir}'")
# endforeach()
