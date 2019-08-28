cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)

# ===========================
#     Set Build Options
# ===========================

add_library(utils SHARED utilities/utils.cpp)
list(APPEND RoboCommander_LIBRARIES utils)

add_library(serial SHARED communication/serial.cpp)
target_link_libraries(serial PUBLIC ${RoboCommander_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES serial)

# i2c
add_library(i2c SHARED communication/i2c.cpp)
target_link_libraries(i2c PUBLIC ${RoboCommander_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES i2c)

# UDP
add_library(udp SHARED communication/udp.cpp)
target_link_libraries(udp PUBLIC ${RoboCommander_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES udp)

# UART
add_library(uart SHARED communication/uart.cpp)
target_link_libraries(uart PUBLIC ${RoboCommander_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES uart)

# DC Motor
add_library(dcmotor SHARED actuators/dc_motor.cpp)
target_link_libraries(dcmotor PUBLIC ${RoboCommander_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES dcmotor)

# Servo
add_library(servo SHARED actuators/servo.cpp)
target_link_libraries(servo PUBLIC ${RoboCommander_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES servo)

# Encoder
add_library(encoder SHARED sensors/encoder.cpp)
target_link_libraries(encoder PUBLIC ${RoboCommander_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES encoder)

# IMU
add_library(imu SHARED sensors/imu.cpp)
target_link_libraries(imu PUBLIC RTIMULib ${RoboCommander_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES imu)

# MPU-9250
add_library(mpu9250 SHARED sensors/mpu9250.cpp)
target_link_libraries(mpu9250 PUBLIC ${RoboCommander_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES mpu9250)

# BNO-055
add_library(bno055 SHARED sensors/bno055.cpp)
target_link_libraries(bno055 PUBLIC ${RoboCommander_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES bno055)

# # Camera
# add_library(camera SHARED camera.cpp)
# target_link_libraries(camera ${LIBS})

if(BUILD_CONTROLS)
     # PID
     add_library(pid SHARED controllers/pid.cpp)
     target_link_libraries(pid PUBLIC ${RoboCommander_LIBRARIES})
     list(APPEND RoboCommander_LIBRARIES pid)

     # Pure Pursuit
     add_library(purepursuit SHARED controllers/waypoint_follower.cpp)
     target_link_libraries(purepursuit PUBLIC ${RoboCommander_LIBRARIES} ${EXTERNAL_PACKAGE_LIBRARIES} python2.7)
     list(APPEND RoboCommander_LIBRARIES purepursuit)
endif(BUILD_CONTROLS)

# EKF
add_library(ekf SHARED algorithms/ekf.cpp)
target_link_libraries(ekf PUBLIC ${RoboCommander_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES ekf)

# # iSAM
# add_library(isam SHARED algorithms/isam.cpp)
# target_link_libraries(isam PUBLIC ${LIBS})

# RoboClaw
add_library(roboclaw SHARED devices/roboclaw.cpp)
target_link_libraries(roboclaw PUBLIC ${RoboCommander_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES roboclaw)

# PWM Driver - PCA-9685
add_library(pca9685 SHARED devices/pca9685.cpp)
target_link_libraries(pca9685 PUBLIC ${RoboCommander_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES pca9685)

# Camera Gimbal
add_library(camera_gimbal SHARED devices/camera_gimbal.cpp)
target_link_libraries(camera_gimbal PUBLIC ${RoboCommander_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES camera_gimbal)


# 4-wheel drive robot using two roboclaws
add_library(dualclaw SHARED vehicle_profiles/dual_roboclaw.cpp)
target_link_libraries(dualclaw PUBLIC ${RoboCommander_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES dualclaw)

# 4-wheel drive robot using two roboclaws
add_library(swansonV2 SHARED robots/swansonV2.cpp)
target_link_libraries(swansonV2 PUBLIC ${RoboCommander_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES swansonV2)

set(RoboCommander_LIBRARIES ${RoboCommander_LIBRARIES} PARENT_SCOPE)

# ===========================
#          DEBUG
# ===========================
# foreach(dir ${LIBS})
#      message(STATUS "         included='${dir}'")
# endforeach()
