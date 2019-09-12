cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)

# ===========================
#     Set Build Options
# ===========================


# =========================================
#          Build Common Libraries
# =========================================
add_library(utils SHARED utilities/utils.cpp)
set(RoboCommander_LIBRARIES utils)

# =========================================
#      Build Communication Libraries
# =========================================
add_library(serial SHARED communication/serial.cpp)
target_link_libraries(serial PUBLIC ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES serial)

# i2c
add_library(i2c SHARED communication/i2c.cpp)
target_link_libraries(i2c PUBLIC ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES i2c)

# UDP
add_library(udp SHARED communication/udp.cpp)
target_link_libraries(udp PUBLIC ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES udp)

# UART
add_library(uart SHARED communication/uart.cpp)
target_link_libraries(uart PUBLIC ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES uart)

# =========================================
#         Build Actuator Libraries
# =========================================

# DC Motor
add_library(dcmotor SHARED actuators/dc_motor.cpp)
target_link_libraries(dcmotor PUBLIC utils ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES dcmotor)

# Servo
add_library(servo SHARED actuators/servo.cpp)
target_link_libraries(servo PUBLIC ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES servo)

# =========================================
#         Build Sensor Libraries
# =========================================

# Encoder
add_library(encoder SHARED sensors/encoder.cpp)
target_link_libraries(encoder PUBLIC ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES encoder)

# IMU
add_library(rtimu SHARED sensors/generic_rtimu.cpp)
target_link_libraries(rtimu PUBLIC RTIMULib ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES rtimu)

# MPU-9250
add_library(mpu9250 SHARED sensors/mpu9250.cpp)
target_link_libraries(mpu9250 PUBLIC utils serial ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES mpu9250)

# BNO-055 - Serial (UART)
add_library(bno055_uart SHARED sensors/bno055.cpp)
target_link_libraries(bno055_uart PUBLIC uart ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES bno055_uart)
# BNO-055 - I2C
add_library(bno055_i2c SHARED sensors/bno055_i2c.cpp)
target_link_libraries(bno055_i2c PUBLIC i2c tca9548a ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES bno055_i2c)

# Camera - D415
add_library(camera_d415 SHARED sensors/camera_d415.cpp)
target_link_libraries(camera_d415 ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES camera_d415)

# =========================================
#      Build Controller Libraries
# =========================================

# PID
add_library(pid SHARED controllers/pid.cpp)
target_link_libraries(pid PUBLIC ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES pid)

# Pure Pursuit
add_library(purepursuit SHARED controllers/waypoint_follower.cpp)
target_link_libraries(purepursuit PUBLIC ${RoboCommander_EXTERNAL_LIBRARIES} ${EXTERNAL_PACKAGE_LIBRARIES} python2.7)
list(APPEND RoboCommander_LIBRARIES purepursuit)

# Camera Gimbal Controller
add_library(camera_gimbal SHARED devices/camera_gimbal.cpp)
target_link_libraries(camera_gimbal PUBLIC pid bno055_uart bno055_i2c pca9685 ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES camera_gimbal)
# =========================================
#      Build Algorithm Libraries
# =========================================

# EKF
add_library(ekf SHARED algorithms/ekf.cpp)
target_link_libraries(ekf PUBLIC ${RoboCommander_EXTERNAL_LIBRARIES} ${EXTERNAL_PACKAGE_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES ekf)

# # iSAM
# add_library(isam SHARED algorithms/isam.cpp)
# target_link_libraries(isam PUBLIC ${LIBS})

# =========================================
#      Build Device Libraries
# =========================================

# RoboClaw
add_library(roboclaw SHARED devices/roboclaw.cpp)
target_link_libraries(roboclaw PUBLIC ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES roboclaw)

# PWM Driver - PCA-9685
add_library(pca9685 SHARED devices/pca9685.cpp)
target_link_libraries(pca9685 PUBLIC i2c ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES pca9685)

# I2C Multiplexer - TCA9548A
add_library(tca9548a SHARED devices/tca9548a.cpp)
target_link_libraries(tca9548a PUBLIC i2c ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES tca9548a)

# =========================================
#      Build Interface Libraries
# =========================================

# Android app interface
add_library(android_app_interface SHARED interfaces/android_app_interface.cpp)
target_link_libraries(android_app_interface PUBLIC udp ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES android_app_interface)

# =========================================
#         Build Drivetrain Profiles
# =========================================

# 4-wheel drive robot using two roboclaws
add_library(dualclaw SHARED drivetrains/dual_roboclaw.cpp)
target_link_libraries(dualclaw PUBLIC utils roboclaw ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_LIBRARIES dualclaw)

# =========================================
#         Build Robot Profiles
# =========================================

# Swanson V2
add_library(swansonV2 SHARED robots/swansonV2.cpp)
target_link_libraries(swansonV2 PUBLIC dualclaw android_app_interface rtimu ${RoboCommander_EXTERNAL_LIBRARIES})
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
