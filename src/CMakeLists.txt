cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

# ===========================
#     Set Build Options
# ===========================
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-narrowing -Wno-write-strings -Wno-format -Wno-conversion-null")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-narrowing -Wno-write-strings -Wno-format -Wno-conversion-null")
# =========================================
#          Build Common Libraries
# =========================================
add_library(utils SHARED utilities/utils.cpp)
set(${PROJECT_NAME}_LIBRARIES utils)

add_library(image_utils SHARED utilities/image_utils.cpp)
target_link_libraries(image_utils PUBLIC utils ${OpenCV_LIBS} ${Matplotlib_LIBS})
list(APPEND ${PROJECT_NAME}_LIBRARIES image_utils)

add_library(plot_utils SHARED utilities/plot_utils.cpp)
target_link_libraries(plot_utils PUBLIC ${OpenCV_LIBS})
list(APPEND ${PROJECT_NAME}_LIBRARIES plot_utils)

# =========================================
#      Build Communication Libraries
# =========================================
add_library(serial SHARED communication/serial.cpp)
target_link_libraries(serial PUBLIC ${Boost_LIBRARIES})
list(APPEND ${PROJECT_NAME}_LIBRARIES serial)

# i2c
add_library(i2c SHARED communication/i2c.cpp)
target_link_libraries(i2c PUBLIC ${pigpiod_if2_LIBRARY})
list(APPEND ${PROJECT_NAME}_LIBRARIES i2c)

# UDP
add_library(udp SHARED communication/udp.cpp)
list(APPEND ${PROJECT_NAME}_LIBRARIES udp)

# UART
add_library(uart SHARED communication/uart.cpp)
list(APPEND ${PROJECT_NAME}_LIBRARIES uart)

# =========================================
#         Build Actuator Libraries
# =========================================

# DC Motor
add_library(dcmotor SHARED actuators/dc_motor.cpp)
target_link_libraries(dcmotor PUBLIC utils ${pigpiod_if2_LIBRARY})
list(APPEND ${PROJECT_NAME}_LIBRARIES dcmotor)

# Servo
add_library(servo SHARED actuators/servo.cpp)
target_link_libraries(servo PUBLIC ${pigpiod_if2_LIBRARY})
list(APPEND ${PROJECT_NAME}_LIBRARIES servo)

# =========================================
#         Build Sensor Libraries
# =========================================

# Encoder
add_library(encoder SHARED sensors/encoder.cpp)
target_link_libraries(encoder PUBLIC ${pigpiod_if2_LIBRARY})
list(APPEND ${PROJECT_NAME}_LIBRARIES encoder)

# IMU
add_library(rtimu SHARED sensors/generic_rtimu.cpp)
target_link_libraries(rtimu PUBLIC ${RTIMULib})
list(APPEND ${PROJECT_NAME}_LIBRARIES rtimu)

# MPU-9250
add_library(mpu9250 SHARED sensors/mpu9250.cpp)
target_link_libraries(mpu9250 PUBLIC utils serial ${pigpiod_if2_LIBRARY})
list(APPEND ${PROJECT_NAME}_LIBRARIES mpu9250)

# BNO-055 - Serial (UART)
add_library(bno055_uart SHARED sensors/bno055.cpp)
target_link_libraries(bno055_uart PUBLIC uart)
list(APPEND ${PROJECT_NAME}_LIBRARIES bno055_uart)
# BNO-055 - I2C
add_library(bno055_i2c SHARED sensors/bno055_i2c.cpp)
target_link_libraries(bno055_i2c PUBLIC i2c tca9548a)
list(APPEND ${PROJECT_NAME}_LIBRARIES bno055_i2c)

# Camera - D415
if(WITH_LIBREALSENSE)
     add_library(camera_d4xx SHARED sensors/camera_d4xx.cpp)
     target_link_libraries(camera_d4xx PUBLIC image_utils ${OpenCV_LIBS} ${realsense2_LIBRARY})
     list(APPEND ${PROJECT_NAME}_LIBRARIES camera_d4xx)
endif(WITH_LIBREALSENSE)

# =========================================
#      Build Controller Libraries
# =========================================

# PID
add_library(pid SHARED controllers/pid.cpp)
list(APPEND ${PROJECT_NAME}_LIBRARIES pid)

# Pure Pursuit
add_library(purepursuit SHARED controllers/waypoint_follower.cpp)
target_link_libraries(purepursuit PUBLIC ${ARMADILLO_LIBRARIES} ${Matplotlib_LIBS})
list(APPEND ${PROJECT_NAME}_LIBRARIES purepursuit)

# Camera Gimbal Controller
add_library(camera_gimbal SHARED devices/camera_gimbal.cpp)
target_link_libraries(camera_gimbal PUBLIC pid bno055_uart bno055_i2c pca9685)
list(APPEND ${PROJECT_NAME}_LIBRARIES camera_gimbal)
# =========================================
#      Build Algorithm Libraries
# =========================================

# EKF
add_library(ekf SHARED algorithms/ekf.cpp)
target_link_libraries(ekf PUBLIC ${ARMADILLO_LIBRARIES})
list(APPEND ${PROJECT_NAME}_LIBRARIES ekf)

# # iSAM
# add_library(isam SHARED algorithms/isam.cpp)
# target_link_libraries(isam PUBLIC ${LIBS})

# VBOATS - RGBD Obstacle Avoidance
add_library(vboats_utils SHARED algorithms/vboats/vboats_utils.cpp)
target_link_libraries(vboats_utils PUBLIC image_utils ${OpenCV_LIBS})
list(APPEND ${PROJECT_NAME}_LIBRARIES vboats_utils)

add_library(vboats_obstacle SHARED algorithms/vboats/obstacle.cpp)
target_link_libraries(vboats_obstacle PUBLIC utils)
list(APPEND ${PROJECT_NAME}_LIBRARIES vboats_obstacle)

add_library(vboats SHARED algorithms/vboats/vboats.cpp)
target_link_libraries(vboats PUBLIC vboats_utils vboats_obstacle ${PCL_LIBRARIES}) # image_utils utils
list(APPEND ${PROJECT_NAME}_LIBRARIES vboats)

# =========================================
#      Build Device Libraries
# =========================================

# RoboClaw
add_library(roboclaw SHARED devices/roboclaw.cpp)
target_link_libraries(roboclaw PUBLIC ${pigpiod_if2_LIBRARY})
list(APPEND ${PROJECT_NAME}_LIBRARIES roboclaw)

add_library(roboclaw_px4 SHARED devices/roboclaw_px4.cpp)
target_link_libraries(roboclaw PUBLIC ${pigpiod_if2_LIBRARY})
list(APPEND ${PROJECT_NAME}_LIBRARIES roboclaw_px4)

# PWM Driver - PCA-9685
add_library(pca9685 SHARED devices/pca9685.cpp)
target_link_libraries(pca9685 PUBLIC i2c ${pigpiod_if2_LIBRARY})
list(APPEND ${PROJECT_NAME}_LIBRARIES pca9685)

# I2C Multiplexer - TCA9548A
add_library(tca9548a SHARED devices/tca9548a.cpp)
target_link_libraries(tca9548a PUBLIC i2c)
list(APPEND ${PROJECT_NAME}_LIBRARIES tca9548a)

# =========================================
#      Build Interface Libraries
# =========================================

# Android app interface
add_library(android_app_interface SHARED interfaces/android_app_interface.cpp)
target_link_libraries(android_app_interface PUBLIC udp)
list(APPEND ${PROJECT_NAME}_LIBRARIES android_app_interface)

# =========================================
#         Build Drivetrain Profiles
# =========================================

# 4-wheel drive robot using two roboclaws
add_library(dualclaw SHARED drivetrains/dual_roboclaw.cpp)
target_link_libraries(dualclaw PUBLIC utils roboclaw ${pigpiod_if2_LIBRARY})
list(APPEND ${PROJECT_NAME}_LIBRARIES dualclaw)

# =========================================
#         Build Robot Profiles
# =========================================

# Swanson V2
add_library(swansonV2 SHARED robots/swansonV2.cpp)
target_link_libraries(swansonV2 PUBLIC dualclaw android_app_interface rtimu ${pigpiod_if2_LIBRARY})
list(APPEND ${PROJECT_NAME}_LIBRARIES swansonV2)

# =========================================
#  Combine everthing into single variable
# =========================================
set(${PROJECT_NAME}_LIBRARIES ${${PROJECT_NAME}_LIBRARIES} PARENT_SCOPE)

# ===========================
#          INSTALL
# ===========================
install(
     TARGETS ${${PROJECT_NAME}_LIBRARIES}
     EXPORT ${PROJECT_NAME}Targets
     DESTINATION "${PRJOECT_LIB_DEST}"
)

install(
     DIRECTORY "${INCS_DIR}/"
     DESTINATION "${INC_DEST_DIR}"
     FILES_MATCHING PATTERN "*.h"
)
