cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# ===========================
#     Set Build Options
# ===========================
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-narrowing -Wwrite-strings")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-narrowing -Wwrite-strings")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-narrowing -Wno-write-strings -Wno-format -Wno-conversion-null")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-narrowing -Wno-write-strings -Wno-format -Wno-conversion-null")
# =========================================
#          Build Common Libraries
# =========================================
add_library(utils SHARED utilities/utils.cpp)
set(robocommander_LIBRARIES utils)

if(OpenCV_FOUND)
     add_library(image_utils SHARED utilities/image_utils.cpp)
     target_link_libraries(image_utils PUBLIC utils ${OpenCV_LIBRARIES})
     list(APPEND robocommander_LIBRARIES image_utils)

     add_library(plot_utils SHARED utilities/plot_utils.cpp)
     target_link_libraries(plot_utils PUBLIC ${OpenCV_LIBRARIES})
     list(APPEND robocommander_LIBRARIES plot_utils)
endif(OpenCV_FOUND)
# =========================================
#      Build Communication Libraries
# =========================================
add_library(serial SHARED communication/serial.cpp)
target_link_libraries(serial PUBLIC ${Boost_LIBRARIES})
list(APPEND robocommander_LIBRARIES serial)

# UDP
add_library(udp SHARED communication/udp.cpp)
list(APPEND robocommander_LIBRARIES udp)

# UART
add_library(uart SHARED communication/uart.cpp)
list(APPEND robocommander_LIBRARIES uart)

# i2c
if(pigpio_FOUND)
     add_library(i2c SHARED communication/i2c.cpp)
     target_link_libraries(i2c PUBLIC ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_LIBRARIES i2c)
endif(pigpio_FOUND)

# =========================================
#    Build Actuator + Sensor Libraries
# =========================================

if(pigpio_FOUND)
     # DC Motor
     add_library(dcmotor SHARED actuators/dc_motor.cpp)
     target_link_libraries(dcmotor PUBLIC utils ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_LIBRARIES dcmotor)

     # Servo
     add_library(servo SHARED actuators/servo.cpp)
     target_link_libraries(servo PUBLIC ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_LIBRARIES servo)

     # Encoder
     add_library(encoder SHARED sensors/encoder.cpp)
     target_link_libraries(encoder PUBLIC ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_LIBRARIES encoder)

     # MPU-9250
     add_library(mpu9250 SHARED sensors/mpu9250.cpp)
     target_link_libraries(mpu9250 PUBLIC utils serial ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_LIBRARIES mpu9250)
endif(pigpio_FOUND)

# IMU
if(RTIMULib)
     add_library(rtimu SHARED sensors/generic_rtimu.cpp)
     target_link_libraries(rtimu PUBLIC ${RTIMULib})
     list(APPEND robocommander_LIBRARIES rtimu)
endif(RTIMULib)

# BNO-055 - Serial (UART)
add_library(bno055_uart SHARED sensors/bno055.cpp)
target_link_libraries(bno055_uart PUBLIC uart)
list(APPEND robocommander_LIBRARIES bno055_uart)

# BNO-055 - I2C
if(pigpio_FOUND)
     add_library(bno055_i2c SHARED sensors/bno055_i2c.cpp)
     target_link_libraries(bno055_i2c PUBLIC i2c tca9548a)
     list(APPEND robocommander_LIBRARIES bno055_i2c)
endif(pigpio_FOUND)

# Camera - D415
if(realsense2_FOUND AND OpenCV_FOUND)
     add_library(camera_d4xx SHARED sensors/camera_d4xx.cpp)
     target_link_libraries(camera_d4xx PUBLIC image_utils ${OpenCV_LIBRARIES} ${realsense2_LIBRARY})
     list(APPEND robocommander_LIBRARIES camera_d4xx)
endif()

# =========================================
#      Build Controller Libraries
# =========================================

# PID
add_library(pid SHARED controllers/pid.cpp)
list(APPEND robocommander_LIBRARIES pid)

# Pure Pursuit
if(Armadillo_FOUND)
     add_library(purepursuit SHARED controllers/waypoint_follower.cpp)
     if(WITH_MATPLOTLIB AND PYTHONLIBS_FOUND)
          target_link_libraries(purepursuit PUBLIC ${ARMADILLO_LIBRARIES} ${PYTHON_LIBRARIES})
     else()
          target_link_libraries(purepursuit PUBLIC ${ARMADILLO_LIBRARIES})
     endif()
     list(APPEND robocommander_LIBRARIES purepursuit)
endif(Armadillo_FOUND)
# =========================================
#      Build Algorithm Libraries
# =========================================

# EKF
if(Armadillo_FOUND)
     add_library(ekf SHARED algorithms/ekf.cpp)
     target_link_libraries(ekf PUBLIC ${ARMADILLO_LIBRARIES})
     list(APPEND robocommander_LIBRARIES ekf)
endif(Armadillo_FOUND)

# iSAM
if(GTSAM_FOUND AND Armadillo_FOUND)
     add_library(isam SHARED algorithms/isam.cpp)
     target_link_libraries(isam PUBLIC gtsam ${ARMADILLO_LIBRARIES})
     list(APPEND robocommander_LIBRARIES isam)
endif()

# VBOATS - RGBD Obstacle Avoidance
if(OpenCV_FOUND)
     add_library(uvmap_utils SHARED algorithms/vboats/uvmap_utils.cpp)
     target_link_libraries(uvmap_utils PUBLIC utils image_utils ${OpenCV_LIBRARIES})
     list(APPEND robocommander_LIBRARIES uvmap_utils)

     add_library(vboats SHARED algorithms/vboats/vboats.cpp)
     target_link_libraries(vboats PUBLIC utils image_utils ${OpenCV_LIBRARIES})
     list(APPEND robocommander_LIBRARIES vboats)
endif(OpenCV_FOUND)

# =========================================
#      Build Device Libraries
# =========================================
# RoboClaw - Unix-based Serial device
add_library(roboclaw_unix SHARED devices/roboclaw_unix.cpp)
target_link_libraries(roboclaw_unix PUBLIC utils)
list(APPEND robocommander_LIBRARIES roboclaw_unix)

if(pigpio_FOUND)
     # RoboClaw
     add_library(roboclaw SHARED devices/roboclaw.cpp)
     target_link_libraries(roboclaw PUBLIC ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_LIBRARIES roboclaw)

     # PWM Driver - PCA-9685
     add_library(pca9685 SHARED devices/pca9685.cpp)
     target_link_libraries(pca9685 PUBLIC i2c ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_LIBRARIES pca9685)

     # I2C Multiplexer - TCA9548A
     add_library(tca9548a SHARED devices/tca9548a.cpp)
     target_link_libraries(tca9548a PUBLIC i2c)
     list(APPEND robocommander_LIBRARIES tca9548a)

     # Camera Gimbal Controller
     add_library(camera_gimbal SHARED devices/camera_gimbal.cpp)
     target_link_libraries(camera_gimbal PUBLIC pid bno055_uart bno055_i2c pca9685)
     list(APPEND robocommander_LIBRARIES camera_gimbal)
endif(pigpio_FOUND)

# =========================================
#      Build Interface Libraries
# =========================================

# Android app interface
add_library(android_app_interface SHARED interfaces/android_app_interface.cpp)
target_link_libraries(android_app_interface PUBLIC udp)
list(APPEND robocommander_LIBRARIES android_app_interface)

# =========================================
#    Build Drivetrain + Robot Profiles
# =========================================
#add_library(roboclaw_px4 SHARED devices/roboclaw_px4.cpp)
#target_link_libraries(roboclaw PUBLIC ${pigpiod_if2_LIBRARY})
#list(APPEND ${PROJECT_NAME}_LIBRARIES roboclaw_px4)

if(pigpio_FOUND)
     # 2-wheel diff drive robot using single roboclaws
     add_library(diffdrive_claw SHARED drivetrains/diffdrive_claw.cpp)
     target_link_libraries(diffdrive_claw PUBLIC utils roboclaw ${pigpiod_if2_LIBRARY})
     list(APPEND ${PROJECT_NAME}_LIBRARIES diffdrive_claw)

     # 4-wheel drive robot using two roboclaws
     add_library(dualclaw SHARED drivetrains/dual_roboclaw.cpp)
     target_link_libraries(dualclaw PUBLIC utils roboclaw ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_LIBRARIES dualclaw)

     # Swanson V2
     add_library(swansonV2 SHARED robots/swansonV2.cpp)
     target_link_libraries(swansonV2 PUBLIC dualclaw android_app_interface rtimu ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_LIBRARIES swansonV2)
endif(pigpio_FOUND)

# =========================================
#  Combine everthing into single variable
# =========================================
set(robocommander_LIBRARIES ${robocommander_LIBRARIES} PARENT_SCOPE)

# ===========================
#          INSTALL
# ===========================
install(
     TARGETS ${robocommander_LIBRARIES}
     EXPORT robocommanderTargets
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
