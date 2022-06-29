cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

if(VERBOSE)
message(STATUS "------ BUILDING Tests")
endif(VERBOSE)

# ===========================
#     Set Build Options
# ===========================
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-narrowing -Wwrite-strings")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-narrowing -Wwrite-strings")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-narrowing -Wno-write-strings -Wno-format -Wno-conversion-null")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-narrowing -Wno-write-strings -Wno-format -Wno-conversion-null")

# =========================================
#            Build Common Libraries
# =========================================

# Utilities
add_executable(test_utils ${TEST_DIR}/utilities/test_utils.cpp)
target_link_libraries(test_utils PUBLIC utils)
set(robocommander_TESTS test_utils)

# # JSON Test
# add_executable(test_json ${TEST_DIR}/utils/test_json.cpp)
# target_link_libraries(test_json utils)
# list(APPEND robocommander_TESTS test_json)

if(OpenCV_FOUND)
     add_executable(test_image_utils ${TEST_DIR}/utilities/test_image_utils.cpp)
     target_link_libraries(test_image_utils PUBLIC utils image_utils plot_utils uvmap_utils vboats)
     list(APPEND robocommander_TESTS test_image_utils)
endif(OpenCV_FOUND)
# =========================================
#         Build Communication Tests
# =========================================

# Serial
add_executable(test_serial ${TEST_DIR}/communication/test_serial.cpp)
target_link_libraries(test_serial PUBLIC serial ${Boost_LIBRARIES})
list(APPEND robocommander_TESTS test_serial)

# UDP
add_executable(test_udp ${TEST_DIR}/communication/test_udp.cpp)
target_link_libraries(test_udp PUBLIC udp)
list(APPEND robocommander_TESTS test_udp)

# UART
add_executable(test_uart ${TEST_DIR}/communication/test_uart.cpp)
target_link_libraries(test_uart PUBLIC uart)
list(APPEND robocommander_TESTS test_uart)

# I2C
if(pigpio_FOUND)
     add_executable(test_i2c ${TEST_DIR}/communication/test_i2c.cpp)
     target_link_libraries(test_i2c PUBLIC i2c ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_TESTS test_i2c)
endif(pigpio_FOUND)

# =========================================
#  Build Actuator + Sensor Libraries Tests
# =========================================
if(pigpio_FOUND)
     # DC Motor
     add_executable(test_dcmotor ${TEST_DIR}/actuators/test_dc.cpp)
     target_link_libraries(test_dcmotor PUBLIC dcmotor ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_TESTS test_dcmotor)

     # Servo
     add_executable(test_servo ${TEST_DIR}/actuators/test_servo.cpp)
     target_link_libraries(test_servo PUBLIC servo ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_TESTS test_servo)

     # Encoder
     add_executable(test_encoder ${TEST_DIR}/sensors/test_encoder.cpp)
     target_link_libraries(test_encoder PUBLIC encoder ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_TESTS test_encoder)

     # MPU-9250
     add_executable(test_mpu9250 ${TEST_DIR}/sensors/test_mpu9250.cpp)
     target_link_libraries(test_mpu9250 PUBLIC serial mpu9250 ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_TESTS test_mpu9250)
endif(pigpio_FOUND)

# IMU - RtImu-based
if(RTIMULib)
     add_executable(test_rtimu ${TEST_DIR}/sensors/test_rtimu.cpp)
     target_link_libraries(test_rtimu PUBLIC rtimu ${RTIMULib})
     list(APPEND robocommander_TESTS test_rtimu)
endif(RTIMULib)

# BNO-055 - Serial (UART)
add_executable(test_bno055_uart ${TEST_DIR}/sensors/test_bno055.cpp)
target_link_libraries(test_bno055_uart PUBLIC bno055_uart uart)
list(APPEND robocommander_TESTS test_bno055_uart)

if(pigpio_FOUND)
     # BNO-055 - I2C
     add_executable(test_bno055_i2c ${TEST_DIR}/sensors/test_bno055_i2c.cpp)
     target_link_libraries(test_bno055_i2c PUBLIC bno055_i2c i2c tca9548a)
     list(APPEND robocommander_TESTS test_bno055_i2c)

     # BNO-055 - I2C Multiplexed
     add_executable(test_bno055_mux ${TEST_DIR}/sensors/test_bno055_muxed.cpp)
     target_link_libraries(test_bno055_mux PUBLIC bno055_i2c i2c tca9548a)
     list(APPEND robocommander_TESTS test_bno055_mux)
endif(pigpio_FOUND)

# Camera - D415
if(realsense2_FOUND AND OpenCV_FOUND)
     add_executable(test_minimal_d4xx ${TEST_DIR}/sensors/test_minimal_d4xx.cpp)
     target_link_libraries(test_minimal_d4xx camera_d4xx image_utils ${OpenCV_LIBRARIES} ${realsense2_LIBRARY})
     list(APPEND robocommander_TESTS test_minimal_d4xx)

     add_executable(test_camera_d4xx ${TEST_DIR}/sensors/test_camera_d4xx.cpp)
     target_link_libraries(test_camera_d4xx camera_d4xx image_utils ${OpenCV_LIBRARIES} ${realsense2_LIBRARY})
     list(APPEND robocommander_TESTS test_camera_d4xx)
endif(realsense2_FOUND AND OpenCV_FOUND)

# =========================================
#         Build Controller Tests
# =========================================

# PID
add_executable(test_pid ${TEST_DIR}/controllers/test_pid.cpp)
target_link_libraries(test_pid PUBLIC dcmotor rtimu utils pid)
list(APPEND robocommander_TESTS test_pid)

# Pure Pursuit
if(Armadillo_FOUND)
     add_executable(test_purepursuit ${TEST_DIR}/controllers/test_waypoint_follower.cpp)
     if(WITH_MATPLOTLIB AND PYTHONLIBS_FOUND)
          target_link_libraries(test_purepursuit PUBLIC purepursuit ${ARMADILLO_LIBRARIES} ${PYTHON_LIBRARIES})
     else()
          target_link_libraries(test_purepursuit PUBLIC purepursuit ${ARMADILLO_LIBRARIES})
     endif()
     list(APPEND robocommander_TESTS test_purepursuit)
endif(Armadillo_FOUND)

# =========================================
#          Build Algorithm Tests
# =========================================

# EKF
if(Armadillo_FOUND)
     add_executable(test_ekf ${TEST_DIR}/algorithms/test_ekf.cpp)
     target_link_libraries(test_ekf PUBLIC ekf utils ${ARMADILLO_LIBRARIES})
     list(APPEND robocommander_TESTS test_ekf)
endif(Armadillo_FOUND)

# ISAM2
if(GTSAM_FOUND AND Armadillo_FOUND)
     add_executable(test_isam ${TEST_DIR}/algorithms/test_isam.cpp)
     target_link_libraries(test_isam PUBLIC isam gtsam ${ARMADILLO_LIBRARIES})
     list(APPEND robocommander_TESTS test_isam)
endif()

# VBOATS - RGBD Obstacle Avoidance
if(realsense2_FOUND AND OpenCV_FOUND)
     add_executable(test_vboats ${TEST_DIR}/algorithms/vboats/test_vboats.cpp)
     target_link_libraries(test_vboats PUBLIC utils image_utils plot_utils uvmap_utils camera_d4xx vboats ${OpenCV_LIBRARIES} ${realsense2_LIBRARY})
     list(APPEND robocommander_TESTS test_vboats)
endif()

# =========================================
#         Build Device Tests
# =========================================
# RoboClaw - Unix-based Serial device
add_executable(test_roboclaw_unix ${TEST_DIR}/devices/test_roboclaw_unix.cpp)
target_link_libraries(test_roboclaw_unix PUBLIC roboclaw_unix)
list(APPEND robocommander_TESTS test_roboclaw_unix)

if(pigpio_FOUND)
     # RoboClaw
     add_executable(test_roboclaw ${TEST_DIR}/devices/test_roboclaw.cpp)
     target_link_libraries(test_roboclaw PUBLIC roboclaw ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_TESTS test_roboclaw)

     # Two RoboClaws over usb cable
     add_executable(test_roboclaw_usb ${TEST_DIR}/devices/test_roboclaw_usb.cpp)
     target_link_libraries(test_roboclaw_usb PUBLIC roboclaw ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_TESTS test_roboclaw_usb)

     # PWM Driver - PCA-9685
     add_executable(test_pca9685 ${TEST_DIR}/devices/test_pca9685.cpp)
     target_link_libraries(test_pca9685 PUBLIC pca9685 ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_TESTS test_pca9685)

     # LED Color Test
     add_executable(test_leds ${TEST_DIR}/devices/test_leds.cpp)
     target_link_libraries(test_leds PUBLIC pca9685 ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_TESTS test_leds)

     # I2C Multiplexer - TCA9548a
     add_executable(test_tca9548a ${TEST_DIR}/devices/test_tca9548a.cpp)
     target_link_libraries(test_tca9548a PUBLIC bno055_i2c tca9548a ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_TESTS test_tca9548a)

     # Camera Gimbal Angle Controller
     add_executable(test_camera_gimbal ${TEST_DIR}/controllers/test_camera_gimbal.cpp)
     target_link_libraries(test_camera_gimbal PUBLIC pca9685 bno055_uart bno055_i2c utils pid ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_TESTS test_camera_gimbal)

     # Camera Gimbal Assembly Test
     add_executable(test_camera_gimbal_assembly ${TEST_DIR}/devices/test_camera_gimbal_assembly.cpp)
     target_link_libraries(test_camera_gimbal_assembly PUBLIC utils camera_gimbal ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_TESTS test_camera_gimbal_assembly)
endif(pigpio_FOUND)

# =========================================
#         Build Interface Tests
# =========================================

# Android app interface
add_executable(test_android_iface ${TEST_DIR}/interfaces/test_android_app_interface.cpp)
target_link_libraries(test_android_iface PUBLIC utils android_app_interface)
list(APPEND robocommander_TESTS test_android_iface)

# =========================================
#  Build Drivetrain + Robot Profile Tests
# =========================================
if(pigpio_FOUND)
     # 4-wheel drive robot using two roboclaws
     add_executable(test_dualclaw ${TEST_DIR}/drivetrains/test_dual_roboclaw.cpp)
     target_link_libraries(test_dualclaw PUBLIC dualclaw ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_TESTS test_dualclaw)

     # Swanson V2
     add_executable(test_swansonV2 ${TEST_DIR}/robots/test_swansonV2.cpp)
     target_link_libraries(test_swansonV2 PUBLIC swansonV2 utils ${pigpiod_if2_LIBRARY})
     list(APPEND robocommander_TESTS test_swansonV2)
endif(pigpio_FOUND)

# ===========================
#          INSTALL
# ===========================
install(
     TARGETS ${robocommander_TESTS}
     EXPORT robocommanderTargets
     DESTINATION "${EXEC_DEST_DIR}"
)

# ===========================
#          DEBUG
# ===========================
# message(STATUS " Tests to install ------")
# foreach(dir ${robocommander_TESTS})
#      message(STATUS "         included='${dir}'")
# endforeach()
