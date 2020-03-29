cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)

if(VERBOSE)
message(STATUS "------ BUILDING Tests")
endif(VERBOSE)

# =========================================
#            Build Common Tests
# =========================================

# Utilities
add_executable(test_utils ${TEST_DIR}/utilities/test_utils.cpp)
target_link_libraries(test_utils PUBLIC utils ${RoboCommander_EXTERNAL_LIBRARIES})
set(RoboCommander_TESTS test_utils)

# # JSON Test
# add_executable(test_json ${TEST_DIR}/utils/test_json.cpp)
# target_link_libraries(test_json utils)
# list(APPEND RoboCommander_TESTS test_json)

add_executable(test_img_utils ${TEST_DIR}/utilities/test_image_utils.cpp)
target_link_libraries(test_img_utils PUBLIC utils img_utils cv_utils plot_utils uvmap_utils vboats)
list(APPEND RoboCommander_TESTS test_img_utils)

# =========================================
#         Build Communication Tests
# =========================================

# Serial
add_executable(test_serial ${TEST_DIR}/communication/test_serial.cpp)
target_link_libraries(test_serial PUBLIC serial ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_serial)

# UDP
add_executable(test_udp ${TEST_DIR}/communication/test_udp.cpp)
target_link_libraries(test_udp PUBLIC udp ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_udp)

# I2C
add_executable(test_i2c ${TEST_DIR}/communication/test_i2c.cpp)
target_link_libraries(test_i2c PUBLIC i2c ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_i2c)

# UART
add_executable(test_uart ${TEST_DIR}/communication/test_uart.cpp)
target_link_libraries(test_uart PUBLIC uart ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_uart)

# =========================================
#           Build Actuator Tests
# =========================================

# DC Motor
add_executable(test_dcmotor ${TEST_DIR}/actuators/test_dc.cpp)
target_link_libraries(test_dcmotor PUBLIC dcmotor ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_dcmotor)

# Servo
add_executable(test_servo ${TEST_DIR}/actuators/test_servo.cpp)
target_link_libraries(test_servo PUBLIC servo ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_servo)

# =========================================
#           Build Sensor Tests
# =========================================

# Encoder
add_executable(test_encoder ${TEST_DIR}/sensors/test_encoder.cpp)
target_link_libraries(test_encoder PUBLIC encoder ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_encoder)

# IMU
add_executable(test_rtimu ${TEST_DIR}/sensors/test_rtimu.cpp)
target_link_libraries(test_rtimu PUBLIC rtimu ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_rtimu)

# MPU-9250
add_executable(test_mpu9250 ${TEST_DIR}/sensors/test_mpu9250.cpp)
target_link_libraries(test_mpu9250 PUBLIC serial mpu9250 ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_mpu9250)

# BNO-055 - Serial (UART)
add_executable(test_bno055_uart ${TEST_DIR}/sensors/test_bno055.cpp)
target_link_libraries(test_bno055_uart PUBLIC bno055_uart ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_bno055_uart)

# BNO-055 - I2C
add_executable(test_bno055_i2c ${TEST_DIR}/sensors/test_bno055_i2c.cpp)
target_link_libraries(test_bno055_i2c PUBLIC bno055_i2c ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_bno055_i2c)

# BNO-055 - I2C Multiplexed
add_executable(test_bno055_mux ${TEST_DIR}/sensors/test_bno055_muxed.cpp)
target_link_libraries(test_bno055_mux PUBLIC bno055_i2c ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_bno055_mux)

# Camera - D415
if(WITH_LIBREALSENSE)
     add_executable(test_minimal_d4xx ${TEST_DIR}/sensors/test_minimal_d4xx.cpp)
     target_link_libraries(test_minimal_d4xx camera_d4xx)
     list(APPEND RoboCommander_TESTS test_minimal_d4xx)

     add_executable(test_camera_d4xx ${TEST_DIR}/sensors/test_camera_d4xx.cpp)
     target_link_libraries(test_camera_d4xx camera_d4xx)
     list(APPEND RoboCommander_TESTS test_camera_d4xx)
endif(WITH_LIBREALSENSE)

# =========================================
#         Build Controller Tests
# =========================================

# PID
add_executable(test_pid ${TEST_DIR}/controllers/test_pid.cpp)
target_link_libraries(test_pid PUBLIC dcmotor rtimu utils pid ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_pid)

# Pure Pursuit
add_executable(test_purepursuit ${TEST_DIR}/controllers/test_waypoint_follower.cpp)
target_link_libraries(test_purepursuit PUBLIC purepursuit ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_purepursuit)

# =========================================
#          Build Algorithm Tests
# =========================================

# EKF
add_executable(test_ekf ${TEST_DIR}/algorithms/test_ekf.cpp)
target_link_libraries(test_ekf PUBLIC ekf utils ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_ekf)

# ISAM2
# add_executable(test_isam ${TEST_DIR}/algorithms/test_isam.cpp)
# target_link_libraries(test_isam PUBLIC ${RoboCommander_LIBRARIES})
# list(APPEND RoboCommander_TESTS test_isam)

# VBOATS - RGBD Obstacle Avoidance
if(WITH_LIBREALSENSE)
     add_executable(test_vboats ${TEST_DIR}/algorithms/vboats/test_vboats.cpp)
     target_link_libraries(test_vboats PUBLIC utils img_utils plot_utils cv_utils uvmap_utils camera_d4xx vboats)
     list(APPEND RoboCommander_TESTS test_vboats)
endif(WITH_LIBREALSENSE)

# =========================================
#         Build Device Tests
# =========================================

# RoboClaw
add_executable(test_roboclaw ${TEST_DIR}/devices/test_roboclaw.cpp)
target_link_libraries(test_roboclaw PUBLIC roboclaw ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_roboclaw)

# Two RoboClaws over usb cable
add_executable(test_roboclaw_usb ${TEST_DIR}/devices/test_roboclaw_usb.cpp)
target_link_libraries(test_roboclaw_usb PUBLIC roboclaw ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_roboclaw_usb)

# PWM Driver - PCA-9685
add_executable(test_pca9685 ${TEST_DIR}/devices/test_pca9685.cpp)
target_link_libraries(test_pca9685 PUBLIC pca9685 ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_pca9685)

# Camera Gimbal Angle Controller
add_executable(test_camera_gimbal ${TEST_DIR}/controllers/test_camera_gimbal.cpp)
target_link_libraries(test_camera_gimbal PUBLIC pca9685 bno055_uart bno055_i2c utils pid ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_camera_gimbal)

# LED Color Test
add_executable(test_leds ${TEST_DIR}/devices/test_leds.cpp)
target_link_libraries(test_leds PUBLIC pca9685 ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_leds)

# Camera Gimbal Assembly Test
add_executable(test_camera_gimbal_assembly ${TEST_DIR}/devices/test_camera_gimbal_assembly.cpp)
target_link_libraries(test_camera_gimbal_assembly PUBLIC utils camera_gimbal ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_camera_gimbal_assembly)

# I2C Multiplexer - TCA9548a
add_executable(test_tca9548a ${TEST_DIR}/devices/test_tca9548a.cpp)
target_link_libraries(test_tca9548a PUBLIC bno055_i2c tca9548a ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_tca9548a)

# =========================================
#         Build Interface Tests
# =========================================

# Android app interface
add_executable(test_android_iface ${TEST_DIR}/interfaces/test_android_app_interface.cpp)
target_link_libraries(test_android_iface PUBLIC utils android_app_interface ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_android_iface)

# =========================================
#        Build Drivetrain Tests
# =========================================

# DualClaw
add_executable(test_dualclaw ${TEST_DIR}/drivetrains/test_dual_roboclaw.cpp)
target_link_libraries(test_dualclaw PUBLIC dualclaw ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_dualclaw)

# =========================================
#        Build Robot Profile Tests
# =========================================

# Swanson V2
add_executable(test_swansonV2 ${TEST_DIR}/robots/test_swansonV2.cpp)
target_link_libraries(test_swansonV2 PUBLIC swansonV2 utils ${RoboCommander_EXTERNAL_LIBRARIES})
list(APPEND RoboCommander_TESTS test_swansonV2)

# ===========================
#          INSTALL
# ===========================
install(
     TARGETS ${RoboCommander_TESTS}
     EXPORT RoboCommanderTargets
     DESTINATION "${EXEC_DEST_DIR}"
)

# ===========================
#          DEBUG
# ===========================
# message(STATUS " Tests to install ------")
# foreach(dir ${RoboCommander_TESTS})
#      message(STATUS "         included='${dir}'")
# endforeach()
