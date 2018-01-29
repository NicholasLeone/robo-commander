cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)

if(VERBOSE)
message(STATUS "------ BUILDING ACTUATOR DRIVERS")
endif(VERBOSE)

include_directories(${INCS_DIR}/actuators actuators)

# DC Motor
add_library(dcmotor SHARED dc_motor.cpp)
target_link_libraries(dcmotor utils ${LIBS})

# Servo
add_library(servo SHARED servo.cpp)
target_link_libraries(servo ${LIBS})

if(BUILD_TESTS)

     if(VERBOSE)
     message(STATUS "------------ Building tests")
     endif(VERBOSE)

     # DC Motor
     include_directories(${TEST_DIR}/actuators)
     add_executable(test_dcmotor ${TEST_DIR}/actuators/test_dc.cpp)
     target_link_libraries(test_dcmotor dcmotor)

     # Servo
     add_executable(test_servo ${TEST_DIR}/actuators/test_servo.cpp)
     target_link_libraries(test_servo servo)

endif(BUILD_TESTS)


if(VERBOSE)
     get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
     foreach(dir ${dirs})
          message(STATUS "         included='${dir}'")
     endforeach()
endif(VERBOSE)
