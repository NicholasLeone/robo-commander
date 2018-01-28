cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

message(STATUS "------ BUILDING ACTUATOR DRIVERS")

set(CMAKE_CXX_STANDARD 11)

include_directories(${INCS_DIR}/actuators actuators)

add_library(dcmotor SHARED dc_motor.cpp)
target_link_libraries(dcmotor utils ${LINK_LIBS})

if(BUILD_TESTS)
     message(STATUS "------------ Building tests")
     include_directories(${TEST_DIR}/actuators)
     add_executable(test_dcmotor ${TEST_DIR}/actuators/test_dc.cpp)
     target_link_libraries(test_dcmotor dcmotor ${SHARED})
endif(BUILD_TESTS)


if(VERBOSE)
     get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
     foreach(dir ${dirs})
          message(STATUS "         included='${dir}'")
     endforeach()
endif(VERBOSE)
