cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)

if(VERBOSE)
message(STATUS "------ BUILDING COMMUNICATION DRIVERS")
endif(VERBOSE)

include_directories(${INCS_DIR}/comms comms)

# Serial
add_library(serial SHARED serial.cpp)
target_link_libraries(serial utils ${LIBS})

# UDP
add_library(udp SHARED udp.cpp)
target_link_libraries(udp ${LIBS})

if(BUILD_TESTS)

     if(VERBOSE)
     message(STATUS "------------ Building tests")
     endif(VERBOSE)

     # Serial
     include_directories(${TEST_DIR}/comms)
     add_executable(test_serial ${TEST_DIR}/comms/test_serial.cpp)
     target_link_libraries(test_serial serial)

     # UDP
     add_executable(test_udp ${TEST_DIR}/comms/test_udp.cpp)
     target_link_libraries(test_udp udp)

endif(BUILD_TESTS)


if(VERBOSE)
     get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
     foreach(dir ${dirs})
          message(STATUS "         included='${dir}'")
     endforeach()
endif(VERBOSE)
