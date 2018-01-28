cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

message(STATUS "------ BUILDING PROGRAMMING UTILITIES")

include_directories(${INCS_DIR}/utils utils)
add_library(utils SHARED utils.cpp)
target_link_libraries(utils ${LINK_LIBS})

if(BUILD_TESTS)
     message(STATUS "------------ Building utility tests")
     include_directories(${TEST_DIR}/utils)
     add_executable(test_utils ${TEST_DIR}/utils/test_utils.cpp)
     target_link_libraries(test_utils utils)
endif(BUILD_TESTS)

if(VERBOSE)
     get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
     foreach(dir ${dirs})
          message(STATUS "         included='${dir}'")
     endforeach()
endif(VERBOSE)
