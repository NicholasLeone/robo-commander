cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ===========================
#     Set Build Options
# ===========================
if(BUILD_ACTUATORS)
     add_subdirectory(actuators)
endif(BUILD_ACTUATORS)

# ===========================
#     Set Build Options
# ===========================
if(BUILD_COMMS)
     add_subdirectory(comms)
endif(BUILD_COMMS)

# ===========================
#     Set Build Options
# ===========================
if(BUILD_DEVICES)
     add_subdirectory(devices)
endif(BUILD_DEVICES)

# ===========================
#     Set Build Options
# ===========================
if(BUILD_SENSORS)
     add_subdirectory(sensors)
endif(BUILD_SENSORS)

# ===========================
#     Set Build Options
# ===========================
if(BUILD_CONTROLS)
     add_subdirectory(controls)
endif(BUILD_CONTROLS)

# ===========================
#     Set Build Options
# ===========================
if(BUILD_FILTERS)
     add_subdirectory(filters)
endif(BUILD_FILTERS)

# ===========================
#     Set Build Options
# ===========================
if(BUILD_UTILS)
     add_subdirectory(utils)
endif(BUILD_UTILS)

# ===========================
#     Set Build Options
# ===========================
if(BUILD_PROFILES)
     add_subdirectory(profiles)
endif(BUILD_PROFILES)


# ===========================
#     Set Build Options
# ===========================
if(BUILD_ROBOTS)
     add_subdirectory(robots)
endif(BUILD_ROBOTS)



# ===========================
#          DEBUG
# ===========================
# foreach(dir ${LIBS})
#      message(STATUS "         included='${dir}'")
# endforeach()
