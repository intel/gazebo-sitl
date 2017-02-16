cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Check if submodules have been initialized
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/modules/mavlink_vehicles/CMakeLists.txt)
    message(SEND_ERROR "mavlink_vehicles submodule not found")
    return()
endif()

# Find Gazebo
find_package(gazebo REQUIRED)

# Gazebo Include and Link directories
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Find ignition-math
find_package(ignition-math3 REQUIRED)

# ignition-math Include and Link directories
include_directories(${IGNITION-MATH_INCLUDE_DIRS})
link_directories(${IGNITION-MATH_LIBRARY_DIRS})

# Custom instalation paths
if (GZSITL_PLUGIN_INSTALL_PATH)
    GET_FILENAME_COMPONENT(GZSITL_PLUGIN_PATH ${GZSITL_PLUGIN_INSTALL_PATH} ABSOLUTE)
else()
    set(GZSITL_PLUGIN_PATH ${GAZEBO_PLUGIN_PATH})
endif()

# Gazebo 7 does not provide a ${GAZEBO_MODEL_PATH} variable.
# That's why we are accessing it relatively to ${GAZEBO_MEDIA_PATH}.
# A proper ${GAZEBO_MODEL_PATH} variable might be available in Gazebo 8+.
if (GZSITL_MODEL_INSTALL_PATH)
    GET_FILENAME_COMPONENT(GZSITL_MODEL_PATH ${GZSITL_MODEL_INSTALL_PATH} ABSOLUTE)
elseif(GAZEBO_MODEL_PATH)
    set(GZSITL_MODEL_PATH ${GAZEBO_MODEL_PATH})
else()
    GET_FILENAME_COMPONENT(GZSITL_MODEL_PATH ${GAZEBO_MEDIA_PATH}/../models ABSOLUTE)
endif()

# Submodule includes
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/modules/mavlink_vehicles/mavlink_vehicles)

# Subdirectories
add_subdirectory(modules/mavlink_vehicles)
add_subdirectory(gzsitl)
add_subdirectory(models)

