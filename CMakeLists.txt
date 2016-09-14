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

# Custom instalation paths
if (GZSITL_INSTALL_PATH)
    set(GZSITL_PLUGIN_PATH ${GZSITL_INSTALL_PATH})
else()
    set(GZSITL_PLUGIN_PATH ${GAZEBO_PLUGIN_PATH})
endif()

# Submodule includes
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/modules/mavlink_vehicles/mavlink_vehicles)

# Subdirectories
add_subdirectory(modules/mavlink_vehicles)
add_subdirectory(gzsitl)
add_subdirectory(models)

