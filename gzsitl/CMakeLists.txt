cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_program(PYTHON python)

if (NOT PYTHON)
    message(SEND_ERROR "Python not found")
endif()

# Generate Mavlink Messages
if(NOT EXISTS ${CMAKE_BINARY_DIR}/lib/)
    execute_process(
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/../modules/mavlink
            COMMAND
            ${PYTHON}
            -m pymavlink.tools.mavgen
            -o ${CMAKE_BINARY_DIR}/lib
            --lang C
            ./message_definitions/v1.0/common.xml)
endif()

# Find Gazebo and Includes
find_package(gazebo REQUIRED)
include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${CMAKE_BINARY_DIR}/lib/common
    ${CMAKE_BINARY_DIR}/lib)
link_directories(
    ${GAZEBO_LIBRARY_DIRS})

if (INSTALL_PATH)
    set(GZSITL_PLUGIN_PATH ${INSTALL_PATH})
else()
    set(GZSITL_PLUGIN_PATH ${GAZEBO_PLUGIN_PATH})
endif()

set(CUSTOM_COMPILE_FLAGS "-Wall")
set(LDFLAGS "-z noexecstack -z relro -z now")
set(CFLAGS "-fstack-protector-all -fPIE -fPIC -O2 -D_FORTIFY_SOURCE=2")
set(CFLAGS "${CFLAGS} -Wformat -Wformat-security")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CFLAGS} ${CUSTOM_COMPILE_FLAGS}")

# Build Plugin
add_library(gzsitl_plugin SHARED
    gzsitl_plugin.cc
    mavserver.cc
)
target_link_libraries(gzsitl_plugin ${GAZEBO_libraries} ${LDFLAGS})

# Set Gazebo Environment Variables

install(TARGETS gzsitl_plugin DESTINATION ${GZSITL_PLUGIN_PATH})
