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
