cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CUSTOM_COMPILE_FLAGS "${CUSTOM_COMPILE_FLAGS} -Wall")
set(LDFLAGS "${LDFLAGS} -z noexecstack -z relro -z now")
set(CFLAGS "${CFLAGS} -fstack-protector-all -fPIE -fPIC -O2 -D_FORTIFY_SOURCE=2")
set(CFLAGS "${CFLAGS} -Wformat -Wformat-security")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CFLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CUSTOM_COMPILE_FLAGS}")

add_library(gzsitl_plugin SHARED gzsitl_plugin.cc)
add_library(gzsitl_visflags_plugin SHARED gzsitl_visflags_plugin.cc)

target_link_libraries(gzsitl_plugin mavlink_vehicles ${GAZEBO_libraries})
target_link_libraries(gzsitl_visflags_plugin ${GAZEBO_libraries})

install(TARGETS gzsitl_plugin DESTINATION ${GZSITL_PLUGIN_PATH})
install(TARGETS gzsitl_visflags_plugin DESTINATION ${GZSITL_PLUGIN_PATH})
