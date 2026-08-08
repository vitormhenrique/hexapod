cmake_minimum_required(VERSION 3.16)

project(openrb150_portable_controller LANGUAGES CXX)

include(CTest)

# The portable target is intentionally source-policy based instead of holding a
# second long allow-list beside PlatformIO's native environment. Every source
# under src/ is included unless it is the known owner of physical I/O, RTOS task
# scheduling, or the Arduino entry point. Adding a source that accidentally
# reaches Arduino/FreeRTOS/DYNAMIXEL/Wire causes this host build to fail.
set(HEXAPOD_EMBEDDED_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app/tasks.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/board/board.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/config/qwiic_openlog.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/dxl/dxl_bus.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/sensors/bno085.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/sensors/finger_sensor.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/sensors/i2c_bus.cpp")
  list(APPEND HEXAPOD_EMBEDDED_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/sensors/qwiic_debug_oled.cpp")

file(GLOB_RECURSE HEXAPOD_PORTABLE_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
foreach(embedded_source IN LISTS HEXAPOD_EMBEDDED_SOURCES)
  list(REMOVE_ITEM HEXAPOD_PORTABLE_SOURCES "${embedded_source}")
endforeach()
list(SORT HEXAPOD_PORTABLE_SOURCES)

add_library(hexapod_controller_portable STATIC ${HEXAPOD_PORTABLE_SOURCES})
add_library(hexapod::controller_portable ALIAS hexapod_controller_portable)

target_compile_features(hexapod_controller_portable PUBLIC cxx_std_17)
target_include_directories(hexapod_controller_portable
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/ChannelPack>)

if(BUILD_TESTING)
  add_executable(hexapod_portable_smoke cmake/portable_smoke.cpp)
  target_link_libraries(hexapod_portable_smoke
    PRIVATE hexapod::controller_portable)
  add_test(NAME hexapod_portable_smoke COMMAND hexapod_portable_smoke)
endif()