cmake_minimum_required(VERSION 3.16)
project(hexapod_controller_ros LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hexapod_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Build the same Arduino-free ControllerCore library used by the native firmware
# tests. The child CMake target deliberately excludes the FreeRTOS, DXL, I2C,
# board, and Arduino entry-point owners.
get_filename_component(HEXAPOD_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
add_subdirectory(
  "${HEXAPOD_ROOT}/firmware/openrb150"
  "${CMAKE_CURRENT_BINARY_DIR}/openrb150_portable")

add_library(${PROJECT_NAME}_adapter STATIC
  src/controller_core_adapter.cpp
  src/controller_core_node.cpp
  src/ros2_control_sil_adapter.cpp)
target_compile_features(${PROJECT_NAME}_adapter PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_adapter
  hexapod::controller_portable)
ament_target_dependencies(${PROJECT_NAME}_adapter
  geometry_msgs
  hexapod_msgs
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  tf2_ros)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_adapter
  hexapod::controller_portable)
ament_target_dependencies(${PROJECT_NAME}_node
  geometry_msgs
  hexapod_msgs
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  tf2_ros)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/companion_sim_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME companion_sim_bridge)
install(FILES scripts/companion_sim_protocol.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_controller_core_node
    test/test_controller_core_node.cpp)
  target_compile_features(test_controller_core_node PRIVATE cxx_std_17)
  target_link_libraries(test_controller_core_node
    ${PROJECT_NAME}_adapter
    hexapod::controller_portable)
  ament_target_dependencies(test_controller_core_node
    lifecycle_msgs
    rclcpp
    rclcpp_lifecycle)

  ament_add_gtest(test_ros2_control_sil_adapter
    test/test_ros2_control_sil_adapter.cpp)
  target_compile_features(test_ros2_control_sil_adapter PRIVATE cxx_std_17)
  target_link_libraries(test_ros2_control_sil_adapter
    ${PROJECT_NAME}_adapter
    hexapod::controller_portable)
  ament_target_dependencies(test_ros2_control_sil_adapter
    hexapod_msgs
    lifecycle_msgs
    rclcpp
    rclcpp_lifecycle
    sensor_msgs
    std_msgs)
endif()

ament_package()