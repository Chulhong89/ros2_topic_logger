cmake_minimum_required(VERSION 3.8)
project(ros2_topic_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(topic_logger_node
  src/main.cpp
  src/topic_logger_node.cpp
  src/csv_writer.cpp
  src/retention_manager.cpp
)

target_include_directories(topic_logger_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(topic_logger_node PUBLIC cxx_std_17)

ament_target_dependencies(topic_logger_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

target_link_libraries(topic_logger_node yaml-cpp)

install(TARGETS topic_logger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install test YAML so the test launch file can resolve it via FindPackageShare.
install(FILES test/logger_test.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# ── Tests ──────────────────────────────────────────────────────────────────────

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # --- test_topic_config (header-only, no ROS runtime needed) ---
  ament_add_gtest(test_topic_config test/test_topic_config.cpp)
  target_include_directories(test_topic_config PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_compile_features(test_topic_config PUBLIC cxx_std_17)

  # --- test_event_buffer (header-only template, needs sensor_msgs for Imu) ---
  ament_add_gtest(test_event_buffer test/test_event_buffer.cpp)
  target_include_directories(test_event_buffer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_compile_features(test_event_buffer PUBLIC cxx_std_17)
  ament_target_dependencies(test_event_buffer sensor_msgs)

  # --- test_message_serializers (header-only serializers) ---
  ament_add_gtest(test_message_serializers test/test_message_serializers.cpp)
  target_include_directories(test_message_serializers PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_compile_features(test_message_serializers PUBLIC cxx_std_17)
  ament_target_dependencies(test_message_serializers
    sensor_msgs nav_msgs geometry_msgs)

  # --- test_csv_writer ---
  ament_add_gtest(test_csv_writer
    test/test_csv_writer.cpp
    src/csv_writer.cpp)
  target_include_directories(test_csv_writer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_compile_features(test_csv_writer PUBLIC cxx_std_17)

  # --- test_retention_manager ---
  ament_add_gtest(test_retention_manager
    test/test_retention_manager.cpp
    src/retention_manager.cpp)
  target_include_directories(test_retention_manager PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_compile_features(test_retention_manager PUBLIC cxx_std_17)

  # --- test_topic_logger_node (full integration, needs rclcpp) ---
  # Pass the path to the test YAML so the test binary can locate it.
  set(TEST_CONFIG_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/logger_test.yaml")

  ament_add_gtest(test_topic_logger_node
    test/test_topic_logger_node.cpp
    src/topic_logger_node.cpp
    src/csv_writer.cpp
    src/retention_manager.cpp)
  target_include_directories(test_topic_logger_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_compile_features(test_topic_logger_node PUBLIC cxx_std_17)
  target_compile_definitions(test_topic_logger_node PRIVATE
    TEST_CONFIG_PATH="${TEST_CONFIG_PATH}")
  ament_target_dependencies(test_topic_logger_node
    rclcpp sensor_msgs nav_msgs geometry_msgs)
  target_link_libraries(test_topic_logger_node yaml-cpp)
endif()

ament_package()
