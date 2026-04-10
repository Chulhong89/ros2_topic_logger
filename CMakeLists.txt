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

ament_package()
