cmake_minimum_required(VERSION 3.8)
project(cortexai_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(cortexai_manager_component SHARED
  src/cortexai_manager.cpp
)

target_include_directories(cortexai_manager_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cortexai_manager_component
  rclcpp
  rclcpp_components
)

rclcpp_components_register_nodes(cortexai_manager_component
  "cortexai_bringup::CortexAIManager"
)

install(TARGETS cortexai_manager_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

