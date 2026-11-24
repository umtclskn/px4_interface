cmake_minimum_required(VERSION 3.8)
project(px4_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(offboard_hover_node
  src/offboard_hover_main.cpp
  src/offboard_hover_node.cpp
)
target_include_directories(offboard_hover_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(offboard_hover_node
  rclcpp
  px4_msgs
)

install(TARGETS offboard_hover_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Keyboard velocity teleop (Python). Names match ros2 run entry points.
install(PROGRAMS
  scripts/teleop/velocity_control.py
  scripts/teleop/keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)
install(CODE "
  execute_process(COMMAND \"${CMAKE_COMMAND}\" -E create_symlink
    velocity_control.py
    \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/velocity_teleop_bridge\")
  execute_process(COMMAND \"${CMAKE_COMMAND}\" -E create_symlink
    keyboard_teleop.py
    \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/velocity_teleop_keyboard\")
")

ament_package()
