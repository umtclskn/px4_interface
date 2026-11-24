cmake_minimum_required(VERSION 3.8)
project(px4_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(px4_interface_core
  src/frame_conversions.cpp
  src/path_tracker.cpp
)
target_include_directories(px4_interface_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

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

add_executable(goal_follower_node
  src/goal_follower_main.cpp
  src/goal_follower_node.cpp
)
target_include_directories(goal_follower_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(goal_follower_node px4_interface_core)
ament_target_dependencies(goal_follower_node
  rclcpp
  px4_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  tf2_ros
)

install(TARGETS px4_interface_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS offboard_hover_node goal_follower_node
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
  scripts/test/publish_test_path.py
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame_conversions test/test_frame_conversions.cpp)
  target_link_libraries(test_frame_conversions px4_interface_core)
  ament_add_gtest(test_path_tracker test/test_path_tracker.cpp)
  target_link_libraries(test_path_tracker px4_interface_core)
  ament_add_gtest(test_goal_follower_construct test/test_goal_follower_construct.cpp)
  target_link_libraries(test_goal_follower_construct px4_interface_core)
  ament_target_dependencies(test_goal_follower_construct
    rclcpp
    px4_msgs
    geometry_msgs
    nav_msgs
    std_srvs
    tf2_ros
  )
  target_sources(test_goal_follower_construct PRIVATE src/goal_follower_node.cpp)
endif()

ament_package()
