cmake_minimum_required(VERSION 3.5)
project(my_ros2ject)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(my_ros2ject REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FindWall.srv"
  "action/OdomRecord.action"
  DEPENDENCIES geometry_msgs)

# Add include directories
include_directories(
  include
)

# Add library
add_library(odometry_action_client
  src/odometry_action_client.cpp
)

# this line to exports the library
ament_export_targets(odometry_action_client HAS_LIBRARY_TARGET)
# add dependencies to the library
ament_target_dependencies(odometry_action_client rclcpp rclcpp_action std_msgs nav_msgs geometry_msgs sensor_msgs my_ros2ject)
ament_export_dependencies(odometry_action_client rclcpp rclcpp_action std_msgs nav_msgs geometry_msgs sensor_msgs my_ros2ject)


add_executable(follow_wall_node src/follow_wall.cpp)
ament_target_dependencies(follow_wall_node rclcpp std_msgs geometry_msgs sensor_msgs my_ros2ject rclcpp_action)

add_executable(find_wall_node src/find_wall_server.cpp)
ament_target_dependencies(find_wall_node rclcpp std_msgs geometry_msgs sensor_msgs my_ros2ject)

add_executable(odom_record_action_server_node src/odometry_action_server.cpp)
ament_target_dependencies(odom_record_action_server_node rclcpp rclcpp_action std_msgs nav_msgs geometry_msgs sensor_msgs my_ros2ject)

# add_executable(odom_record_action_client_node src/odometry_action_client.cpp)
# ament_target_dependencies(odom_record_action_client_node rclcpp rclcpp_action std_msgs nav_msgs geometry_msgs sensor_msgs my_ros2ject)

# target_link_libraries(follow_wall_node
#     OdometryActionClient
#     ${rclcpp_LIBRARIES}
# )

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS odometry_action_client
  EXPORT odometry_action_client
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
   follow_wall_node
   find_wall_node
   odom_record_action_server_node
   DESTINATION lib/${PROJECT_NAME}
 )

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
