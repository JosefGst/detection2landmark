cmake_minimum_required(VERSION 3.8)
project(detection2landmark)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(apriltag_ros REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


add_executable(detection2landmark src/detection2landmark.cpp)
ament_target_dependencies(detection2landmark rclcpp tf2 tf2_ros std_msgs apriltag_msgs cartographer_ros_msgs)

install(TARGETS
  detection2landmark
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
