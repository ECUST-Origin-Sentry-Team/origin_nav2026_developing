cmake_minimum_required(VERSION 3.8)
project(calculate_attack_pose_if_available)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(referee_msg REQUIRED)  
find_package(rm_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)


add_executable(invincible_checker src/check_invincible.cpp)
ament_target_dependencies(invincible_checker rclcpp referee_msg rm_interfaces)


add_executable(dec_shoot_and_pose src/decide_shoot_and_pose.cpp)
ament_target_dependencies(dec_shoot_and_pose rclcpp rm_interfaces  tf2 tf2_ros tf2_geometry_msgs visualization_msgs nav_msgs nav2_msgs)


include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

install(TARGETS
  invincible_checker
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  dec_shoot_and_pose
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)



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
