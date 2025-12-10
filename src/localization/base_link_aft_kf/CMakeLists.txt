cmake_minimum_required(VERSION 3.8)
project(base_link_aft_kf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rm_interfaces REQUIRED)

add_executable(base_link_aft_kf src/kf_node.cpp)

ament_target_dependencies(base_link_aft_kf
  rclcpp rclcpp_components geometry_msgs tf2_ros tf2 rm_interfaces
)
target_include_directories(base_link_aft_kf PRIVATE ${EIGEN3_INCLUDE_DIR})
include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

install(TARGETS
base_link_aft_kf
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

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
