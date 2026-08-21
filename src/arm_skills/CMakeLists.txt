cmake_minimum_required(VERSION 3.8)
project(arm_skills)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(arm_interfaces REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(arm_kinematics REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)

add_executable(skill_server
  src/main.cpp
  src/skill_server.cpp
  src/motion.cpp
  src/gripper.cpp
  src/skill_move_to.cpp
  src/skill_pick.cpp
  src/skill_place.cpp)
target_include_directories(skill_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(skill_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  skill_server
  "rclcpp"
  "rclcpp_action"
  "arm_interfaces"
  "moveit_ros_planning_interface"
  "sensor_msgs"
  "control_msgs"
  "arm_kinematics"
)

# 구역 사각형의 네 꼭지점을 짚어 보는 확인용 도구(2026-08-20). 액션 서버가 아니다.
add_executable(trace_zone src/trace_zone.cpp)
target_include_directories(trace_zone PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(trace_zone PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  trace_zone
  "rclcpp"
  "moveit_ros_planning_interface"
  "arm_kinematics"
)

install(TARGETS skill_server trace_zone
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
