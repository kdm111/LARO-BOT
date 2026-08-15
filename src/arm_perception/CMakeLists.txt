cmake_minimum_required(VERSION 3.8)
project(arm_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
install(
  DIRECTORY urdf launch worlds models
  DESTINATION share/${PROJECT_NAME}
)
# 파이썬 노드는 lib/<패키지>에 실행 파일로 설치해야 ros2 run이 찾는다.
# PROGRAMS(=실행 권한 부여)로 설치한다. DIRECTORY와 목적지가 다르다는 점에 주의.
install(PROGRAMS
  scripts/object_detector.py
  DESTINATION lib/${PROJECT_NAME}
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
