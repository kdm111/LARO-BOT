cmake_minimum_required(VERSION 3.8)
project(arm_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 이 소스로 라이브러리를 만들어라.
add_library(arm_kinematics src/ik.cpp)
# 이 라이브러리를 빌드할 때, 링크하는 애들도 include/ 폴더를 헤더 검색 경로에 넣어라.(PUBLIC)
target_include_directories(arm_kinematics PUBLIC include)

# orocos_kdl
find_package(orocos_kdl_vendor REQUIRED)

add_executable(bench_ik src/bench_ik.cpp)
target_link_libraries(bench_ik arm_kinematics orocos-kdl)
install(TARGETS bench_ik DESTINATION lib/${PROJECT_NAME})
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
  # ament_add_gtest 매크로를 제공하는 패키지를 찾는다. 이게 없으면 다음 줄 매크로가 정의 안됨으로 죽는다.
  find_package(ament_cmake_gtest REQUIRED)
  # test_ik 테스트 실행 파일을 이 소스로 만들고, gtest 러너의 main을 자동으로 붙이고 ctest에 등록해라. test_ik 파일에 main 함수가 없는 이유
  ament_add_gtest(test_ik test/test_ik.cpp)
  # 테스트 실행파일을 우리 라이브러리와 링크해라.
  target_link_libraries(test_ik arm_kinematics)

  ament_add_gtest(test_kdl_chain test/test_kdl_chain.cpp)
  target_link_libraries(test_kdl_chain arm_kinematics orocos-kdl)
endif()

ament_package()
