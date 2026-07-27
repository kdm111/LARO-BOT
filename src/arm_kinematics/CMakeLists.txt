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

# 헤더 검색 경로. export를 위해 두 상황을 나눈다
# BUILD_INTERFACE : 이 패키지를 빌드하는 동안에는 소스의 include/를 본다.
# INSTALL_INTERFACE : 설치된 뒤(다른 패키지가 쓸 때) 설치된 include/를 본다.
# 그냥 include만 쓰면 export된 타깃이 '남의 소스 트리' 경로를 가리켜 깨진다.
target_include_directories(arm_kinematics PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

# orocos_kdl
find_package(orocos_kdl_vendor REQUIRED)

add_executable(bench_ik src/bench_ik.cpp)
target_link_libraries(bench_ik arm_kinematics orocos-kdl)
install(TARGETS bench_ik DESTINATION lib/${PROJECT_NAME})

add_executable(reach_once src/reach_once.cpp)
target_link_libraries(reach_once arm_kinematics)
install(TARGETS reach_once DESTINATION lib/${PROJECT_NAME})
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

# 헤더들을 공유 공간(install/.../include)으로 복사 = 다른 패키지가 #include 할 수 있게
install(DIRECTORY include/ DESTINATION include)

# 컴파일된 라이브러리를 공유 공간으로 복사하고 "export_arm_kinematics"로 묶는다.
install(
  TARGETS arm_kinematics
  EXPORT export_arm_kinematics # 이 설치를 이 이름의 export 세트에 등록
  LIBRARY DESTINATION lib # 동적 라이브러리(.so) -> lib/
  ARCHIVE DESTINATION lib # 정적 라이브러리(.a) -> lib/
  RUNTIME DESTINATION bin # 실행파일 -> bin/ (라이브러리만 내보낼 거지만 관례적으로 명시)
  INCLUDES DESTINATION include) # 이 타깃을 쓰는 쪽에 include 경로를 물려줌.

# export 세트를 ament 카탈로그에 공개
# HAS_LIBRARY_TARGET = "이 안에 링크할 라이브러리 타깃이 있다는 표시"
# 이게 있어야 arm_skills가 find_packages 후 링크 할 수 있다.
ament_export_targets(export_arm_kinematics HAS_LIBRARY_TARGET)

# include 경로 역시 카탈로그에 공개
ament_export_include_directories(include)

ament_package()
