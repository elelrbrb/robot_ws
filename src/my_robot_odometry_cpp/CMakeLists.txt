#──────────────────────────────────────────────────
# CMake = "빌드 레시피"
  # 이 파일이 하는 일:
    # 1. 컴파일러 설정 (C++17)
    # 2. 의존 패키지 찾기
    # 3. 소스 → 실행 파일 생성
    # 4. 설치 경로 지정
    # 5. 테스트 등록
#──────────────────────────────────────────────────

set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "")


cmake_minimum_required(VERSION 3.8)
  
project(my_robot_odometry_cpp)

#──── 컴파일러 설정 ────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 표준 강제
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

  

#──── 의존 패키지 찾기 ────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

find_package(gazebo REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
  # find_package: 시스템에 설치된 패키지를 찾아서
  # include 경로, 라이브러리 경로를 자동으로 가져온다
  # REQUIRED: 없으면 CMake 에러 → "이 패키지 없이는 빌드 불가!"



#──── 헤더 파일 경로 ────
include_directories(include)
  # include/ 디렉토리를 헤더 검색 경로에 추가
  # → #include "my_robot_odometry_cpp/odometry_node.hpp" 가능



#──── 실행 파일 생성 ────
add_executable(odometry_node
  src/main.cpp
  src/odometry_node.cpp
  src/differential_drive_odometry.cpp
)
  # 3개의 소스 파일을 컴파일하여 'odometry_node' 바이너리 생성
  # 결과: install/my_robot_odometry_cpp/lib/my_robot_odometry_cpp/odometry_node



ament_target_dependencies(odometry_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
)
  # ament_target_dependencies: ROS 2 전용 CMake 매크로
    # 자동으로:
      # 1. include 경로 추가
      # 2. 라이브러리 링크
      # 3. 컴파일 플래그 설정
      # → target_link_libraries + target_include_directories를 한 번에!



# ---- 공유 라이브러리로 빌드 (실행파일이 아님!) ----
add_library(velocity_limiter SHARED
    src/velocity_limiter.cpp
)
  # SHARED: 동적 라이브러리(.so) 생성
  # Gazebo가 dlopen()으로 런타임에 로드!

target_include_directories(velocity_limiter PRIVATE include)

ament_target_dependencies(velocity_limiter
    gazebo_ros
    geometry_msgs
    rclcpp
)



#──── 설치 ────
install(TARGETS odometry_node
  DESTINATION lib/${PROJECT_NAME}
)
  # 바이너리를 install/lib/my_robot_odometry_cpp/에 복사
  # → ros2 run my_robot_odometry_cpp odometry_node 으로 실행 가능!


# Gazebo 플러그인 경로에 설치
install(TARGETS velocity_limiter
    LIBRARY DESTINATION lib
)


install(DIRECTORY include/
  DESTINATION include
)
  # 헤더 파일 설치 (다른 패키지에서 이 코드를 사용할 경우)




  
#──── 테스트 ────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_odometry test/test_odometry.cpp
    src/differential_drive_odometry.cpp
  )
  # GTest 테스트 등록
  # test_odometry: 테스트 실행 파일 이름
  # 소스: test/test_odometry.cpp + 오도메트리 계산 구현
  target_include_directories(test_odometry PRIVATE include)
  ament_target_dependencies(test_odometry rclcpp)
endif()

ament_package()
  # ament_package(): 반드시 마지막에 호출!
  # 패키지 메타데이터를 최종 생성