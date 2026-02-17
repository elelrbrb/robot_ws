// ════════════════════════════════════════════════════════════
// Day 40 — 차동 구동 오도메트리 계산기 (C++ 헤더)
    // Day 39 Python 클래스의 C++ 포팅.
    // ROS 2에 의존하지 않는 순수 계산 클래스!
        // → 단위 테스트가 쉽다 (ROS 초기화 불필요)
        // → 다른 프로젝트에서 재사용 가능
// ════════════════════════════════════════════════════════════

#ifndef MY_ROBOT_ODOMETRY_CPP__DIFFERENTIAL_DRIVE_ODOMETRY_HPP_
#define MY_ROBOT_ODOMETRY_CPP__DIFFERENTIAL_DRIVE_ODOMETRY_HPP_
// Include Guard (인클루드 가드):
    // 동일 헤더가 여러 번 #include 되어도 한 번만 처리!
    // #pragma once도 가능하지만, #ifndef이 더 이식성 좋음


#include <cmath>
  // std::cos, std::sin, std::atan2
  // C++에서 수학 함수는 <cmath>! (<math.h>는 C 스타일)
#include <tuple>
  // std::tuple: 여러 값을 한 번에 반환


namespace my_robot_odometry_cpp
{
// 네임스페이스: 이름 충돌 방지
    // 다른 패키지에도 "DifferentialDriveOdometry"가 있을 수 있다!
    // → my_robot_odometry_cpp::DifferentialDriveOdometry 로 구분


class DifferentialDriveOdometry
{
public:
  // ──── 생성자 ────
    // explicit: 암묵적 변환 방지
    // → DifferentialDriveOdometry obj = 0.0325; 같은 실수 차단!
  explicit DifferentialDriveOdometry(
    double wheel_radius = 0.0325,
    double wheel_separation = 0.345,
    int ticks_per_rev = 2244
  );

  // ──── 핵심 메서드 ────
    /// 엔코더 라디안 변화로 오도메트리 갱신
        /// @param delta_rad_left  좌바퀴 회전 변화 [rad]
        /// @param delta_rad_right 우바퀴 회전 변화 [rad]
        /// @param dt              시간 간격 [s]
        /// @return (x, y, theta) 갱신된 글로벌 위치
  std::tuple<double, double, double> update_from_radians(
    double delta_rad_left,
    double delta_rad_right,
    double dt
  );

    /// 엔코더 틱 변화로 오도메트리 갱신
  std::tuple<double, double, double> update_from_ticks(
    int delta_ticks_left,
    int delta_ticks_right,
    double dt
  );



  // ──── Getter ────
  [[nodiscard]] std::tuple<double, double, double> get_pose() const;
    // [[nodiscard]]: 반환값을 무시하면 경고!
        // → get_pose()를 호출하고 결과를 안 쓰면 → 버그 가능성!
        // const: 이 함수는 내부 상태를 변경하지 않음
  [[nodiscard]] std::tuple<double, double> get_velocity() const;

  /// 상태 초기화
  void reset();



  // 런타임에 wheel_separation 값을 갱신할 수 있도록 setter 추가
    // 초기화되던 값이라, 노드 실행 중에는 바꿀 방법이 없음 -> 이 함수를 추가하면 실행 중에도 값을 갱신
  void set_wheel_separation(double ws);




private:
  // ──── 로봇 물리 상수 (불변) ────
  double wheel_radius_;
  double wheel_separation_;
  int ticks_per_rev_;
  double meters_per_tick_;
    // _ 접미사: 멤버 변수 규약 (Google C++ Style Guide)
    // → 지역 변수와 구분!

  // ──── 상태 (가변) ────
  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};
  double v_{0.0};
  double omega_{0.0};
    // C++11 인라인 초기화:
        // 생성자에서 초기화를 빠뜨려도 0.0으로 시작!
        // 미초기화 변수 버그 방지!

        
  // ──── 내부 유틸리티 ────
  static double normalize_angle(double angle);
    // static: 객체 없이도 호출 가능
    // → 순수 수학 함수이므로 멤버 상태 불필요
};

}  

#endif 