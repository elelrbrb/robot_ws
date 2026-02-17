// ════════════════════════════════════════════════════════════
// Day 40 — 차동 구동 오도메트리 계산기 (C++ 구현)
    // Day 39 Python의 1:1 포팅.
    // 모든 수학은 Day 38에서 유도한 것과 동일.
// ════════════════════════════════════════════════════════════

#include "my_robot_odometry_cpp/differential_drive_odometry.hpp"
  // 자기 자신의 헤더를 첫 번째로 include → 헤더가 자립적(self-contained)인지 확인하는 모범 관행

#include <cmath>
  // M_PI, cos, sin, atan2


namespace my_robot_odometry_cpp
{


// ════════════════════════════════════════
// 생성자
// ════════════════════════════════════════
DifferentialDriveOdometry::DifferentialDriveOdometry(
  double wheel_radius,
  double wheel_separation,
  int ticks_per_rev
)
: wheel_radius_(wheel_radius)
, wheel_separation_(wheel_separation)
, ticks_per_rev_(ticks_per_rev)
, meters_per_tick_((2.0 * M_PI * wheel_radius) / static_cast<double>(ticks_per_rev))
  // 초기화 리스트 (Initializer List):
    // C++에서 멤버 변수를 초기화하는 가장 효율적인 방법!
    // 본문(body)에서 대입하면 → 기본 생성 후 대입 = 2단계
    // 초기화 리스트에서 하면 → 생성 시 바로 값 설정 = 1단계

  // static_cast<double>: int → double 명시적 변환
    // C 스타일 캐스트 (double)ticks_per_rev는 위험!
    // → static_cast가 컴파일 타임에 타입 검사를 해줌
{
  // 본문은 비어있음 — 모든 초기화가 리스트에서 완료됨!
  // 인라인 초기화(헤더에서 x_{0.0})와 초기화 리스트가 합쳐져
  // 모든 멤버 변수가 확정적으로 초기화된 상태!
}



// ════════════════════════════════════════
// 핵심: update_from_radians
// ════════════════════════════════════════
std::tuple<double, double, double>
DifferentialDriveOdometry::update_from_radians(
  double delta_rad_left,
  double delta_rad_right,
  double dt
)
{
  // ---- Step 1: 라디안 → 이동 거리 ----
  const double dl = delta_rad_left * wheel_radius_;
    // const: 이 값은 한 번 계산된 후 변경되지 않음!
        // → 실수로 dl을 수정하는 버그 방지
        // → 컴파일러 최적화 힌트 (레지스터에 유지 가능)
  const double dr = delta_rad_right * wheel_radius_;

  // ---- Step 2: 중심 이동 + 방향 변화 ----
  const double dc = (dr + dl) / 2.0;
  const double dtheta = (dr - dl) / wheel_separation_;

  // ---- Step 3: Runge-Kutta 2차 적분 ----
  const double theta_mid = theta_ + dtheta / 2.0;
    // 호의 중간 방향을 사용 → Euler 대비 10배 정확
  x_ += dc * std::cos(theta_mid);
  y_ += dc * std::sin(theta_mid);
  theta_ += dtheta;

  // ---- 각도 래핑 ----
  theta_ = normalize_angle(theta_);

  // ---- Step 4: 속도 계산 ----
  if (dt > 1e-6) {
    v_ = dc / dt;
    omega_ = dtheta / dt;
  } else {
    v_ = 0.0;
    omega_ = 0.0;
  }

  return {x_, y_, theta_};
    // C++17 Structured Return:
    // std::make_tuple(x_, y_, theta_) 의 축약형!
}



// ════════════════════════════════════════
// update_from_ticks (실제 하드웨어용)
// ════════════════════════════════════════
std::tuple<double, double, double>
DifferentialDriveOdometry::update_from_ticks(
  int delta_ticks_left,
  int delta_ticks_right,
  double dt
)
{
  // 틱 → 라디안 변환 후 위임
  const double delta_rad_left =
    static_cast<double>(delta_ticks_left) * (2.0 * M_PI / static_cast<double>(ticks_per_rev_));
    // Δticks / ticks_per_rev × 2π = Δrad
  const double delta_rad_right =
    static_cast<double>(delta_ticks_right) * (2.0 * M_PI / static_cast<double>(ticks_per_rev_));

  return update_from_radians(delta_rad_left, delta_rad_right, dt);
    // 코드 중복 방지! 핵심 로직은 한 곳에만!
    // DRY 원칙 (Don't Repeat Yourself)
}



// ════════════════════════════════════════
// Getter
// ════════════════════════════════════════
std::tuple<double, double, double>
DifferentialDriveOdometry::get_pose() const
{
  return {x_, y_, theta_};
}

std::tuple<double, double>
DifferentialDriveOdometry::get_velocity() const
{
  return {v_, omega_};
}



// ════════════════════════════════════════
// reset
// ════════════════════════════════════════
void DifferentialDriveOdometry::reset()
{
  x_ = 0.0;
  y_ = 0.0;
  theta_ = 0.0;
  v_ = 0.0;
  omega_ = 0.0;
}


// ════════════════════════════════════════
// normalize_angle: θ를 [-π, +π]로 래핑
// ════════════════════════════════════════
double DifferentialDriveOdometry::normalize_angle(double angle)
{
  // Python의 atan2(sin(θ), cos(θ))와 동일
  return std::atan2(std::sin(angle), std::cos(angle));
    // 왜 이 방법이 작동하는가? (Day 39에서 설명)
    // sin과 cos은 2π 주기 → 어떤 큰 각도든 [-π, +π] 범위로 매핑
    // atan2는 4사분면을 구분 → 정확한 부호 보장
}




void DifferentialDriveOdometry::set_wheel_separation(double ws)
{
  // 멤버 변수 wheel_separation_을 새로운 값으로 갱신
  wheel_separation_ = ws;
  // 필요하다면 meters_per_tick_도 다시 계산 (바퀴 반지름과 엔코더 해상도에 따라 한 틱당 이동 거리를 다시 계산)
  meters_per_tick_ = (2.0 * M_PI * wheel_radius_) / static_cast<double>(ticks_per_rev_);
}



} 