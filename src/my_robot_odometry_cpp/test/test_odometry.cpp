// ════════════════════════════════════════════════════════════
// Day 40 — 오도메트리 단위 테스트 (Google Test)
  // 테스트 원칙:
    // 1. 각 테스트는 독립적 (순서 무관)
    // 2. Given-When-Then 패턴
    // 3. 허용 오차 = 부동소수점 정밀도 고려
// ════════════════════════════════════════════════════════════

#include <gtest/gtest.h>
  // Google Test 프레임워크
#include <cmath>
#include "my_robot_odometry_cpp/differential_drive_odometry.hpp"

using my_robot_odometry_cpp::DifferentialDriveOdometry;


// ──────────────────────────────────────
// Test 1: 직진 — 두 바퀴 동일 회전
// ──────────────────────────────────────
TEST(OdometryTest, StraightForward)
{
  // Given: 기본 파라미터 로봇
  DifferentialDriveOdometry odom(0.0325, 0.345, 2244);

  // When: 양쪽 바퀴가 동일하게 1 rad 회전
  const double delta_rad = 1.0;  // 1 rad
  const double dt = 0.02;        // 50Hz → 20ms
  auto [x, y, theta] = odom.update_from_radians(delta_rad, delta_rad, dt);

  // Then: 이동 거리 = 1.0 × 0.0325 = 0.0325 m
  EXPECT_NEAR(x, 0.0325, 1e-6);
    // EXPECT_NEAR(actual, expected, tolerance)
    // |actual - expected| < tolerance → PASS
    // 1e-6: double 정밀도 고려 (15자리 중 6자리까지 일치 요구)
  EXPECT_NEAR(y, 0.0, 1e-6);
    // 직진 → Y 변위 = 0
  EXPECT_NEAR(theta, 0.0, 1e-6);
    // 직진 → 방향 변화 없음
}


// ──────────────────────────────────────
// Test 2: 제자리 회전 — 좌우 반대 방향
// ──────────────────────────────────────
TEST(OdometryTest, SpinInPlace)
{
  DifferentialDriveOdometry odom(0.0325, 0.345, 2244);

  // When: 좌=-1 rad, 우=+1 rad
  auto [x, y, theta] = odom.update_from_radians(-1.0, 1.0, 0.02);

  // Then: 중심 이동 = 0 (좌+우 = 0)
  EXPECT_NEAR(x, 0.0, 1e-6);
  EXPECT_NEAR(y, 0.0, 1e-6);

  // 방향 변화:
    // dtheta = (dr - dl) / L
    // dl = -1.0 × 0.0325 = -0.0325
    // dr = +1.0 × 0.0325 = +0.0325
    // dtheta = (0.0325 - (-0.0325)) / 0.345 = 0.065 / 0.345 ≈ 0.18841
  const double expected_dtheta = (2.0 * 0.0325) / 0.345;
  EXPECT_NEAR(theta, expected_dtheta, 1e-6);
}


// ──────────────────────────────────────
// Test 3: 누적 직진 — 1000 스텝
// ──────────────────────────────────────
TEST(OdometryTest, AccumulatedStraight)
{
  DifferentialDriveOdometry odom(0.0325, 0.345, 2244);

  const double delta_rad = 0.1;  // 작은 회전
  const double dt = 0.02;
  const int steps = 1000;

  for (int i = 0; i < steps; ++i) {
    odom.update_from_radians(delta_rad, delta_rad, dt);
  }

  auto [x, y, theta] = odom.get_pose();

  // 총 이동 거리 = 1000 × 0.1 × 0.0325 = 3.25 m
  EXPECT_NEAR(x, 3.25, 1e-4);
    // 1e-4: 1000번 누적 → 부동소수점 오차 약간 허용
  EXPECT_NEAR(y, 0.0, 1e-4);
  EXPECT_NEAR(theta, 0.0, 1e-6);
}


// ──────────────────────────────────────
// Test 4: 원형 주행 — 복귀 오차
// ──────────────────────────────────────
TEST(OdometryTest, CircularPath)
{
  DifferentialDriveOdometry odom(0.0325, 0.345, 2244);

  // 반지름 1m 원형 주행: v_L < v_R
  // v = 0.3 m/s, R = 1.0 m → ω = v/R = 0.3 rad/s
  // v_L = v - ω×L/2 = 0.3 - 0.3×0.1725 = 0.24825 m/s
  // v_R = v + ω×L/2 = 0.3 + 0.3×0.1725 = 0.35175 m/s
  // ω_L = v_L / r = 0.24825 / 0.0325 = 7.638 rad/s
  // ω_R = v_R / r = 0.35175 / 0.0325 = 10.823 rad/s

  const double dt = 0.001;  // 1kHz (고정밀)
  const double omega_left = 7.638;
  const double omega_right = 10.823;
  const double circumference = 2.0 * M_PI * 1.0;  // 2π m
  const double total_time = circumference / 0.3;   // ≈ 20.94 s

  const int steps = static_cast<int>(total_time / dt);

  for (int i = 0; i < steps; ++i) {
    odom.update_from_radians(omega_left * dt, omega_right * dt, dt);
  }

  auto [x, y, theta] = odom.get_pose();

  // 1바퀴 후 원점 복귀
  // RK2 적분이므로 오차가 작아야 함!
  EXPECT_NEAR(x, 0.0, 0.05);
    // 0.05m = 5cm 허용
    // RK2 + 1kHz → 실제로 ~1cm 이내 예상
  EXPECT_NEAR(y, 0.0, 0.05);
}


// ──────────────────────────────────────
// Test 5: 각도 래핑 — θ가 π를 넘을 때
// ──────────────────────────────────────
TEST(OdometryTest, AngleWrapping)
{
  DifferentialDriveOdometry odom(0.0325, 0.345, 2244);

  // 많이 회전시켜서 θ > π가 되게 함
  const double dt = 0.01;
  for (int i = 0; i < 2000; ++i) {
    // 좌=-0.5, 우=+0.5 → 계속 반시계 회전
    odom.update_from_radians(-0.5, 0.5, dt);
  }

  auto [x, y, theta] = odom.get_pose();

  // θ가 [-π, +π] 범위 안에 있어야!
  EXPECT_GE(theta, -M_PI);
    // GE = Greater or Equal
  EXPECT_LE(theta, M_PI);
    // LE = Less or Equal
}


// ──────────────────────────────────────
// Test 6: Tick 기반 업데이트
// ──────────────────────────────────────
TEST(OdometryTest, TickBasedUpdate)
{
  DifferentialDriveOdometry odom(0.0325, 0.345, 2244);

  // 100 ticks = 100 × (2π × 0.0325 / 2244) = 0.00910 m
  auto [x, y, theta] = odom.update_from_ticks(100, 100, 0.02);

  const double expected_dist = 100.0 * (2.0 * M_PI * 0.0325) / 2244.0;
  EXPECT_NEAR(x, expected_dist, 1e-6);
  EXPECT_NEAR(y, 0.0, 1e-6);
}


// ──────────────────────────────────────
// Test 7: reset 동작
// ──────────────────────────────────────
TEST(OdometryTest, ResetWorks)
{
  DifferentialDriveOdometry odom(0.0325, 0.345, 2244);

  odom.update_from_radians(1.0, 1.0, 0.02);
  odom.reset();

  auto [x, y, theta] = odom.get_pose();
  EXPECT_DOUBLE_EQ(x, 0.0);
    // EXPECT_DOUBLE_EQ: 완전히 같아야 PASS (부동소수점 ULP 비교)
  EXPECT_DOUBLE_EQ(y, 0.0);
  EXPECT_DOUBLE_EQ(theta, 0.0);
}