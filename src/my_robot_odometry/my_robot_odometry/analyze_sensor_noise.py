#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# analyze_sensor_noise.py
# Day 68 — 센서 노이즈 정량화 (R 결정!)
#
# 정적 테스트 데이터에서 각 센서의 노이즈 분산을 계산.
# → 이 값이 R 행렬의 대각 원소!
# ════════════════════════════════════════════════════════════════

import numpy as np
import math


def analyze_odometry_noise(odom_data: dict) -> dict:
    """
    정적 오도메트리 데이터 분석.

    odom_data: {
        'x': [...], 'y': [...], 'theta': [...],
        'vx': [...], 'omega_z': [...]
    }

    정지 중이므로:
      x, y, theta의 변화 = 적분 노이즈!
      vx, omega_z = 순간 노이즈!
    """
    results = {}

    # 위치: 시간에 따른 드리프트 → 분산 계산은 "변화량"에 대해!
    # 정지 중 x가 서서히 변하면 → 시스템적 오차 (바이어스!)
    # 변동 (진동) → 노이즈!
    x = np.array(odom_data['x'])
    y = np.array(odom_data['y'])
    theta = np.array(odom_data['theta'])

    # 드리프트 제거 (선형 추세 빼기)
    from numpy.polynomial import polynomial as P
    t = np.arange(len(x))

    x_detrend = x - np.polyval(np.polyfit(t, x, 1), t)
    y_detrend = y - np.polyval(np.polyfit(t, y, 1), t)
    theta_detrend = theta - np.polyval(np.polyfit(t, theta, 1), t)

    results['sigma_x'] = np.std(x_detrend)
    results['sigma_y'] = np.std(y_detrend)
    results['sigma_theta'] = np.std(theta_detrend)

    # 속도: 정지 중이므로 전부 노이즈!
    vx = np.array(odom_data['vx'])
    omega_z = np.array(odom_data['omega_z'])

    results['sigma_vx'] = np.std(vx)
    results['sigma_omega_z'] = np.std(omega_z)

    # 분산 (R 행렬용)
    results['R_x'] = results['sigma_x'] ** 2
    results['R_y'] = results['sigma_y'] ** 2
    results['R_theta'] = results['sigma_theta'] ** 2
    results['R_vx'] = results['sigma_vx'] ** 2
    results['R_omega_z'] = results['sigma_omega_z'] ** 2

    return results


def analyze_imu_noise(imu_data: dict) -> dict:
    """
    정적 IMU 데이터 분석.

    imu_data: {
        'yaw': [...],
        'omega_z': [...],
        'ax': [...], 'ay': [...], 'az': [...]
    }
    """
    results = {}

    yaw = np.array(imu_data['yaw'])
    omega_z = np.array(imu_data['omega_z'])
    ax = np.array(imu_data['ax'])

    # Yaw: 드리프트 제거 후 노이즈
    t = np.arange(len(yaw))
    yaw_detrend = yaw - np.polyval(np.polyfit(t, yaw, 1), t)
    results['sigma_yaw'] = np.std(yaw_detrend)
    results['R_yaw'] = results['sigma_yaw'] ** 2

    # 드리프트 자체도 기록 (Q 추정에 필요!)
    yaw_drift_rate = (yaw[-1] - yaw[0]) / (len(yaw) * 0.01)
    # 0.01s = 100Hz
    results['yaw_drift_rate'] = yaw_drift_rate  # rad/s

    # 각속도: 전부 노이즈 + 바이어스
    results['omega_z_mean'] = np.mean(omega_z)  # 바이어스!
    results['sigma_omega_z'] = np.std(omega_z)
    results['R_omega_z'] = results['sigma_omega_z'] ** 2

    # 가속도: 중력 제거 후 노이즈
    # Gazebo: az ≈ 9.81 → ax, ay ≈ 0
    results['sigma_ax'] = np.std(ax)
    results['R_ax'] = results['sigma_ax'] ** 2

    return results


def print_noise_report(odom_r, imu_r):
    """R 행렬 추천값 출력."""

    print('=' * 60)
    print('    SENSOR NOISE ANALYSIS — R Matrix Recommendations')
    print('=' * 60)

    print('\n  [Odometry]')
    print(f'    σ_x     = {odom_r["sigma_x"]:.6f} m')
    print(f'    σ_y     = {odom_r["sigma_y"]:.6f} m')
    print(f'    σ_θ     = {math.degrees(odom_r["sigma_theta"]):.4f}°')
    print(f'    σ_vx    = {odom_r["sigma_vx"]:.6f} m/s')
    print(f'    σ_ωz    = {odom_r["sigma_omega_z"]:.6f} rad/s')

    print(f'\n    R_x     = {odom_r["R_x"]:.2e}')
    print(f'    R_y     = {odom_r["R_y"]:.2e}')
    print(f'    R_θ     = {odom_r["R_theta"]:.2e}')

    print('\n  [IMU]')
    print(f'    σ_yaw   = {math.degrees(imu_r["sigma_yaw"]):.4f}°')
    print(f'    σ_ωz    = {imu_r["sigma_omega_z"]:.6f} rad/s')
    print(f'    σ_ax    = {imu_r["sigma_ax"]:.6f} m/s²')
    print(f'    ω_z bias= {imu_r["omega_z_mean"]:.6f} rad/s')
    print(f'    Yaw drift= {math.degrees(imu_r["yaw_drift_rate"])*60:.2f} °/min')

    print(f'\n    R_yaw   = {imu_r["R_yaw"]:.2e}')
    print(f'    R_ωz    = {imu_r["R_omega_z"]:.2e}')
    print(f'    R_ax    = {imu_r["R_ax"]:.2e}')

    print('\n  [Recommendations for ekf.yaml]')
    # 안전 계수 2~5배 적용
    sf = 3.0
    print(f'    (Safety Factor: {sf}x)')
    print(f'\n    odom covariance (pose):')
    print(f'      x:   {odom_r["R_x"] * sf:.2e}')
    print(f'      y:   {odom_r["R_y"] * sf:.2e}')
    print(f'      yaw: {odom_r["R_theta"] * sf:.2e}')
    print(f'\n    imu covariance:')
    print(f'      yaw:     {imu_r["R_yaw"] * sf:.2e}')
    print(f'      omega_z: {imu_r["R_omega_z"] * sf:.2e}')
    print(f'      ax:      {imu_r["R_ax"] * sf:.2e}')
    print('=' * 60)






# ───────────────────────────────────────────────
# Bag 파일 읽기
# ───────────────────────────────────────────────
import rosbag2_py   
    # ROS2 bag 파일을 읽기 위한 라이브러리.
from rclpy.serialization import deserialize_message
    # bag 안에 저장된 raw 데이터를 실제 ROS 메시지 객체로 변환.
from rosidl_runtime_py.utilities import get_message
import tf_transformations  


def load_bag_data(bag_path="static_test"):
    
    """ bag 파일을 읽어서 오도메트리와 IMU 데이터를 딕셔너리 형태로 반환하는 함수."""
    '''
        SequentialReader: bag 파일을 순차적으로 읽는 객체.
        StorageOptions: bag 파일 경로와 저장 방식(sqlite3) 지정.
        ConverterOptions: 메시지 변환 옵션 (여기서는 기본값).
        reader.open: bag 파일 열기.
    '''
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)


    OdomMsg = get_message("nav_msgs/msg/Odometry")
    ImuMsg = get_message("sensor_msgs/msg/Imu")

    odom_data = {'x': [], 'y': [], 'theta': [], 'vx': [], 'omega_z': []}
    imu_data = {'yaw': [], 'omega_z': [], 'ax': [], 'ay': [], 'az': []}


    while reader.has_next():
        '''
            bag 파일에서 메시지를 하나씩 읽음.
                topic: 메시지가 기록된 토픽 이름 (/odom, /imu/data).
                data: raw 데이터.
                t: 타임스탬프.
        '''
        (topic, data, t) = reader.read_next()


        if topic == "/odom":
            '''
                /odom 메시지를 Odometry 타입으로 변환.
                위치(x, y) 저장.

                orientation(쿼터니언)을 yaw(라디안)으로 변환해서 저장.
                속도(vx, ωz) 저장.
            '''
            msg = deserialize_message(data, OdomMsg)
            odom_data['x'].append(msg.pose.pose.position.x)
            odom_data['y'].append(msg.pose.pose.position.y)
            q = msg.pose.pose.orientation
            _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            odom_data['theta'].append(yaw)
            odom_data['vx'].append(msg.twist.twist.linear.x)
            odom_data['omega_z'].append(msg.twist.twist.angular.z)


        elif topic == "/imu/data":
            '''
                /imu/data 메시지를 Imu 타입으로 변환.
                orientation → yaw 추출.
                
                angular_velocity.z → ωz 저장.
                linear_acceleration.x, y, z → 가속도 저장.
            '''
            msg = deserialize_message(data, ImuMsg)
            q = msg.orientation
            _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            imu_data['yaw'].append(yaw)
            imu_data['omega_z'].append(msg.angular_velocity.z)
            imu_data['ax'].append(msg.linear_acceleration.x)
            imu_data['ay'].append(msg.linear_acceleration.y)
            imu_data['az'].append(msg.linear_acceleration.z)

    return odom_data, imu_data



# ───────────────────────────────────────────────
# 메인 실행
# ───────────────────────────────────────────────
if __name__ == "__main__":
    # Bag 파일에서 데이터 읽기
    odom_data, imu_data = load_bag_data("static_test")

    # 오도메트리/IMU 노이즈 분석
    odom_r = analyze_odometry_noise(odom_data)
    imu_r = analyze_imu_noise(imu_data)

    # 결과 출력
    print_noise_report(odom_r, imu_r)

