#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════
# rosbag2에서 IMU 데이터를 추출하여 NumPy 배열로 변환
# ════════════════════════════════════════════════════════════

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
import numpy as np

import numpy as np
import matplotlib.pyplot as plt



def extract_imu_from_bag(bag_path, topic='/imu/data'):
    """
    ROS 2 bag에서 IMU 데이터 추출.

    Returns:
        timestamps: np.array [s]
        gyro_xyz: np.array shape (N, 3) [rad/s]
        accel_xyz: np.array shape (N, 3) [m/s²]
        orient_xyzw: np.array shape (N, 4) [quaternion]
    """

    reader = SequentialReader()

    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
        # sqlite3: ROS 2 Humble 기본 저장 형식
        # MCAP: 더 효율적 (Humble 후반부터 지원)
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)

    timestamps = []
    gyro_data = []
    accel_data = []
    orient_data = []


    while reader.has_next():
        topic_name, data, timestamp_ns = reader.read_next()

        if topic_name != topic:
            continue

        msg = deserialize_message(data, Imu)

        # 타임스탬프 (나노초 → 초)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        timestamps.append(t)

        # 각속도
        gyro_data.append([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        # 가속도
        accel_data.append([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # 자세
        orient_data.append([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])


    timestamps = np.array(timestamps)
    timestamps -= timestamps[0]
        # 시작 시간 = 0으로 정규화

    gyro_xyz = np.array(gyro_data)
    accel_xyz = np.array(accel_data)
    orient_xyzw = np.array(orient_data)

    # 샘플링 주기 추정
    dt_array = np.diff(timestamps)
    dt_mean = np.mean(dt_array)
    dt_std = np.std(dt_array)


    print(f'Extracted {len(timestamps)} IMU samples')
    print(f'Duration: {timestamps[-1]:.1f} s')
    print(f'Sampling: dt_mean={dt_mean*1000:.2f} ms, '
          f'dt_std={dt_std*1000:.2f} ms, '
          f'f_s={1.0/dt_mean:.1f} Hz')

    return timestamps, gyro_xyz, accel_xyz, orient_xyzw, dt_mean








# ════════════════════════════════════════════════════════════
# Day 53 — Allan Variance 계산 및 시각화
    # 사용법:
        # 1. 센서를 완전히 정지 상태로 놓기!
        # 2. ros2 bag record /imu/data -d 300 (5분 이상 녹화!)
        # 3. bag에서 데이터 추출 → NumPy 배열
        # 4. 이 스크립트로 Allan Variance 계산!
# ════════════════════════════════════════════════════════════


def compute_allan_variance(data, dt):
    """
    Allan Variance 계산.

    Args:
        data: 1D numpy array — 센서 시계열 데이터
              (예: 자이로 Z축 angular_velocity.z)
        dt: 샘플링 주기 [s] (예: 0.01 for 100Hz)

    Returns:
        taus: 클러스터 시간 배열 [s]
        adevs: Allan Deviation 배열 [data 단위]
    """
    N = len(data)
    print(f"[DEBUG] Total samples: {N}")

    # ---- 클러스터 크기 배열 ----
    # m = 1, 2, 4, 8, 16, ... (2의 거듭제곱으로!)
        # → log 간격으로 균등하게 분포!
        # → 최대 m = N/2 (최소 2개 클러스터 필요!)
    max_m = N // 2  
    m_values = []
    m = 1

    while m <= max_m:
        m_values.append(m)
            # 1.5배씩 증가 → log 스케일에서 균등!
            # 2배보다 촘촘하게 → 더 부드러운 곡선
        m = int(m * 2)
        # 중복 제거 + 정렬   
    m_values = sorted(set(m_values))
    print(f"[DEBUG] m count: {len(m_values)}")    

    taus = []
    avars = []


    for m in m_values:
        tau = m * dt
            # 클러스터 시간

        # 클러스터 수 - 최소 2개 클러스터 필요!
        K = N // m
        if K < 2:
            break
            
        # 클러스터 평균 계산 - data를 m개씩 묶어서 평균
        truncated = data[:K * m]
            # 딱 맞게 자르기 (나머지 버림)
        reshaped = truncated.reshape(K, m)
            # (K, m) 2D 배열로!
        cluster_means = reshaped.mean(axis=1)
            # 각 행(클러스터)의 평균 → K개

        # Allan Variance
        diffs = np.diff(cluster_means)
            # 연속 클러스터 평균의 차이 → K-1개
        avar = 0.5 * np.mean(diffs ** 2)
            # σ²_A(τ) = (1/2) × E[(ω̄_{k+1} - ω̄_k)²]

        taus.append(tau)
        avars.append(avar)

    taus = np.array(taus)
    adevs = np.sqrt(np.array(avars))
        # Allan Deviation = √(Allan Variance)

    print("[DEBUG] Allan finished.")

    return taus, adevs



def extract_noise_params(taus, adevs):
    """
    Allan Deviation 곡선에서 노이즈 파라미터 추출.

    Returns:
        params: dict with:
            'white_noise': σ_w [unit/√s] (ARW/VRW)
            'bias_instability': σ_b [unit]
            'bias_instability_tau': τ at minimum [s]
    """
    # ---- White Noise (기울기 -1/2 피팅) ----
    # ADEV(τ) = σ_w / √τ
        # → log(ADEV) = log(σ_w) - 0.5 × log(τ)
        # → τ=1에서: ADEV(1) = σ_w

    # 짧은 τ 영역에서 피팅 (처음 1/4)
    n_fit = len(taus) // 4
    if n_fit < 3:
        n_fit = 3

    log_taus = np.log10(taus[:n_fit])
    log_adevs = np.log10(adevs[:n_fit])


    # 선형 회귀: log(ADEV) = a × log(τ) + b
    coeffs = np.polyfit(log_taus, log_adevs, 1)
    slope = coeffs[0]
    intercept = coeffs[1]

    # σ_w = ADEV at τ=1 on the fitted line
    # log(ADEV(τ=1)) = a × 0 + b = b
    white_noise = 10 ** intercept


    # ---- Bias Instability ----
    min_idx = np.argmin(adevs)
    bias_instability = adevs[min_idx] * 0.6643
        # 보정 계수: √(2×ln2/π) ≈ 0.6643
    bi_tau = taus[min_idx]

    params = {
        'white_noise': white_noise,
        'white_noise_slope': slope,
        'bias_instability': bias_instability,
        'bias_instability_tau': bi_tau,
        'min_adev': adevs[min_idx],
    }

    return params



def plot_allan_deviation(taus, adevs, params, title='Allan Deviation', unit='rad/s'):
    """
    Allan Deviation 로그-로그 플롯 생성.
    """
    fig, ax = plt.subplots(1, 1, figsize=(10, 7))

    # ---- 데이터 ----
    ax.loglog(taus, adevs, 'b-', linewidth=1.5, label='Allan Deviation')

    # ---- White Noise 기준선 ----
    wn_line = params['white_noise'] / np.sqrt(taus)
    ax.loglog(taus, wn_line, 'r--', linewidth=1,
              label=f'White Noise: σ_w = {params["white_noise"]:.2e} {unit}/√Hz')

    # ---- Bias Instability 표시 ----
    ax.axhline(y=params['min_adev'], color='g', linestyle=':', linewidth=1,
               label=f'Bias Instability: σ_b = {params["bias_instability"]:.2e} {unit}')
    ax.plot(params['bias_instability_tau'], params['min_adev'], 'go', markersize=10)


    # ---- 기울기 참조선 ----
    # -1/2 기울기 (White Noise 영역)
    tau_ref = taus[0]
    adev_ref = adevs[0]
    slope_line = adev_ref * (taus / tau_ref) ** (-0.5)
    ax.loglog(taus, slope_line, 'k:', alpha=0.3, label='slope = -1/2')

    # +1/2 기울기 (Random Walk 영역)
    tau_ref2 = taus[-1]
    adev_ref2 = adevs[-1]
    slope_line2 = adev_ref2 * (taus / tau_ref2) ** (0.5)
    ax.loglog(taus, slope_line2, 'm:', alpha=0.3, label='slope = +1/2')

    ax.set_xlabel('Cluster Time τ (s)', fontsize=12)
    ax.set_ylabel(f'Allan Deviation ({unit})', fontsize=12)
    ax.set_title(title, fontsize=14)
    ax.legend(fontsize=10)
    ax.grid(True, which='both', alpha=0.3)

    plt.tight_layout()
    return fig



def analyze_imu_data(gyro_z, accel_z, dt):
    """
    자이로 Z축과 가속도계 Z축의 Allan Variance 분석.
    """
    print('=' * 60)
    print('       Allan Variance Analysis')
    print('=' * 60)

    # ---- 자이로 Z축 ----
    print('\n--- Gyroscope Z-axis ---')
    taus_g, adevs_g = compute_allan_variance(gyro_z, dt)
    params_g = extract_noise_params(taus_g, adevs_g)
    print(f'  White Noise (ARW): {params_g["white_noise"]:.6e} rad/s/√Hz')
    print(f'  White Noise slope: {params_g["white_noise_slope"]:.3f} (ideal: -0.500)')
    print(f'  Bias Instability:  {params_g["bias_instability"]:.6e} rad/s')
    print(f'  BI tau:            {params_g["bias_instability_tau"]:.1f} s')

    # → EKF covariance 변환
    gyro_cov = params_g['white_noise'] ** 2 * dt
        # σ²_d = σ²_c × Δt (연속 → 이산 변환)
    print(f'\n  → angular_velocity_covariance: {gyro_cov:.2e}')
    print(f'    (이 값을 EKF의 R 행렬에!)')

    # ---- 가속도계 Z축 ----
    print('\n--- Accelerometer Z-axis ---')
    taus_a, adevs_a = compute_allan_variance(accel_z, dt)
    params_a = extract_noise_params(taus_a, adevs_a)
    print(f'  White Noise (VRW): {params_a["white_noise"]:.6e} m/s²/√Hz')
    print(f'  Bias Instability:  {params_a["bias_instability"]:.6e} m/s²')

    accel_cov = params_a['white_noise'] ** 2 * dt
    print(f'\n  → linear_acceleration_covariance: {accel_cov:.2e}')

    # ---- 플롯 ----
    fig1 = plot_allan_deviation(
        taus_g, adevs_g, params_g,
        title='Gyroscope Z-axis Allan Deviation',
        unit='rad/s'
    )

    fig2 = plot_allan_deviation(
        taus_a, adevs_a, params_a,
        title='Accelerometer Z-axis Allan Deviation',
        unit='m/s²'
    )

    return fig1, fig2, params_g, params_a









def main():
    bag_path = 'imu_static_test'  # 필요하면 절대경로로 지정

    timestamps, gyro_xyz, accel_xyz, orient, dt = \
        extract_imu_from_bag(bag_path)

    gyro_z = gyro_xyz[:, 2]
    accel_z = accel_xyz[:, 2]

    fig1, fig2, params_g, params_a = analyze_imu_data(gyro_z, accel_z, dt)

    # 결과 그래프를 파일로 저장
    fig1.savefig("gyro_allan.png")
    fig2.savefig("accel_allan.png")

    print("\n그래프가 'gyro_allan.png', 'accel_allan.png' 파일로 저장되었습니다.")
    print("콘솔에 출력된 White Noise, Bias Instability 값들을 확인하세요.")




if __name__ == '__main__':
    main()





