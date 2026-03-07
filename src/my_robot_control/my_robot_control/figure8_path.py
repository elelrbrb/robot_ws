#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# figure8_path.py
# Day 81 — Figure-8 경로 생성기 (고급)
#
# 단순 이중 원이 아닌, 교차점에서 부드러운 곡률 전환!
# → 곡률 불연속 방지!
# ════════════════════════════════════════════════════════════════

import math
from typing import List, Tuple


def generate_figure8_smooth(
        radius: float = 1.0,
        resolution: float = 0.03,
        blend_fraction: float = 0.05) -> List[Tuple[float, float]]:
    """
    부드러운 Figure-8 경로 생성.

    교차점 근처에서 곡률이 +κ → 0 → -κ로 부드럽게 전환!
    (blend_fraction으로 전환 구간 조절!)

    Args:
        radius: 각 원의 반지름 [m]
        resolution: 점 간격 [m]
        blend_fraction: 교차점 근처 블렌딩 비율 (0~0.5)

    Returns:
        [(x, y), ...] 경로
    """
    circumference = 2 * math.pi * radius
    n_per_circle = int(circumference / resolution) + 1

    path = []


    # ═══ 상단 원 (반시계) ═══
    # 중심: (0, R), 반지름 R
    # 시작: (0, 0) → 각도 -π/2 (6시 방향!)
    for i in range(n_per_circle):
        angle = -math.pi / 2 + 2 * math.pi * i / n_per_circle
        x = radius * math.cos(angle)
        y = radius + radius * math.sin(angle)
        path.append((x, y))

    # ═══ 하단 원 (시계!) ═══
    # 중심: (0, -R), 반지름 R
    # 시작: (0, 0) → 각도 +π/2 (12시 방향!)
    # 시계 = 각도 감소!
    for i in range(1, n_per_circle + 1):
        angle = math.pi / 2 - 2 * math.pi * i / n_per_circle
        x = radius * math.cos(angle)
        y = -radius + radius * math.sin(angle)
        path.append((x, y))

    # ═══ 교차점 블렌딩 ═══
    if blend_fraction > 0:
        path = _smooth_crossover(path, radius, blend_fraction)

    return path



def _smooth_crossover(path: List[Tuple[float, float]],
                      radius: float,
                      fraction: float) -> List[Tuple[float, float]]:
    """
    교차점 근처를 부드럽게 처리.

    방법: 교차점 근처 점들을 코사인 블렌딩으로 보간!
    → 곡률이 급변하지 않음!
    """
    n = len(path)

    # ✅ 수정 1: blend_count 를 훨씬 작게 (전체의 5% → 약 20 점)
    blend_count = max(5, int(n * fraction))  # 0.15 → 0.05 로 축소!
    result = list(path)

    
    # 교차점 인덱스 찾기 (원점 근처!)
    crossover_indices = []
    for i in range(n):
        x, y = path[i]
        if abs(x) < 0.05 and abs(y) < 0.05:
            crossover_indices.append(i)

    for ci in crossover_indices:
        if ci < blend_count or ci >= n - blend_count:
            continue

        for j in range(-blend_count, blend_count + 1):
            idx = ci + j
            if idx < 0 or idx >= n:
                continue

            # 코사인 블렌딩 가중치 - w: 0→1→0 (양쪽 원 점으로 부드럽게)
            t = (j + blend_count) / (2 * blend_count)
            w = 0.5 * (1 - math.cos(math.pi * t))
            
            # ✅ 수정 6: w 를 factor 에 직접 반영
                # 중앙 (w=1) 에서 최대 10% 축소, 끝 (w=0) 에서 축소 없음
            max_shrink = 0.10  # 최대 10% 안쪽으로
            factor = 1.0 - (w * max_shrink)

            # 블렌딩된 위치 - 원래 점에서 원점 방향으로 약간 당김
            ox, oy = path[idx]
            dist = math.sqrt(ox**2 + oy**2)

            if dist > 0.01:
                # 교차점 근처에서 궤적을 약간 안쪽으로
                factor = 1.0 - 0.1 * math.exp(
                    -(j / blend_count) ** 2
                )
                result[idx] = (ox * factor, oy * factor)

    return result



def compute_path_properties(
        path: List[Tuple[float, float]]) -> dict:
    """
    경로의 물리적 특성 분석.

    Returns:
        총 길이, 최대 곡률, 곡률 변화율 등
    """
    n = len(path)

    # 총 길이
    total_length = 0.0
    for i in range(1, n):
        dx = path[i][0] - path[i-1][0]
        dy = path[i][1] - path[i-1][1]
        total_length += math.sqrt(dx**2 + dy**2)

    # 곡률 프로파일 (Menger Curvature!)
    curvatures = []
    for i in range(1, n - 1):
        ax, ay = path[i-1]
        bx, by = path[i]
        cx, cy = path[i+1]

        ab = math.sqrt((bx-ax)**2 + (by-ay)**2)
        bc = math.sqrt((cx-bx)**2 + (cy-by)**2)
        ca = math.sqrt((ax-cx)**2 + (ay-cy)**2)

        if ab < 1e-9 or bc < 1e-9 or ca < 1e-9:
            curvatures.append(0.0)
            continue

        cross = (bx-ax)*(cy-ay) - (by-ay)*(cx-ax)
        kappa = 2.0 * cross / (ab * bc * ca)
        curvatures.append(kappa)

    curv_arr = [abs(k) for k in curvatures]
    max_curv = max(curv_arr) if curv_arr else 0.0
    mean_curv = sum(curv_arr) / len(curv_arr) if curv_arr else 0.0

    # 곡률 변화율
    curv_rate = []
    ds = total_length / n if n > 0 else 1.0
    for i in range(1, len(curvatures)):
        rate = abs(curvatures[i] - curvatures[i-1]) / ds
        curv_rate.append(rate)

    max_curv_rate = max(curv_rate) if curv_rate else 0.0

    return {
        'total_length': total_length,
        'num_points': n,
        'max_curvature': max_curv,
        'mean_curvature': mean_curv,
        'min_radius': 1.0 / max_curv if max_curv > 0 else float('inf'),
        'max_curvature_rate': max_curv_rate,
        'curvatures': curvatures,
    }