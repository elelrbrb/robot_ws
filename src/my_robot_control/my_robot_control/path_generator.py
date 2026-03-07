#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════
# path_generator.py
# Day 76 — 테스트 경로 생성
# ════════════════════════════════════════════════════════════════

import math
from typing import List, Tuple


def generate_straight(length: float = 3.0,
                      resolution: float = 0.05) -> List[Tuple[float, float]]:
    """직선 경로."""
    n = int(length / resolution) + 1
    return [(i * resolution, 0.0) for i in range(n)]



def generate_circle(radius: float = 1.0,
                    resolution: float = 0.05) -> List[Tuple[float, float]]:
    """원형 경로 (반시계)."""
    circumference = 2 * math.pi * radius
    n = int(circumference / resolution) + 1
    
    path = []
    for i in range(n):
        angle = 2 * math.pi * i / (n - 1)
        x = radius * math.sin(angle)
        y = radius * (1 - math.cos(angle))
        # 원점에서 시작, 중심은 (0, radius)!
        path.append((x, y))
    return path



def generate_square(side: float = 2.0,
                    resolution: float = 0.05) -> List[Tuple[float, float]]:
    """사각형 경로."""
    path = []
    n_side = int(side / resolution)

    # 변 1: → (x 증가)
    for i in range(n_side):
        path.append((i * resolution, 0.0))
    # 변 2: ↑ (y 증가)
    for i in range(n_side):
        path.append((side, i * resolution))
    # 변 3: ← (x 감소)
    for i in range(n_side):
        path.append((side - i * resolution, side))
    # 변 4: ↓ (y 감소) 
    for i in range(n_side): 
        path.append((0.0, side - i * resolution))

    return path



def generate_figure8(radius: float = 1.0,
                     resolution: float = 0.05) -> List[Tuple[float, float]]:
    """8자 경로."""
    path = []
    circ = 2 * math.pi * radius
    n = int(circ / resolution) + 1

    # 왼쪽 원 (반시계)
    for i in range(n):
        angle = 2 * math.pi * i / (n - 1)
        x = radius * math.sin(angle)
        y = radius * (1 - math.cos(angle))
        path.append((x, y))

    # 오른쪽 원 (시계) - 연결점부터 시작
    for i in range(1, n):
        angle = -2 * math.pi * i / (n - 1)
        x = radius * math.sin(-angle) 
        # x 반전 안 함! 그냥 시계 방향!
        y = -radius * (1 - math.cos(angle))
        # y 반전! 아래쪽 원!
        path.append((x, y))

    return path



def generate_slalom(amplitude: float = 0.5,
                    wavelength: float = 2.0,
                    length: float = 6.0,
                    resolution: float = 0.05) -> List[Tuple[float, float]]:
    """사인파 슬라럼 경로."""
    n = int(length / resolution) + 1
    path = []
    for i in range(n):
        x = i * resolution
        y = amplitude * math.sin(2 * math.pi * x / wavelength)
        path.append((x, y))
    return path