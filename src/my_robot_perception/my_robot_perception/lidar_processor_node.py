#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════
# Day 47 — LiDAR 데이터 처리 파이프라인 노드
    # 5-Stage 파이프라인:
        # 1. Validation (무효값 제거)
        # 2. Median Filter (노이즈 제거)
        # 3. Polar → Cartesian (좌표 변환)
        # 4. Clustering (군집화)
        # 5. Feature Extraction (특징 추출)

    # 입력: /scan (sensor_msgs/LaserScan)
    # 출력:
        # /scan_filtered (sensor_msgs/LaserScan) — 필터링된 스캔
        # /obstacles (visualization_msgs/MarkerArray) — 장애물 시각화
# ════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

import math
import copy


class LidarProcessorNode(Node):
    """LiDAR 데이터 처리 파이프라인."""

    def __init__(self):
        super().__init__('lidar_processor_node')

        # ---- 파라미터 ----
        self.declare_parameter('median_kernel_size', 5)
        self.declare_parameter('min_cluster_size', 3)
        self.declare_parameter('cluster_threshold_factor', 2.5)

        self.kernel_size = self.get_parameter('median_kernel_size').value
        self.min_cluster = self.get_parameter('min_cluster_size').value
        self.threshold_factor = self.get_parameter('cluster_threshold_factor').value

        # ---- 구독 ----
        scan_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, scan_qos
        )

        # ---- 발행 ----
        self.filtered_pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/obstacles', 10)

        self.get_logger().info('[LidarProcessor] Pipeline ready.')

    def scan_callback(self, msg: LaserScan):
        """5-Stage 파이프라인 실행."""

        # ════════ Stage 1: Validation ════════
        validated, valid_count = self._validate(
            msg.ranges, msg.range_min, msg.range_max
        )

        if valid_count < 10:
            # 유효 데이터가 10개 미만 → 스캔 불량 → 스킵
            return

        # ════════ Stage 2: Median Filter ════════
        filtered = self._median_filter(validated, self.kernel_size)

        # ════════ 필터링된 LaserScan 발행 ════════
        filtered_msg = copy.deepcopy(msg)
            # 원본 메타데이터(angle, time 등)를 그대로 복사!
        filtered_msg.ranges = filtered
        self.filtered_pub.publish(filtered_msg)

        # ════════ Stage 3 & 4: 좌표 변환 + 군집화 ════════
        clusters = self._cluster(
            filtered,
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max
        )

        # ════════ Stage 5: 특징 추출 ════════
        features = self._extract_features(clusters)

        # ════════ 시각화 마커 발행 ════════
        self._publish_markers(features, clusters, msg.header)

    # ──────────────────────────────────────
    # Stage 1: Validation
    # ──────────────────────────────────────
    @staticmethod
    def _validate(ranges, range_min, range_max):
        validated = []
        valid_count = 0
        for r in ranges:
            if math.isnan(r) or math.isinf(r) or r <= 0.0:
                validated.append(float('nan'))
            elif r < range_min or r > range_max:
                validated.append(float('nan'))
            else:
                validated.append(r)
                valid_count += 1
        return validated, valid_count

    # ──────────────────────────────────────
    # Stage 2: Median Filter
    # ──────────────────────────────────────
    @staticmethod
    def _median_filter(ranges, kernel_size):
        n = len(ranges)
        half = kernel_size // 2
        filtered = list(ranges)

        for i in range(n):
            if math.isnan(ranges[i]):
                continue
            window = []
            for j in range(-half, half + 1):
                idx = (i + j) % n
                r = ranges[idx]
                if not math.isnan(r):
                    window.append(r)
            if len(window) >= 3:
                window.sort()
                filtered[i] = window[len(window) // 2]
        return filtered

    # ──────────────────────────────────────
    # Stage 3 & 4: 좌표 변환 + 군집화
    # ──────────────────────────────────────
    def _cluster(self, ranges, angle_min, angle_increment, range_min, range_max):
        # 유효 점 추출
        valid = []
        for i, r in enumerate(ranges):
            if math.isnan(r):
                continue
            if not (range_min <= r <= range_max):
                continue
            theta = angle_min + i * angle_increment
            valid.append({
                'x': r * math.cos(theta),
                'y': r * math.sin(theta),
                'range': r,
                'angle': theta,
                'index': i
            })

        if len(valid) < self.min_cluster:
            return []

        # Break-Point Detection
        C = 2.0 * math.tan(angle_increment / 2.0)
        clusters = []
        current = [valid[0]]

        for j in range(1, len(valid)):
            prev = valid[j - 1]
            curr = valid[j]

            index_gap = curr['index'] - prev['index']
            if index_gap > 3:
                if len(current) >= self.min_cluster:
                    clusters.append(current)
                current = [curr]
                continue

            avg_range = (prev['range'] + curr['range']) / 2.0
            threshold = self.threshold_factor * C * avg_range
            delta = abs(curr['range'] - prev['range'])

            if delta > threshold:
                if len(current) >= self.min_cluster:
                    clusters.append(current)
                current = [curr]
            else:
                current.append(curr)

        if len(current) >= self.min_cluster:
            clusters.append(current)

        return clusters

    # ──────────────────────────────────────
    # Stage 5: Feature Extraction
    # ──────────────────────────────────────
    @staticmethod
    def _extract_features(clusters):
        features = []
        for cluster in clusters:
            n = len(cluster)
            cx = sum(p['x'] for p in cluster) / n
            cy = sum(p['y'] for p in cluster) / n
            min_dist = min(p['range'] for p in cluster)
            centroid_dist = math.sqrt(cx**2 + cy**2)
            centroid_angle = math.atan2(cy, cx)

            first, last = cluster[0], cluster[-1]
            width = math.sqrt(
                (last['x'] - first['x'])**2 +
                (last['y'] - first['y'])**2
            )

            features.append({
                'centroid': (cx, cy),
                'centroid_distance': centroid_dist,
                'centroid_angle_deg': math.degrees(centroid_angle),
                'min_distance': min_dist,
                'width': width,
                'num_points': n,
            })

        features.sort(key=lambda f: f['min_distance'])
        return features

    # ──────────────────────────────────────
    # 시각화: RViz MarkerArray
    # ──────────────────────────────────────
    def _publish_markers(self, features, clusters, header):
        marker_array = MarkerArray()

        # 이전 마커 삭제
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # 색상 팔레트
        colors = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8),  # 빨강
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8),  # 초록
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8),  # 파랑
            ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8),  # 노랑
            ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8),  # 자홍
            ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.8),  # 시안
        ]

        for idx, (feature, cluster) in enumerate(zip(features, clusters)):
            color = colors[idx % len(colors)]

            # ---- 군집 점들 (LINE_STRIP) ----
            line_marker = Marker()
            line_marker.header = header
            line_marker.ns = 'cluster_lines'
            line_marker.id = idx
            line_marker.type = Marker.LINE_STRIP
                # LINE_STRIP: 점들을 선으로 연결!
                # → 군집의 형태가 보임!
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.02
                # 선 두께 2cm
            line_marker.color = color

            for p in cluster:
                point = Point()
                point.x = p['x']
                point.y = p['y']
                point.z = 0.0
                line_marker.points.append(point)

            marker_array.markers.append(line_marker)

            # ---- 중심점 (SPHERE) ----
            centroid_marker = Marker()
            centroid_marker.header = header
            centroid_marker.ns = 'centroids'
            centroid_marker.id = idx
            centroid_marker.type = Marker.SPHERE
            centroid_marker.action = Marker.ADD
            centroid_marker.pose.position.x = feature['centroid'][0]
            centroid_marker.pose.position.y = feature['centroid'][1]
            centroid_marker.pose.position.z = 0.1
                # 약간 위에 표시 → 점과 겹치지 않게
            centroid_marker.scale.x = 0.08
            centroid_marker.scale.y = 0.08
            centroid_marker.scale.z = 0.08
            centroid_marker.color = color

            marker_array.markers.append(centroid_marker)

            # ---- 텍스트 정보 (TEXT_VIEW_FACING) ----
            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = 'labels'
            text_marker.id = idx
            text_marker.type = Marker.TEXT_VIEW_FACING
                # 항상 카메라를 향하는 텍스트!
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = feature['centroid'][0]
            text_marker.pose.position.y = feature['centroid'][1]
            text_marker.pose.position.z = 0.25
            text_marker.scale.z = 0.08
                # 텍스트 높이 8cm
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = (
                f"#{idx} d={feature['min_distance']:.2f}m "
                f"w={feature['width']:.2f}m"
            )
                # 예: "#0 d=0.82m w=0.31m"
                # → 0번 장애물, 최소 거리 0.82m, 폭 0.31m

            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
