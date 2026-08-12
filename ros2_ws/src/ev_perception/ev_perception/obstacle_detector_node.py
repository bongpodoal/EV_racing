"""라이다(LaserScan) 기반 전방 장애물 인지 노드.

카메라 라바콘 색상 인식(~/Desktop/EV_formula_camera 참고: 오른쪽=파란색, 왼쪽=노란색 HSV 검출)은
아직 이 패키지에 통합되지 않았다 - 별도 노드로 추가 예정 (TODO).

지금은 라이다로 차량 정면 기준 각도창 안에서 가장 가까운 장애물까지의 거리만 낸다.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from ev_interfaces.msg import ObstacleInfo


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')

        self.declare_parameter('front_half_angle_deg', 60.0)  # rplidar_front_monitor.py와 동일 기본값(전방 120도)
        self.declare_parameter('obstacle_distance_m', 2.0)

        self.front_half_angle_deg = self.get_parameter('front_half_angle_deg').value
        self.obstacle_distance_m = self.get_parameter('obstacle_distance_m').value

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.obstacle_pub = self.create_publisher(ObstacleInfo, '/perception/obstacle', 10)

    def on_scan(self, msg: LaserScan) -> None:
        half_angle = math.radians(self.front_half_angle_deg)
        nearest_range = None
        nearest_angle = 0.0

        for i, r in enumerate(msg.ranges):
            if r <= 0.0 or math.isinf(r) or math.isnan(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            wrapped = math.atan2(math.sin(angle), math.cos(angle))  # -pi~pi로 정규화, 라이다 0도=차량 정면 가정
            if abs(wrapped) > half_angle:
                continue
            if nearest_range is None or r < nearest_range:
                nearest_range = r
                nearest_angle = wrapped

        out = ObstacleInfo()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        if nearest_range is not None and nearest_range <= self.obstacle_distance_m:
            out.obstacle_detected = True
            out.distance_m = float(nearest_range)
            out.angle_rad = float(nearest_angle)
        else:
            out.obstacle_detected = False
            out.distance_m = -1.0
            out.angle_rad = 0.0

        self.obstacle_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
