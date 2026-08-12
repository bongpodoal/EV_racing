"""인지 결과(ObstacleInfo, ConeArray)를 받아 차량 제어 명령(VehicleCommand)을 결정하는 판단 노드.

우선순위: 라이다 장애물 정지 > 콘 트랙 추종. 콘이 한쪽만 보이면 반대쪽까지의 거리를 추정해서
게이트 중심으로 조향하고, 둘 다 안 보이면(코너 진입 등) 감속 직진한다. 인지 자체가 끊기면
(라이다든 카메라든) 안전 정지한다.
"""
import rclpy
from rclpy.node import Node

from ev_interfaces.msg import ConeArray, ObstacleInfo, VehicleCommand


class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')

        self.declare_parameter('cruise_throttle_percent', 20.0)
        self.declare_parameter('search_throttle_percent', 10.0)  # 콘이 안 보일 때(코너 등) 감속
        self.declare_parameter('stop_distance_m', 1.0)
        self.declare_parameter('perception_timeout_s', 0.5)
        # 콘이 한쪽만 보일 때 반대쪽 콘까지 있다고 가정하는 픽셀 거리 - TODO: 실제 카메라 화각/설치
        # 높이에 맞춰 캘리브레이션 필요 (지금은 640px 폭 프레임 기준 임의값)
        self.declare_parameter('assumed_track_half_width_px', 220.0)
        self.declare_parameter('steer_gain_deg', 30.0)  # 화면 절반 폭만큼 어긋났을 때 낼 조향각
        # TODO: henes_control.ino의 실측 최대 조향각(±25°)을 임시로 참고한 값 - EV_racing
        # 실차(L7SA004B + APMC-FBL04AMK) 최대 조향각으로 교체해야 함
        self.declare_parameter('max_steer_deg', 25.0)

        self.cruise_throttle_percent = self.get_parameter('cruise_throttle_percent').value
        self.search_throttle_percent = self.get_parameter('search_throttle_percent').value
        self.stop_distance_m = self.get_parameter('stop_distance_m').value
        self.perception_timeout_s = self.get_parameter('perception_timeout_s').value
        self.assumed_track_half_width_px = self.get_parameter('assumed_track_half_width_px').value
        self.steer_gain_deg = self.get_parameter('steer_gain_deg').value
        self.max_steer_deg = self.get_parameter('max_steer_deg').value

        self.latest_obstacle = None
        self.latest_obstacle_stamp = None
        self.latest_cones = None
        self.latest_cones_stamp = None

        self.obstacle_sub = self.create_subscription(
            ObstacleInfo, '/perception/obstacle', self.on_obstacle, 10)
        self.cone_sub = self.create_subscription(
            ConeArray, '/perception/cones', self.on_cones, 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/vehicle_cmd', 10)

        self.timer = self.create_timer(0.05, self.on_timer)  # 20Hz

    def on_obstacle(self, msg: ObstacleInfo) -> None:
        self.latest_obstacle = msg
        self.latest_obstacle_stamp = self.get_clock().now()

    def on_cones(self, msg: ConeArray) -> None:
        self.latest_cones = msg
        self.latest_cones_stamp = self.get_clock().now()

    def _is_stale(self, stamp) -> bool:
        if stamp is None:
            return True
        return (self.get_clock().now() - stamp).nanoseconds > self.perception_timeout_s * 1e9

    def _compute_steering_deg(self, cones: ConeArray):
        """가장 가까운(면적 최대) YELLOW(왼쪽)/BLUE(오른쪽) 콘 쌍의 중점으로 조향각을 낸다.

        둘 다 보이면 그 중점, 한쪽만 보이면 반대쪽까지 assumed_track_half_width_px만큼
        떨어져 있다고 가정한 게이트 중심, 둘 다 안 보이면 None(조향 판단 불가).

        side_ok가 False인 콘(색상 검출과 화면상 위치가 안 맞는 오검출 의심)은 제외한다 - 예를 들어
        화면 오른쪽에서 잡힌 노란색 콘을 왼쪽 경계로 오인해 반대 방향으로 조향하는 걸 막기 위함.
        """
        yellow = [c for c in cones.cones if c.color == 'YELLOW' and c.side_ok]
        blue = [c for c in cones.cones if c.color == 'BLUE' and c.side_ok]
        nearest_yellow = max(yellow, key=lambda c: c.area, default=None)
        nearest_blue = max(blue, key=lambda c: c.area, default=None)

        half_w = self.assumed_track_half_width_px
        if nearest_yellow is not None and nearest_blue is not None:
            target_cx = (nearest_yellow.cx + nearest_blue.cx) / 2.0
        elif nearest_yellow is not None:
            target_cx = nearest_yellow.cx + half_w
        elif nearest_blue is not None:
            target_cx = nearest_blue.cx - half_w
        else:
            return None

        center = cones.image_width / 2.0
        if center <= 0.0:
            return 0.0
        normalized_offset = (target_cx - center) / center
        steer = normalized_offset * self.steer_gain_deg
        return max(-self.max_steer_deg, min(self.max_steer_deg, steer))

    def _publish_stop(self) -> None:
        cmd = VehicleCommand()
        cmd.enable = False
        cmd.throttle_percent = 0.0
        cmd.steering_deg = 0.0
        cmd.brake_percent = 100.0
        self.cmd_pub.publish(cmd)

    def on_timer(self) -> None:
        if self._is_stale(self.latest_obstacle_stamp):
            self._publish_stop()  # 라이다 인지가 끊기면 무조건 정지 (안전 기본값)
            return

        if self.latest_obstacle.obstacle_detected and \
                self.latest_obstacle.distance_m < self.stop_distance_m:
            self._publish_stop()
            return

        if self._is_stale(self.latest_cones_stamp):
            self._publish_stop()  # 카메라/콘 인지가 끊기면도 정지 (라이다만으로는 트랙 추종 불가)
            return

        steer = self._compute_steering_deg(self.latest_cones)

        cmd = VehicleCommand()
        cmd.enable = True
        cmd.brake_percent = 0.0
        if steer is None:
            # 콘이 안 보임(코너 진입 등) - 감속하고 직진 유지
            cmd.throttle_percent = float(self.search_throttle_percent)
            cmd.steering_deg = 0.0
        else:
            cmd.throttle_percent = float(self.cruise_throttle_percent)
            cmd.steering_deg = float(steer)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
