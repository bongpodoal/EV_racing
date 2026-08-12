"""인지 결과(ObstacleInfo)를 받아 차량 제어 명령(VehicleCommand)을 결정하는 판단 노드.

지금은 "장애물이 정지거리 안에 있으면 정지, 아니면 일정 속도로 직진"하는 최소 로직만 구현한다.
라바콘 트랙 추종 등 실제 경로 계획/조향 로직은 이후 단계에서 이 노드에 추가한다 (TODO).
"""
import rclpy
from rclpy.node import Node

from ev_interfaces.msg import ObstacleInfo, VehicleCommand


class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')

        self.declare_parameter('cruise_throttle_percent', 20.0)
        self.declare_parameter('stop_distance_m', 1.0)
        self.declare_parameter('perception_timeout_s', 0.5)

        self.cruise_throttle_percent = self.get_parameter('cruise_throttle_percent').value
        self.stop_distance_m = self.get_parameter('stop_distance_m').value
        self.perception_timeout_s = self.get_parameter('perception_timeout_s').value

        self.latest_obstacle = None
        self.latest_obstacle_stamp = None

        self.obstacle_sub = self.create_subscription(
            ObstacleInfo, '/perception/obstacle', self.on_obstacle, 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/vehicle_cmd', 10)

        self.timer = self.create_timer(0.05, self.on_timer)  # 20Hz, 아두이노 CAN 폴링(50ms)보다 촘촘히 갱신

    def on_obstacle(self, msg: ObstacleInfo) -> None:
        self.latest_obstacle = msg
        self.latest_obstacle_stamp = self.get_clock().now()

    def on_timer(self) -> None:
        cmd = VehicleCommand()

        perception_stale = (
            self.latest_obstacle_stamp is None
            or (self.get_clock().now() - self.latest_obstacle_stamp).nanoseconds
            > self.perception_timeout_s * 1e9
        )

        if perception_stale:
            # 인지 데이터가 끊기면 무조건 정지 (안전 기본값)
            cmd.enable = False
            cmd.throttle_percent = 0.0
            cmd.brake_percent = 100.0
        elif self.latest_obstacle.obstacle_detected and \
                self.latest_obstacle.distance_m < self.stop_distance_m:
            cmd.enable = False
            cmd.throttle_percent = 0.0
            cmd.brake_percent = 100.0
        else:
            cmd.enable = True
            cmd.throttle_percent = float(self.cruise_throttle_percent)
            cmd.brake_percent = 0.0

        cmd.steering_deg = 0.0  # TODO: 라바콘 트랙 추종 등 실제 조향 로직

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
