"""VehicleCommand를 받아 시리얼로 아두이노(후륜구동 CAN 브릿지)에 전달하는 노드.

프로토콜(아두이노 쪽 파싱: firmware/EZkontrol_RearDrive_CAN/EZkontrol_RearDrive_CAN.ino 참고):
  "C,<enable:0|1>,<throttle_percent:float>,<steer_deg:float>,<brake_percent:float>\\n"

- steer_deg/brake_percent는 현재 이 아두이노 스케치에서 사용하지 않는다 (후륜구동 CAN 전용).
  조향/브레이크(L7SA 서보드라이브, Modbus RTU)는 별도 인터페이스가 필요하다 (TODO).
- 아두이노 쪽에 SERIAL_CMD_TIMEOUT_MS 안에 새 명령이 없으면 자동으로 정지하는 워치독이 있으므로,
  이 노드가 죽거나 시리얼이 끊겨도 차량은 안전측(정지)으로 fallback 된다.
"""
import rclpy
from rclpy.node import Node
import serial

from ev_interfaces.msg import VehicleCommand


class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        self.serial = serial.Serial(port, baud, timeout=0)
        self.get_logger().info(f'Arduino serial opened: {port} @ {baud}')

        self.cmd_sub = self.create_subscription(
            VehicleCommand, '/vehicle_cmd', self.on_cmd, 10)

    def on_cmd(self, msg: VehicleCommand) -> None:
        line = 'C,{},{:.1f},{:.1f},{:.1f}\n'.format(
            1 if msg.enable else 0,
            msg.throttle_percent,
            msg.steering_deg,
            msg.brake_percent,
        )
        self.serial.write(line.encode('ascii'))


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
