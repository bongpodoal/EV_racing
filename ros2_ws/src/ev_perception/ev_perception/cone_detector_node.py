"""카메라 이미지에서 라바콘(YELLOW=왼쪽 기대, BLUE=오른쪽 기대)을 검출해 발행하는 노드.

검출 로직은 ~/Desktop/EV_formula_camera에서 그대로 포팅한 ev_perception.cone_detection을 쓴다.
이 노드는 카메라를 직접 열지 않고 카메라 드라이버(v4l2_camera 등)가 퍼블리시하는
sensor_msgs/Image를 구독한다 - 원본 스크립트가 setup_camera()/read_frame()만 교체하면 카메라
종류를 바꿀 수 있게 분리해둔 것과 같은 이유로, 카메라 하드웨어가 바뀌어도 이 노드는 그대로 쓴다.
"""
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from ev_interfaces.msg import Cone, ConeArray
from ev_perception import cone_detection


class ConeDetectorNode(Node):
    def __init__(self):
        super().__init__('cone_detector_node')

        # v4l2_camera_node 기본 퍼블리시 토픽이 /image_raw (실측 확인, 2026-08-12) - 그대로 기본값으로 씀
        self.declare_parameter('image_topic', '/image_raw')
        image_topic = self.get_parameter('image_topic').value

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, image_topic, self.on_image, 10)
        self.cone_pub = self.create_publisher(ConeArray, '/perception/cones', 10)

    def on_image(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detections = cone_detection.detect_cones(frame)

        out = ConeArray()
        out.header = msg.header
        out.image_width = frame.shape[1]

        for color, cones in detections.items():
            for (x, y, w, h, area, cx, cy, side, side_ok) in cones:
                cone = Cone()
                cone.color = color
                cone.side = side
                cone.side_ok = side_ok
                cone.cx = float(cx)
                cone.cy = float(cy)
                cone.area = float(area)
                out.cones.append(cone)

        self.cone_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
