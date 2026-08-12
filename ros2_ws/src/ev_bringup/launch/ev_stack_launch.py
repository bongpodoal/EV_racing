"""EV_racing 자율주행 스택 실행 (인지 -> 판단 -> 아두이노 브릿지 3개 노드).

카메라/라이다 드라이버 노드는 실제 장비에 맞는 드라이버 패키지를 별도로 launch 해야 한다.
예: RPLidar A3M1 -> rplidar_ros (~/rplidar_cpp 의 SDK 대신 ROS2 공식 패키지 사용 권장),
    USB 카메라 -> usb_cam / v4l2_camera. 아직 이 launch에는 포함하지 않았다 (TODO).
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ev_perception',
            executable='obstacle_detector_node',
            name='obstacle_detector_node',
            output='screen',
        ),
        Node(
            package='ev_planning',
            executable='planning_node',
            name='planning_node',
            output='screen',
        ),
        Node(
            package='ev_arduino_bridge',
            executable='arduino_bridge_node',
            name='arduino_bridge_node',
            output='screen',
            parameters=[{'port': '/dev/ttyACM0', 'baud': 115200}],
        ),
    ])
