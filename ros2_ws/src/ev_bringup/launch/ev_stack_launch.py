"""EV_racing 자율주행 스택 전체 실행: 드라이버(라이다+카메라) -> 인지 -> 판단 -> 아두이노 브릿지.

라이다/카메라 드라이버 파라미터는 실측 확인된 값(2026-08-12)을 기본값으로 넣었다:
- RPLidar A3M1: rplidar_ros의 rplidar_a3_launch.py와 동일 (~/rplidar_front_monitor.py 설정과도 일치)
  /dev/ttyUSB0, 256000bps, frame_id='laser'. 발행 토픽 /scan.
- v4l2_camera_node: 기본값 그대로 사용(/dev/video0, 발행 토픽 /image_raw) - 카메라 미연결 상태에서
  "Failed opening device /dev/video0" 에러만 나는 것까지 실행 확인함. 다른 장치 인덱스를 쓰면
  video_device 파라미터로 launch 인자를 넘기면 된다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB0')
    video_device = LaunchConfiguration('video_device', default='/dev/video0')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value=lidar_port,
                               description='RPLidar A3M1 serial port'),
        DeclareLaunchArgument('video_device', default_value=video_device,
                               description='카메라 V4L2 장치 경로'),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': lidar_port,
                'serial_baudrate': 256000,  # A3M1 기본 속도 (A1/A2는 115200)
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Sensitivity',
            }],
            output='screen',
        ),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            parameters=[{'video_device': video_device}],
            output='screen',
        ),

        Node(
            package='ev_perception',
            executable='obstacle_detector_node',
            name='obstacle_detector_node',
            output='screen',
        ),
        Node(
            package='ev_perception',
            executable='cone_detector_node',
            name='cone_detector_node',
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
