import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # 패키지 경로에서 sahre 경로에서 카메라 xacro를 찾는다.
    gazebo_path = get_package_share_directory('arm_gazebo')
    xacro_file = os.path.join(gazebo_path, 'urdf', 'camera.urdf.xacro')

    # xacro -> URDF 문자열로 펼침
    camera_description = xacro.process_file(xacro_file).toprettyxml(indent=' ')

    # 카메라 모델을 가제보에 spawn 이미 떠 있는 가제보에 붙는다.
    spawn_camera = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', camera_description,
            '-name', 'fixed_camera',
            '-allow_renaming', 'true'
        ]
    )
    # gz 이미지 토픽 -> ROS 토픽 브리지
    # d인자 형식 토픽@rostype방향기호gz타입
    image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ]
    )
    return LaunchDescription([
        spawn_camera,
        image_bridge
    ])
