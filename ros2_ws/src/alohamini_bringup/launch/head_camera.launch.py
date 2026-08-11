from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device = LaunchConfiguration("device")
    image_topic = LaunchConfiguration("image_topic")
    frame_id = LaunchConfiguration("frame_id")
    fps = LaunchConfiguration("fps")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    jpeg_quality = LaunchConfiguration("jpeg_quality")

    return LaunchDescription(
        [
            DeclareLaunchArgument("device", default_value="/dev/am_camera_forward"),
            DeclareLaunchArgument("image_topic", default_value="/head_camera/image_raw"),
            DeclareLaunchArgument("frame_id", default_value="head_camera"),
            DeclareLaunchArgument("fps", default_value="20.0"),
            DeclareLaunchArgument("width", default_value="640"),
            DeclareLaunchArgument("height", default_value="480"),
            DeclareLaunchArgument("jpeg_quality", default_value="80"),
            Node(
                package="alohamini_bringup",
                executable="head_camera_publisher",
                name="head_camera_publisher",
                output="screen",
                parameters=[
                    {
                        "device": device,
                        "image_topic": image_topic,
                        "frame_id": frame_id,
                        "fps": fps,
                        "width": width,
                        "height": height,
                        "jpeg_quality": jpeg_quality,
                    }
                ],
            ),
        ]
    )
