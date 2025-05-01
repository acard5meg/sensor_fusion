import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sensor_pkg = get_package_share_directory('sensor_fusion')

    sensor_pkg_path = os.path.join(sensor_pkg, 'config', 'navsat_config.yaml')

    navsat_node = Node(
                    package='robot_localization',
                    executable='navsat_transform_node',
                    name='navsat_transform_node',
                    output='screen',
                    parameters=[sensor_pkg_path],
                    remappings=[
                        ("imu/data", "imu/data"),
                        ("gps/fix", "gps/fix"),
                        ("gps/filtered", "gps/filtered"),
                        # (topic_output_from_executable, new_topic_name seen in ros2 topic)
                        ("odometry/gps", "odometry/gps"),
                        ("odometry/filtered", "odometry/global")
                    ],
                )

    ekf_node = Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[sensor_pkg_path],
                remappings=[
                    ("odometry/filtered", "odometry/global")
                ],
            )

    ekf_node_odom = Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_odom",
            output="screen",
            parameters=[sensor_pkg_path],
            remappings=[
                ("odometry/filtered", "odometry/global")
            ],
        )

    return LaunchDescription(
        [ekf_node_odom
            # RegisterEventHandler(
            #     OnProcessStart(
            #         target_action = ekf_node,
            #         on_start=[LogInfo(msg="EKF Node Started!!!"),
            #                             navsat_node]
            #     )
            # )
            
        ]
    )