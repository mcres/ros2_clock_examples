"""Launch a fake temperature sensor publisher and subscriber using a slowed down clock"""

import launch_ros.actions

from launch import LaunchDescription


def generate_launch_description():

    fake_temperature_sensor_publisher = launch_ros.actions.Node(
        package='ros2_clock_examples',
        executable='fake_temperature_sensor_publisher',
        # Always fetch time from slowed down /clock
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    fake_temperature_sensor_subscriber = launch_ros.actions.Node(
        package='ros2_clock_examples',
        executable='fake_temperature_sensor_subscriber',
        output='screen'
    )

    slowed_down_clock_publisher = launch_ros.actions.Node(
        package='ros2_clock_examples',
        executable='slowed_down_clock_publisher',
        # avoid /clock publisher to depend on /clock
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    return LaunchDescription([
        fake_temperature_sensor_publisher,
        fake_temperature_sensor_subscriber,
        slowed_down_clock_publisher
    ])
