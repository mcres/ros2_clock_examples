"""Launch a fake temperature sensor publisher and subscriber using a clock from the past"""

import launch_ros.actions

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Weather Nodes fetch time from the system or from the /clock topic'
    )

    # The get_clock() method gets affected by use_sim_time
    fake_temperature_sensor_publisher = launch_ros.actions.Node(
        package='ros2_clock_examples',
        executable='fake_temperature_sensor_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    fake_temperature_sensor_subscriber = launch_ros.actions.Node(
        package='ros2_clock_examples',
        executable='fake_temperature_sensor_subscriber',
        output='screen'
    )

    past_clock_publisher = launch_ros.actions.Node(
        package='ros2_clock_examples',
        executable='past_clock_publisher',
        # avoid /clock publisher to depend on /clock
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        fake_temperature_sensor_publisher,
        fake_temperature_sensor_subscriber,
        past_clock_publisher
    ])
