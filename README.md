# ros2_clock_examples

## Description

Timing and synchronization in ROS 2 can be abstract concepts.

The aim of this package is to see the effects of how the `use_sim_time` parameter affects the `Clock`s used by ROS 2 `Node`s.

## Nodes implemented

This package implements the following `Node`s:
* `fake_temperature_sensor_publisher`: publishes a constant temperature with a timestamp populated using the [`get_clock()` method](https://docs.ros2.org/latest/api/rclpy/api/node.html?highlight=get_clock#rclpy.node.Node.get_clock). 
* `fake_temperature_sensor_subscriber`: logs the data received from the temperature sensor message.
* `past_clock_publisher`: continually publishes to `/clock` a date and time 30 years back from the present.
* `slowed_down_clock_publisher`: continually publishes to `/clock` a date and time that moves forward half as quick as the system time.

## Example 1: Go back in time

This example ilustrates how the value of the `use_sim_time` parameter can affect the time returned by `Clock`s to the `Node`s.

Use the launch file:
```bash
# get_clock() returns the the system time
ros2 launch ros2_clock_examples example_1_go_back_in_time.launch.py use_sim_time:=False

# get_clock() returns the /clock time published by /past_clock_publisher
ros2 launch ros2_clock_examples example_1_go_back_in_time.launch.py use_sim_time:=True
```

Additionally, you can toggle the behavior using `ros2 param`:
```bash
# Launch the Nodes
ros2 launch ros2_clock_examples example_1_go_back_in_time.launch.py

# Publish timestamp from system time
ros2 param set /fake_temperature_sensor_publisher use_sim_time False

# Publish timestamp from /clock time
ros2 param set /fake_temperature_sensor_publisher use_sim_time True
```

## Example 2: Slowed down time

This example ilustrates how the value of the `use_sim_time` parameter can affect the timers used by `Node`s.

Even though we tell the `/fake_temperature_sensor_publisher` to publish at 1 Hz, it actually publishes at 2 Hz since it is using the `/clock` from `/slowed_down_clock_publisher`.

```bash
ros2 launch ros2_clock_examples example_2_slowed_down_time.launch.py
```

## References

* [ROS 2 design: Clock and Time](https://design.ros2.org/articles/clock_and_time.html)
* [Robotics Stack Exchange Thread](https://robotics.stackexchange.com/questions/104775/rclpy-time-time-vs-node-get-clock-now)