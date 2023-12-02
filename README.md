# ros2_clock_examples

## Description

Timing and synchronization in ROS 2 can be abstract concepts.

The aim of this package is to ilustrate how the value of the `use_sim_time` parameter can affect the behavior of `Node`s.

The example consists of three `Node`s:
* `clock_publisher`: continually publishes to `/clock` a date and time 30 years back from the present.
* `fake_temperature_sensor_publisher`: publishes a constant temperature with a timestamp populated using the [`get_clock()` method](https://docs.ros2.org/latest/api/rclpy/api/node.html?highlight=get_clock#rclpy.node.Node.get_clock). 
* `fake_temperature_sensor_subscriber`: logs the data received from the temperature sensor message.

## Usage

Use the launch file:
```bash
# get_clock returns the the system time
ros2 launch ros2_clock_examples example.launch.py use_sim_time:=False

# get_clock returns the /clock time
ros2 launch ros2_clock_examples example.launch.py use_sim_time:=True
```

Additionally, you can toggle the behavior using `ros2 param`:
```bash
# Launch the Nodes
ros2 launch ros2_clock_examples example.launch.py

# Publish timestamp from system time
ros2 param set /fake_temperature_sensor_publisher use_sim_time False

# Publish timestamp from /clock time
ros2 param set /fake_temperature_sensor_publisher use_sim_time True
```

## References

* [ROS 2 design: Clock and Time](https://design.ros2.org/articles/clock_and_time.html)
* [Robotics Stack Exchange](https://robotics.stackexchange.com/questions/104775/rclpy-time-time-vs-node-get-clock-now)