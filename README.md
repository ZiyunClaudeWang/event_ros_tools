# event_ros_tools

This repository holds tools for working with event based cameras under
ROS and ROS2

## Supported platforms

Currently tested on Ubuntu 20.04 under under ROS Noetic and ROS2 Galactic.


## How to build
Create a workspace (``event_ros_tools_ws``), clone this repo, and use ``wstool``
to pull in the remaining dependencies:

```
mkdir -p ~/event_ros_tools_ws/src
cd ~/event_ros_tools_ws
git clone https://github.com/ZiyunClaudeWang/event_ros_tools.git src/event_ros_tools
wstool init src src/event_ros_tools/event_ros_tools.rosinstall
# to update an existing space:
# wstool merge -t src src/event_ros_tools/event_ros_tools.rosinstall
# wstool update -t src
```

### configure and build on ROS1:

```
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo  # (optionally add -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
catkin build
```

### configure and build on ROS2:

```
cd ~/event_ros_tools_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo  # (optionally add -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
```

## Tools

### Slicer

This tool takes as input events and produces image frames in fixed
time intervals.

Parameters:

- ``slice_time`` time slice of events [seconds] to aggregate into image.
- ``event_queue_size`` ROS receive event queue message size.
- ``message_type`` the message type (dvs, prophesee, event_array)
- ``mode`` (currently must be "ignore_polarities_inv")
- ``statistics_print_interval`` time in seconds between statistics
    printing

Topics:
- ``events`` (subscribes) camera events
- ``image`` (published) frame based image


How to use (ROS1):
```
roslaunch event_ros_tools slicer_node.launch
```

How to use (ROS2):
```
ros2 launch event_ros_tools slicer_node.launch.py
```

## License

This software is issued under the Apache License Version 2.0.
