# event_ros_tools

This repository holds tools for working with event based cameras under
ROS.

## Supported platforms

Currently only tested under ROS Noetic on Ubuntu 20.04.


## How to build
Create a workspace (``event_ros_tools_ws``), clone this repo, and use ``wstool``
to pull in the remaining dependencies:

```
mkdir -p ~/event_ros_tools_ws/src
cd ~/event_ros_tools_ws
git clone https://github.com/ZiyunClaudeWang/EventRosTools.git src/event_ros_tools
wstool init src src/event_ros_tools/event_ros_tools.rosinstall
# to update an existing space:
# wstool merge -t src src/event_ros_tools/event_ros_tools.rosinstall
# wstool update -t src
```

Now configure and build:

```
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo  # (optionally add -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
catkin build
```

## Tools

### Slicer

This tool takes as input events and produces image frames in fixed
time intervals.

How to use:
```
roslaunch event_ros_tools slicer_node.launch
```
Parameters:

- ``slice_time`` time slice of events [seconds] to aggregate into image.
- ``event_queue_size`` ROS receive event queue message size.
- ``mode`` (currently must be "ignore_polarities_inv")
- ``statistics_print_interval`` time in seconds between statistics
  printing


Topics:
- ``events`` (subscribes) camera events
- ``image`` (published) frame based image



### Combined driver and recording

Run the metavision driver and rosbag record in a single nodelet! This
will reduce the likelihood of dropped messages and drops CPU
consumption. In an experiment on an AMD 7480h Ryzen with a SilkyEvCam
this dropped the CPU load from separate driver (135% load) and rosbag (110% load) to a
combined 192% load.

How to use:
```
roslaunch event_ros_tools recording_driver.launch bag:=`pwd`/test.bag
```
Then start/stop recording like this:
```
rosrun event_ros_tools start_recording.py
```
And stop recording:
```
rosrun event_ros_tools stop_recording.py
```


## License

This software is issued under the Apache License Version 2.0.
