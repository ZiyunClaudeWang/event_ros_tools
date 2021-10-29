# -----------------------------------------------------------------------------
# Copyright 2021 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    """Create slicer node."""
    image_topic_config = LaunchConfig('image_topic')
    event_topic_config = LaunchConfig('event_topic')
    image_topic = image_topic_config.perform(context)
    event_topic = event_topic_config.perform(context)
    node = Node(
        package='event_ros_tools',
        executable='slicer_node',
        output='screen',
        # prefix=['xterm -e gdb -ex run --args'],
        name='slicer',
        parameters=[
            {'message_type': 'event_array',
             'statistics_print_interval': 2.0,
             'frame_id': '',
             'slice_time': 0.03,
             'send_queue_size': 1500}],
        remappings=[
            ('~/events', event_topic),
            ('~/image', image_topic)
        ])
    return [node]


def generate_launch_description():
    """Create slicer node by calling opaque function."""
    return launch.LaunchDescription([
        LaunchArg('image_topic', default_value=['/event_camera/image'],
                  description='image topic'),
        LaunchArg('event_topic', default_value=['/event_camera/events'],
                  description='event topic'),
        OpaqueFunction(function=launch_setup)
        ])
