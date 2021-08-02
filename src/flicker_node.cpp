// -*-c++-*--------------------------------------------------------------------
// Copyright 2021 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <dvs_msgs/EventArray.h>
#include <prophesee_event_msgs/EventArray.h>
#include <ros/ros.h>

#include "event_ros_tools/flicker.h"

template <class T>
void run_node(ros::NodeHandle & pnh)
{
  event_ros_tools::Flicker<T> node(pnh);
  if (node.initialize()) {
    ros::spin();
  } else {
    ROS_ERROR("initialization failed, exiting!");
  }
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "flicker_node");
  ros::NodeHandle pnh("~");

  try {
    const std::string msg_mode = pnh.param<std::string>("message_type", "dvs");
    ROS_INFO_STREAM("running in message mode: " << msg_mode);
    if (msg_mode == "prophesee") {
      run_node<prophesee_event_msgs::EventArray>(pnh);
    } else if (msg_mode == "dvs") {
      run_node<dvs_msgs::EventArray>(pnh);
    } else {
      ROS_ERROR_STREAM("exiting due to invalid message mode: " << msg_mode);
    }
  } catch (const std::exception & e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
    return (-1);
  }
  return (0);
}
