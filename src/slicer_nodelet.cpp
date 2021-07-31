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
#include <nodelet/nodelet.h>
#include <prophesee_event_msgs/EventArray.h>
#include <ros/ros.h>

#include <memory>

#include "event_ros_tools/slicer.h"

namespace event_ros_tools
{
class SlicerNodelet : public nodelet::Nodelet
{
public:
  template <class T>
  std::shared_ptr<Slicer<T>> initSlicer(ros::NodeHandle & pnh)
  {
    auto ptr = std::make_shared<Slicer<T>>(pnh);
    if (!ptr->initialize()) {
      ROS_ERROR("slicer initialization failed, exiting!");
    }
    return (ptr);
  }

  void onInit() override
  {
    nh_ = getPrivateNodeHandle();
    const std::string msg_mode = nh_.param<std::string>("message_type", "dvs");
    ROS_INFO_STREAM("running in message mode: " << msg_mode);
    if (msg_mode == "prophesee") {
      prophSlicer_ = initSlicer<prophesee_event_msgs::EventArray>(nh_);
    } else if (msg_mode == "dvs") {
      dvsSlicer_ = initSlicer<dvs_msgs::EventArray>(nh_);
    } else {
      ROS_ERROR_STREAM("exiting due to invalid message mode: " << msg_mode);
    }
  }

private:
  // ------ variables --------
  std::shared_ptr<event_ros_tools::Slicer<prophesee_event_msgs::EventArray>>
    prophSlicer_;
  std::shared_ptr<event_ros_tools::Slicer<dvs_msgs::EventArray>> dvsSlicer_;
  ros::NodeHandle nh_;
};
}  // namespace event_ros_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(event_ros_tools::SlicerNodelet, nodelet::Nodelet)
