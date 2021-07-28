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

#ifndef EVENT_ROS_TOOLS_SLICER_H_
#define EVENT_ROS_TOOLS_SLICER_H_

#include <dvs_msgs/EventArray.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <opencv2/core/core.hpp>

namespace event_ros_tools
{
class Slicer
{
public:
  enum Mode {
    INVALID,
    IGNORE_POLARITIES,
    IGNORE_POLARITIES_INV,
    RESPECT_POLARITIES,
    RESPECT_POLARITIES_INV,
  };
  using EventArray = dvs_msgs::EventArray;
  Slicer(const ros::NodeHandle & nh);
  void initialize();

private:
  void eventCallback(const EventArray & msg);
  void updateImage(const EventArray & msg);
  void updateStatistics(const EventArray & msg);
  void publishImage(const std_msgs::Header & header, const ros::Time & t);
  void resetImage(const int width, const int height);
  // ------- variables
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  cv::Mat image_;
  image_transport::Publisher imagePub_;
  ros::Time lastTime_;
  ros::Duration sliceTime_{0.025};
  Mode mode_{IGNORE_POLARITIES_INV};
  // related to statistics
  ros::Duration statisticsPrintInterval_{1.0};
  float maxRate_{0};
  uint64_t totalEvents_{0};
  float totalTime_{0};
  uint32_t totalMsgs_{0};
  ros::Time lastPrintTime_{0};
  uint32_t lastSequence_{0};
  uint32_t dropped_{0};
};
}  // namespace event_ros_tools
#endif
