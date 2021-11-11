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

#ifndef EVENT_ROS_TOOLS__RATE_ROS2_H_
#define EVENT_ROS_TOOLS__RATE_ROS2_H_

#include <event_ros_tools/msg/rate.hpp>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "event_ros_tools/event_processor.h"
#include "event_ros_tools/event_subscriber_ros2.h"
#include "event_ros_tools/statistics.h"

namespace event_ros_tools
{
class Rate : public rclcpp::Node, EventProcessor
{
public:
  explicit Rate(const rclcpp::NodeOptions & options);
  ~Rate();

  void messageStart(const std_msgs::msg::Header & header, uint32_t width, uint32_t height) override;
  void messageComplete(
    const std_msgs::msg::Header & header, uint64_t endTime, uint64_t seq,
    size_t numEvents) override;
  void event(uint64_t t, uint16_t x, uint16_t y, bool p) override;

  Rate(const Rate &) = delete;
  Rate & operator=(const Rate &) = delete;

private:
  bool initialize();
  void updateStatistics(uint64_t t_start, uint64_t t_end, size_t numEvents, uint64_t seq);

  // ------ variables ----
  Statistics statistics_;
  std::shared_ptr<EventSubscriber> eventSubscriber_;
  rclcpp::Publisher<msg::Rate>::SharedPtr ratePub_;
  std::ofstream rateFile_;
  uint64_t lastTime_{0};
  uint64_t startTime_{0};
  uint64_t binTime_{0};
  uint64_t eventCount_[2];
  uint64_t msgCount_{0};
  msg::Rate msg_;
};
}  // namespace event_ros_tools
#endif  // EVENT_ROS_TOOLS__RATE_ROS2_H_
