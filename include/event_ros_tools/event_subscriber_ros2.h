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

#ifndef EVENT_ROS_TOOLS__EVENT_SUBSCRIBER_ROS2_H_
#define EVENT_ROS_TOOLS__EVENT_SUBSCRIBER_ROS2_H_

#include <dvs_msgs/msg/event_array.hpp>
#include <event_array2_msgs/msg/event_array2.hpp>
#include <prophesee_event_msgs/msg/event_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include "event_ros_tools/event_processor.h"

namespace event_ros_tools
{
class EventSubscriber
{
public:
  EventSubscriber(
    rclcpp::Node * node, EventProcessor * eh, const std::string & topic,
    const std::string & msgType);
  ~EventSubscriber();
  void start();
  void stop();

private:
  using DvsEventArray = dvs_msgs::msg::EventArray;
  using DvsEventArrayConstPtr = DvsEventArray::ConstSharedPtr;
  using ProEventArray = prophesee_event_msgs::msg::EventArray;
  using ProEventArrayConstPtr = ProEventArray::ConstSharedPtr;
  using EventArray2 = event_array2_msgs::msg::EventArray2;
  using EventArray2ConstPtr = EventArray2::ConstSharedPtr;

  bool initialize();
  void callbackEventsDvs(DvsEventArrayConstPtr events);
  void callbackEventsPro(ProEventArrayConstPtr events);
  void callbackEvents2(EventArray2ConstPtr events);

  // ------ variables
  rclcpp::Node * node_;
  EventProcessor * eventProcessor_;
  rclcpp::Subscription<DvsEventArray>::SharedPtr dvsSub_;
  rclcpp::Subscription<ProEventArray>::SharedPtr proSub_;
  rclcpp::Subscription<EventArray2>::SharedPtr array2Sub_;
  std::string topic_;
  std::string msgType_;
  bool firstCallback_{true};
};
}  // namespace event_ros_tools
#endif  // EVENT_ROS_TOOLS__EVENT_SUBSCRIBER_ROS2_H_
