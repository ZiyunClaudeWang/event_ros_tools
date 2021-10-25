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

#ifndef EVENT_ROS_TOOLS__EVENT_SUBSCRIBER_ROS1_H_
#define EVENT_ROS_TOOLS__EVENT_SUBSCRIBER_ROS1_H_

#include <dvs_msgs/EventArray.h>
#include <event_array2_msgs/EventArray2.h>
#include <prophesee_event_msgs/EventArray.h>
#include <ros/ros.h>

#include <memory>

#include "event_ros_tools/event_processor.h"

namespace event_ros_tools
{
class EventSubscriber
{
public:
  EventSubscriber(
    ros::NodeHandle & nh, EventProcessor * ep, const std::string & topic,
    const std::string & msgType);
  ~EventSubscriber();
  void start();
  void stop();

private:
  using DvsEventArray = dvs_msgs::EventArray;
  using DvsEventArrayConstPtr = DvsEventArray::Ptr;
  using ProEventArray = prophesee_event_msgs::EventArray;
  using ProEventArrayConstPtr = ProEventArray::Ptr;
  using EventArray2 = event_array2_msgs::EventArray2;
  using EventArray2ConstPtr = EventArray2::Ptr;

  bool initialize();
  void callbackEventsDvs(DvsEventArrayConstPtr events);
  void callbackEventsPro(ProEventArrayConstPtr events);
  void callbackEvents2(EventArray2ConstPtr events);

  // ------ variables
  ros::NodeHandle nh_;
  EventProcessor * eventProcessor_;
  ros::Subscriber sub_;
  std::string topic_;
  std::string msgType_;
  bool running_{false};
  bool firstCallback_{true};
};
}  // namespace event_ros_tools
#endif  // EVENT_ROS_TOOLS__EVENT_SUBSCRIBER_ROS1_H_
