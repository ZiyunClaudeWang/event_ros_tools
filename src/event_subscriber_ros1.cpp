// -*-c++-*---------------------------------------------------------------------------------------
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

#include "event_ros_tools/event_subscriber_ros1.h"

#include <event_array2_msgs/decode.h>

#include <functional>

#include "event_ros_tools/event_processor.h"
#include "event_ros_tools/logging.h"
#include "event_ros_tools/slicer_ros1.h"

using std::placeholders::_1;

namespace event_ros_tools
{
EventSubscriber::EventSubscriber(
  ros::NodeHandle & nh, EventProcessor * eh, const std::string & topic, const std::string & msgType)
: nh_(nh), eventProcessor_(eh), topic_(topic), msgType_(msgType)
{
}
EventSubscriber::~EventSubscriber() { stop(); }

void EventSubscriber::start()
{
  if (running_) {
    BOMB_OUT_NODE("event subscriber already started!");
  }
  running_ = true;
  const int qs = nh_.param<int>("event_queue_size", 1000);
  if (msgType_ == "dvs") {
    sub_ = nh_.subscribe(topic_, qs, &EventSubscriber::callbackEventsDvs, this);
  } else if (msgType_ == "prophesee") {
    sub_ = nh_.subscribe(topic_, qs, &EventSubscriber::callbackEventsPro, this);
  } else if (msgType_ == "event_array2") {
    sub_ = nh_.subscribe(topic_, qs, &EventSubscriber::callbackEvents2, this);
  } else {
    BOMB_OUT_NODE("bad message type: " << msgType_);
  }
}

void EventSubscriber::stop()
{
  sub_.shutdown();
  running_ = false;
}

void EventSubscriber::callbackEventsDvs(DvsEventArrayConstPtr msg)
{
  if (firstCallback_) {
    eventProcessor_->imageSize(msg->width, msg->height);
    firstCallback_ = false;
  }
  for (const auto & e : msg->events) {
    eventProcessor_->event(ros::Time(e.ts).toNSec(), e.x, e.y, e.polarity);
  }
  const uint64_t endTime =
    (msg->events.empty() ? ros::Time(msg->header.stamp) : msg->events.rbegin()->ts).toNSec();
  eventProcessor_->messageComplete(msg->header, endTime, 0, msg->events.size());
}

void EventSubscriber::callbackEventsPro(ProEventArrayConstPtr msg)
{
  if (firstCallback_) {
    eventProcessor_->imageSize(msg->width, msg->height);
    firstCallback_ = false;
  }
  for (const auto & e : msg->events) {
    eventProcessor_->event(ros::Time(e.ts).toNSec(), e.x, e.y, e.polarity);
  }
  const uint64_t endTime =
    (msg->events.empty() ? ros::Time(msg->header.stamp) : msg->events.rbegin()->ts).toNSec();
  eventProcessor_->messageComplete(msg->header, endTime, 0, msg->events.size());
}

void EventSubscriber::callbackEvents2(EventArray2ConstPtr msg)
{
  if (firstCallback_) {
    eventProcessor_->imageSize(msg->width, msg->height);
    firstCallback_ = false;
  }
  const auto time_base = msg->time_base;
  uint64_t endTime = ros::Time(msg->header.stamp).toNSec();
  for (const auto e : msg->p_y_x_t) {
    uint64_t t;
    uint16_t x, y;
    const bool p = event_array2_msgs::decode_t_x_y_p(e, time_base, &t, &x, &y);
    eventProcessor_->event(t, x, y, p);
    endTime = t;
  }
  eventProcessor_->messageComplete(msg->header, endTime, msg->seq, msg->p_y_x_t.size());
}
}  // namespace event_ros_tools
