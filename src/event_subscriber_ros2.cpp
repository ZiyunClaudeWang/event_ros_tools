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

#include "event_ros_tools/event_subscriber_ros2.h"

#include <event_array2_msgs/decode.h>

#include <functional>

#include "event_ros_tools/logging.h"

using std::placeholders::_1;

namespace event_ros_tools
{
EventSubscriber::EventSubscriber(
  rclcpp::Node * node, EventProcessor * ep, const std::string & topic, const std::string & msgType)
: node_(node), eventProcessor_(ep), topic_(topic), msgType_(msgType)
{
}
EventSubscriber::~EventSubscriber() { stop(); }

void EventSubscriber::start()
{
  if (dvsSub_ || proSub_ || array2Sub_) {
    BOMB_OUT_NODE("event subscriber already started!");
  }
  const size_t EVENT_QUEUE_DEPTH(1000);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(EVENT_QUEUE_DEPTH)).best_effort().durability_volatile();
  if (msgType_ == "dvs") {
    dvsSub_ = node_->create_subscription<DvsEventArray>(
      topic_, qos, std::bind(&EventSubscriber::callbackEventsDvs, this, _1));
  } else if (msgType_ == "prophesee") {
    proSub_ = node_->create_subscription<ProEventArray>(
      topic_, qos, std::bind(&EventSubscriber::callbackEventsPro, this, _1));
  } else if (msgType_ == "event_array2") {
    array2Sub_ = node_->create_subscription<EventArray2>(
      topic_, qos, std::bind(&EventSubscriber::callbackEvents2, this, _1));
  } else {
    BOMB_OUT_NODE("bad message type: " << msgType_);
  }
}

void EventSubscriber::stop()
{
  dvsSub_.reset();
  proSub_.reset();
  array2Sub_.reset();
}

void EventSubscriber::callbackEventsDvs(DvsEventArrayConstPtr msg)
{
  if (firstCallback_) {
    eventProcessor_->imageSize(msg->width, msg->height);
    firstCallback_ = false;
  }
  for (const auto & e : msg->events) {
    eventProcessor_->event(rclcpp::Time(e.ts).nanoseconds(), e.x, e.y, e.polarity);
  }
  const uint64_t endTime =
    rclcpp::Time(msg->events.empty() ? msg->header.stamp : msg->events.rbegin()->ts).nanoseconds();
  eventProcessor_->messageComplete(msg->header, endTime, 0, msg->events.size());
}

void EventSubscriber::callbackEventsPro(ProEventArrayConstPtr msg)
{
  if (firstCallback_) {
    eventProcessor_->imageSize(msg->width, msg->height);
    firstCallback_ = false;
  }
  for (const auto & e : msg->events) {
    eventProcessor_->event(rclcpp::Time(e.ts).nanoseconds(), e.x, e.y, e.polarity);
  }
  const uint64_t endTime =
    rclcpp::Time(msg->events.empty() ? msg->header.stamp : msg->events.rbegin()->ts).nanoseconds();
  eventProcessor_->messageComplete(msg->header, endTime, 0, msg->events.size());
}

void EventSubscriber::callbackEvents2(EventArray2ConstPtr msg)
{
  if (firstCallback_) {
    eventProcessor_->imageSize(msg->width, msg->height);
    firstCallback_ = false;
  }
  const auto time_base = msg->time_base;
  uint64_t endTime = rclcpp::Time(msg->header.stamp).nanoseconds();
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
