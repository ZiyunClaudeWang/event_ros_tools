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

#ifndef EVENT_ROS_TOOLS__SLICER_ROS1_H_
#define EVENT_ROS_TOOLS__SLICER_ROS1_H_

#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <memory>

#include "event_ros_tools/event_processor.h"
#include "event_ros_tools/event_subscriber_ros1.h"
#include "event_ros_tools/image_updater.h"
#include "event_ros_tools/statistics.h"

namespace event_ros_tools
{
class Slicer : public EventProcessor
{
public:
  explicit Slicer(const ros::NodeHandle & nh);
  ~Slicer();

  void messageStart(const std_msgs::Header & header, uint32_t width, uint32_t height) override;
  void messageComplete(
    const std_msgs::Header & header, uint64_t endTime, uint64_t seq, size_t numEvents) override;
  void event(uint64_t t, uint16_t x, uint16_t y, bool p) override
  {
    imageUpdater_->update(t, x, y, p);
  }

  Slicer(const Slicer &) = delete;
  Slicer & operator=(const Slicer &) = delete;

private:
  bool initialize();

  // ------ variables ----
  ros::NodeHandle nh_;
  uint64_t lastTime_{0};
  uint64_t sliceTime_{0};
  Statistics statistics_;
  image_transport::Publisher imagePub_;
  std::shared_ptr<EventSubscriber> eventSubscriber_;
  std::shared_ptr<ImageUpdater> imageUpdater_;
};
}  // namespace event_ros_tools
#endif  // EVENT_ROS_TOOLS__SLICER_ROS1_H_
