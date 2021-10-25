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

#ifndef EVENT_ROS_TOOLS__EVENT_PROCESSOR_H_
#define EVENT_ROS_TOOLS__EVENT_PROCESSOR_H_

#ifdef USING_ROS_1
#include <std_msgs/Header.h>
#else
#include <std_msgs/msg/header.hpp>
#endif

namespace event_ros_tools
{
class EventProcessor
{
public:
#ifdef USING_ROS_1
  typedef std_msgs::Header Header;
#else
  typedef std_msgs::msg::Header Header;
#endif
  virtual ~EventProcessor() {}
  virtual void messageComplete(
    const Header & header, uint64_t endTime, uint64_t seq, size_t numEvents) = 0;
  virtual void imageSize(uint32_t width, uint32_t height) = 0;
  virtual void event(uint64_t t, uint16_t x, uint16_t y, bool p) = 0;
};
}  // namespace event_ros_tools
#endif  // EVENT_ROS_TOOLS__EVENT_PROCESSOR_H_
