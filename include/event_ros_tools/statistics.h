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

#ifndef EVENT_ROS_TOOLS__STATISTICS_H_
#define EVENT_ROS_TOOLS__STATISTICS_H_

#include <stddef.h>
#include <stdint.h>

#include <limits>
#include <string>

namespace event_ros_tools
{
class Statistics
{
public:
  explicit Statistics(const std::string & loggerName);
  ~Statistics();

  void setPrintInterval(uint64_t dt) { printInterval_ = dt; }
  void update(uint64_t t_start, uint64_t t_end, size_t numEvents, uint64_t seq);

private:
  // ------ variables ----
  std::string loggerName_;
  size_t totalMsgs_{0};
  float maxRate_{0};
  float totalTime_{0};
  size_t totalEvents_{0};
  uint64_t lastPrintTime_{std::numeric_limits<uint64_t>::max()};
  uint64_t printInterval_{1000000000ULL};
  uint64_t lastSequence_{0};
  uint64_t dropped_{0};
};
}  // namespace event_ros_tools
#endif  // EVENT_ROS_TOOLS__STATISTICS_H_
