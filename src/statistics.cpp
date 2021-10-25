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

#include "event_ros_tools/statistics.h"

#include "event_ros_tools/logging.h"

namespace event_ros_tools
{
Statistics::Statistics(const std::string & name) : loggerName_(name) {}
Statistics::~Statistics() {}

void Statistics::update(uint64_t t_start, uint64_t t_end, size_t numEvents, uint64_t seq)
{
  const float dt = static_cast<float>((t_end - t_start) * 1e-9);
  const float dt_inv = dt != 0 ? (1.0 / dt) : 0;
  const float rate = numEvents * dt_inv * 1e-6;
  maxRate_ = std::max(rate, maxRate_);
  totalEvents_ += numEvents;
  totalTime_ += dt;
  totalMsgs_++;
  // detect drops
  dropped_ += seq > lastSequence_ ? (seq - lastSequence_ - 1) : 0;
  lastSequence_ = seq;
  // first time call or time restarted
  if (t_start < lastPrintTime_) {
    lastPrintTime_ = t_start;
    dropped_ = 0;
  }
  // print when time has elapsed
  if (t_end > lastPrintTime_ + printInterval_) {
    const float avgRate = 1e-6 * totalEvents_ * (totalTime_ > 0 ? 1.0 / totalTime_ : 0);
    const float avgSize = totalEvents_ / static_cast<float>(totalMsgs_);
    LOG_NAMED_INFO_FMT(
      "rcv sz: %7.0f ev, rate: %7.3f max: %7.3f Mevs, "
      "drop: %3zu",
      avgSize, avgRate, maxRate_, dropped_);
    maxRate_ = 0;
    lastPrintTime_ += printInterval_;
    totalEvents_ = 0;
    totalMsgs_ = 0;
    totalTime_ = 0;
    dropped_ = 0;
  }
}

}  // namespace event_ros_tools
