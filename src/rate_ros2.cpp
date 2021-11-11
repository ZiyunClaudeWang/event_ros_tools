// -*-c++-*----------------------------------------------------------------------------------------
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

#include "event_ros_tools/rate_ros2.h"

#include <cv_bridge/cv_bridge.h>

#include <iomanip>
#include <rclcpp_components/register_node_macro.hpp>

#include "event_ros_tools/logging.h"

namespace event_ros_tools
{
Rate::Rate(const rclcpp::NodeOptions & options) : Node("rate", options), statistics_("rate")
{
  eventCount_[0] = eventCount_[1] = 0;
  if (!initialize()) {
    LOG_ERROR("rate startup failed!");
    throw std::runtime_error("startup of Rate node failed!");
  }
}

Rate::~Rate() { eventSubscriber_->stop(); }

bool Rate::initialize()
{
  rateFile_.open(declare_parameter<std::string>("rate_file", "rate.txt"));
  const std::string msgType = declare_parameter<std::string>("message_type", "event_array");
  statistics_.setPrintInterval(static_cast<uint64_t>(
    std::fabs(declare_parameter<double>("statistics_print_interval", 2.0) * 1e9)));
  eventSubscriber_.reset(new EventSubscriber(this, this, "~/events", msgType));
  eventSubscriber_->start();
  ratePub_ = this->create_publisher<msg::Rate>("rate", 10);
  binTime_ = static_cast<uint64_t>(std::abs(declare_parameter<double>("rate_bin_time") * 1e9));
  LOG_INFO("using rate bin time: " << (binTime_ * 1e-9) << "s");
  return (true);
}

void Rate::messageStart(const std_msgs::msg::Header & header, uint32_t width, uint32_t height)
{
  msg_.header = header;
  (void)width;
  (void)height;
  if (startTime_ == 0) {
    startTime_ = rclcpp::Time(header.stamp).nanoseconds();
    lastTime_ = startTime_;
  }
  msgCount_++;
}

void Rate::messageComplete(
  const std_msgs::msg::Header & header, uint64_t endTime, uint64_t seq, size_t numEvents)
{
  uint64_t t_msg = rclcpp::Time(header.stamp).nanoseconds();

  if (numEvents != 0) {
    statistics_.update(t_msg, endTime, numEvents, seq);
  }
}

void Rate::event(uint64_t t, uint16_t x, uint16_t y, bool p)
{
  (void)x;
  (void)y;
  while (t > lastTime_) {
    const uint64_t dt_uint = (t - lastTime_);
    const double dt = dt_uint * 1e-9;
    const double dt_inv = dt == 0 ? 0 : 1.0 / dt;
    msg_.off_rate = eventCount_[0] * dt_inv;
    msg_.on_rate = eventCount_[1] * dt_inv;
    msg_.msg_rate = msgCount_ * dt_inv;
    msg_.time_bin = dt;
    ratePub_->publish(msg_);
    const std::ios::fmtflags oldFlags = rateFile_.flags();
    rateFile_ << std::fixed;
    rateFile_.precision(6);
    rateFile_ << (lastTime_ - startTime_) * 1e-9;
    rateFile_.flags(oldFlags);
    rateFile_ << " " << msg_.off_rate << " " << msg_.on_rate << " " << msg_.msg_rate << std::endl;
    // reset
    eventCount_[0] = eventCount_[1] = 0;
    msgCount_ = 0;
    lastTime_ += binTime_;
  }
  eventCount_[static_cast<int>(p)]++;
}

}  // namespace event_ros_tools

RCLCPP_COMPONENTS_REGISTER_NODE(event_ros_tools::Rate)
