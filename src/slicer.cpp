// -*-c++-*--------------------------------------------------------------------
// Copyright 2020 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include "event_ros_tools/slicer.h"

#include <cv_bridge/cv_bridge.h>

#include <map>
#include <opencv2/imgproc.hpp>

namespace event_ros_tools
{
Slicer::Slicer(const ros::NodeHandle & nh) : nh_(nh) {}

static std::map<std::string, Slicer::Mode> mode_map({
  {"ignore_polarities_inv", Slicer::Mode::IGNORE_POLARITIES_INV},
});

void Slicer::initialize()
{
  ROS_INFO_STREAM("slicer initialized!");
  int qs = nh_.param<int>("event_queue_size", 1000);
  sub_ = nh_.subscribe("events", qs, &Slicer::eventCallback, this);
  image_transport::ImageTransport it(nh_);
  imagePub_ = it.advertise("image", 1);
  double printInterval;
  nh_.param<double>("statistics_print_interval", printInterval, 1.0);
  statisticsPrintInterval_ = ros::Duration(printInterval);
  sliceTime_ = ros::Duration(nh_.param<double>("slice_time", 0.025));

  std::string mode = nh_.param<std::string>("mode", "ignore_polarities_inv");
  auto mode_it = mode_map.find(mode);
  if (mode_it == mode_map.end()) {
    ROS_WARN_STREAM("ignoring invalid mode: " << mode);
  }
  mode_ = mode_it->second;
}

void Slicer::updateStatistics(const EventArray & msg)
{
  const ros::Time & t_start = msg.events.begin()->ts;
  const ros::Time & t_end = msg.events.rbegin()->ts;
  const float dt = (float)(t_end - t_start).toSec();
  const float dt_inv = dt != 0 ? (1.0 / dt) : 0;
  const float rate = msg.events.size() * dt_inv * 1e-6;
  dropped_ += msg.header.seq - lastSequence_ - 1;
  maxRate_ = std::max(rate, maxRate_);
  totalEvents_ += msg.events.size();
  totalTime_ += dt;
  totalMsgs_++;
  lastSequence_ = msg.header.seq;
  if (t_end > lastPrintTime_ + statisticsPrintInterval_) {
    const float avgRate =
      1e-6 * totalEvents_ * (totalTime_ > 0 ? 1.0 / totalTime_ : 0);
    const float avgSize = totalEvents_ / (float)totalMsgs_;
    ROS_INFO(
      "rcv msg sz: %7.2f ev,  rt avg: %7.3f Mevs, max: %7.3f Mevs, "
      "drop: %3d",
      avgSize, avgRate, maxRate_, dropped_);
    maxRate_ = 0;
    lastPrintTime_ += statisticsPrintInterval_;
    totalEvents_ = 0;
    totalMsgs_ = 0;
    totalTime_ = 0;
    dropped_ = 0;
  }
}

void Slicer::updateImage(const EventArray & msg)
{
  switch (mode_) {
    case IGNORE_POLARITIES: {
      for (const auto & ev : msg.events) {
        image_.at<int32_t>(ev.y, ev.x) += 1;
      }
      break;
    }
    case IGNORE_POLARITIES_INV: {
      for (const auto & ev : msg.events) {
        image_.at<int32_t>(ev.y, ev.x) -= 1;
      }
      break;
    }
    case RESPECT_POLARITIES: {
      for (const auto & ev : msg.events) {
        image_.at<int32_t>(ev.y, ev.x) += ev.polarity * 2 - 1;
      }
      break;
    }
    case RESPECT_POLARITIES_INV: {
      for (const auto & ev : msg.events) {
        image_.at<int32_t>(ev.y, ev.x) -= ev.polarity * 2 - 1;
      }
      break;
    }
    default: {
      ROS_ERROR_STREAM("invalid mode: " << mode_);
      throw std::runtime_error("invalid mode!");
    }
  }
}

void Slicer::publishImage(const std_msgs::Header & header, const ros::Time & t)
{
#ifdef DEBUG_IMAGE
  double min, max;
  cv::minMaxLoc(image_, &min, &max);
  std::cout << "min: " << min << " max: " << max << std::endl;
#endif
  cv::Mat normImg;
  cv::normalize(image_, normImg, 0, 255, cv::NORM_MINMAX, CV_8U);
  cv::Mat imgU8, eqImg;
  cv::equalizeHist(normImg, eqImg);
  imagePub_.publish(cv_bridge::CvImage(header, "mono8", eqImg).toImageMsg());
  resetImage(image_.cols, image_.rows);
}

void Slicer::resetImage(const int width, const int height)
{
  image_ = cv::Mat::zeros(height, width, CV_32SC1);
}

void Slicer::eventCallback(const EventArray & msg)
{
  if (msg.events.empty()) {
    return;
  }
  if (image_.rows == 0 || image_.cols == 0) {
    resetImage(msg.width, msg.height);
    ROS_INFO_STREAM("image has size " << msg.width << " x " << msg.height);
    lastTime_ = msg.header.stamp;
    lastSequence_ = msg.header.seq;
    lastPrintTime_ = msg.header.stamp;
  }
  updateStatistics(msg);
  if (imagePub_.getNumSubscribers() != 0) {
    updateImage(msg);
    const auto & startTime = msg.events.begin()->ts;
    const auto & endTime = msg.events.rbegin()->ts;
    if (endTime > lastTime_ + sliceTime_) {
      publishImage(msg.header, startTime);
      lastTime_ = endTime;
    }
  }
}

}  // namespace event_ros_tools
