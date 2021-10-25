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

#include "event_ros_tools/slicer_ros1.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>

#include "event_ros_tools/logging.h"

namespace event_ros_tools
{
Slicer::Slicer(const ros::NodeHandle & nh) : nh_(nh), statistics_("slicer")
{
  if (!initialize()) {
    LOG_ERROR("slicer startup failed!");
    throw std::runtime_error("startup of Slicer node failed!");
  }
}

Slicer::~Slicer() { eventSubscriber_->stop(); }

bool Slicer::initialize()
{
  image_transport::ImageTransport it(nh_);
  imagePub_ = it.advertise("image", 1);
  const double pi = nh_.param<double>("statistics_print_interval", 1.0);
  statistics_.setPrintInterval(static_cast<uint64_t>(pi * 1e9));

  const std::string mode = nh_.param<std::string>("mode", "ignore_polarities_inv");
  try {
    imageUpdater_ = ImageUpdater::make(mode);
  } catch (const std::runtime_error & e) {
    BOMB_OUT("invalid mode: " << mode);
  }
  sliceTime_ = static_cast<uint64_t>((std::abs(nh_.param<double>("slice_time", 0.025) * 1e9)));

  const std::string msgType = nh_.param<std::string>("msg_type", "event_array2");
  statistics_.setPrintInterval(
    static_cast<uint64_t>(std::fabs(nh_.param<double>("statistics_print_interval", 2.0) * 1e9)));
  eventSubscriber_.reset(new EventSubscriber(nh_, this, "events", msgType));
  eventSubscriber_->start();
  return (true);
}

void Slicer::imageSize(uint32_t width, uint32_t height)
{
  imageUpdater_->resetImage(width, height);
}

void Slicer::messageComplete(
  const std_msgs::Header & header, uint64_t endTime, uint64_t seq, size_t numEvents)
{
  uint64_t t_msg = ros::Time(header.stamp).toNSec();

  if (lastTime_ > t_msg) {  // time must have gone backwards, restart
    lastTime_ = t_msg;
  }
  if (numEvents != 0) {
    statistics_.update(t_msg, endTime, numEvents, seq);
  }

  if (imagePub_.getNumSubscribers() != 0) {
    if (t_msg > lastTime_ + sliceTime_) {
      lastTime_ = t_msg;
      const cv::Mat & img = imageUpdater_->getImage();
      if (img.rows != 0 && img.cols != 0) {
        cv::Mat normImg;
        cv::normalize(img, normImg, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::Mat imgU8, eqImg;
        cv::equalizeHist(normImg, eqImg);
        imagePub_.publish(cv_bridge::CvImage(header, "mono8", eqImg).toImageMsg());
        imageUpdater_->resetImage();
      }
    }
  }
}

}  // namespace event_ros_tools
