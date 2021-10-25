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

#include "event_ros_tools/slicer_ros2.h"

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "event_ros_tools/logging.h"

namespace event_ros_tools
{
Slicer::Slicer(const rclcpp::NodeOptions & options) : Node("slicer", options), statistics_("slicer")
{
  if (!initialize()) {
    LOG_ERROR("slicer startup failed!");
    throw std::runtime_error("startup of Slicer node failed!");
  }
}

Slicer::~Slicer() { eventSubscriber_->stop(); }

bool Slicer::initialize()
{
  const size_t imageQueueSize = 4;
  rmw_qos_profile_t qosProf = rmw_qos_profile_default;
  qosProf.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qosProf.depth = imageQueueSize;  // keep at most this number of images
  qosProf.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
  qosProf.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;  // sender does not have to store
  qosProf.deadline.sec = 5;                                 // max expect time between msgs pub
  qosProf.deadline.nsec = 0;
  qosProf.lifespan.sec = 1;  // how long until msg are considered expired
  qosProf.lifespan.nsec = 0;
  qosProf.liveliness_lease_duration.sec = 10;  // time to declare client dead
  qosProf.liveliness_lease_duration.nsec = 0;
  imagePub_ = image_transport::create_publisher(this, "image", qosProf);
  const std::string mode = declare_parameter<std::string>("mode", "ignore_polarities_inv");
  try {
    imageUpdater_ = ImageUpdater::make(mode);
  } catch (const std::runtime_error & e) {
    BOMB_OUT("invalid mode: " << mode);
  }
  sliceTime_ =
    static_cast<uint64_t>((std::abs(declare_parameter<double>("slice_time", 0.025) * 1e9)));

  const std::string msgType = declare_parameter<std::string>("msg_type", "event_array2");
  statistics_.setPrintInterval(static_cast<uint64_t>(
    std::fabs(declare_parameter<double>("statistics_print_interval", 2.0) * 1e9)));
  eventSubscriber_.reset(new EventSubscriber(this, this, "~/events", msgType));
  eventSubscriber_->start();
  return (true);
}

void Slicer::imageSize(uint32_t width, uint32_t height)
{
  imageUpdater_->resetImage(width, height);
}

void Slicer::messageComplete(
  const std_msgs::msg::Header & header, uint64_t endTime, uint64_t seq, size_t numEvents)
{
  uint64_t t_msg = rclcpp::Time(header.stamp).nanoseconds();

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

RCLCPP_COMPONENTS_REGISTER_NODE(event_ros_tools::Slicer)
