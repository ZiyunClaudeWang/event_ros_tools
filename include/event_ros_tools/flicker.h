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

#ifndef EVENT_ROS_TOOLS_FLICKER_H_
#define EVENT_ROS_TOOLS_FLICKER_H_

#define DEBUG

#include <cv_bridge/cv_bridge.h>
#include <event_ros_tools_msgs/Rate.h>
#include <image_transport/image_transport.h>
#include <math.h>
#include <ros/ros.h>

#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#ifdef DEBUG
#include <fstream>
#endif

namespace event_ros_tools
{
template <class MsgType>
class Flicker
{
public:
  using Rate = event_ros_tools_msgs::Rate;
  Flicker(const ros::NodeHandle & nh) : nh_(nh)
  {
#ifdef DEBUG
    rateDebugFile_.open("rate_debug_file.txt");
#endif
  }

  bool initialize()
  {
    decayRate_ = 1.0 / nh_.param<double>("averaging_time", 0.5);
    binIntervalSec_ = nh_.param<double>("bin_interval", 1e-3);
    binInterval_ = ros::Duration(binIntervalSec_);
    peakDetectionRatio_ = nh_.param<double>("peak_detection_ratio", 0.25);
    int qs = nh_.param<int>("event_queue_size", 1000);
    sub_ = nh_.subscribe("events", qs, &Flicker::eventCallback, this);
    image_transport::ImageTransport it(nh_);
    imagePub_ = it.advertise("image", 1);
    ratePub_ = nh_.advertise<Rate>("rate", 10);
    double printInterval;
    nh_.param<double>("statistics_print_interval", printInterval, 1.0);
    statisticsPrintInterval_ = ros::Duration(printInterval);
    eventCount_[0] = eventCount_[1] = 0;
    intCount_[0] = intCount_[1] = 0;
    statisticsIntCount_[0] = statisticsIntCount_[1] = 0;
    ROS_INFO_STREAM("flicker initialized!");
    return (true);
  }

private:
  void updateStatistics(const MsgType & msg)
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
      const double integ = statisticsIntCount_[0] / (double)totalEvents_;
      ROS_INFO(
        "rcv msg sz: %4d ev,  rt[Mevs] avg: %7.3f, max: %7.3f, int: %7.3f, "
        "drop: %3d",
        (int)avgSize, avgRate, maxRate_, integ, dropped_);
      maxRate_ = 0;
      lastPrintTime_ += statisticsPrintInterval_;
      totalEvents_ = 0;
      totalMsgs_ = 0;
      totalTime_ = 0;
      statisticsIntCount_[0] = statisticsIntCount_[1] = 0;
      dropped_ = 0;
    }
  }

  void eventCallback(const MsgType & msg)
  {
    if (msg.events.empty()) {
      return;
    }
    if (image_.rows == 0 || image_.cols == 0) {
      resetImage(msg.width, msg.height);
      ROS_INFO_STREAM("image has size " << msg.width << " x " << msg.height);
      lastSequence_ = msg.header.seq;
      lastPrintTime_ = msg.header.stamp;
      lastBinTime_ = msg.header.stamp;
      startTime_ = msg.header.stamp;
    }
    updateStatistics(msg);
    if (imagePub_.getNumSubscribers() != 0) {
      if (updateImage(msg)) {
        publishImage(msg.header);
      }
    }
  }

  void publishImage(const std_msgs::Header & header)
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

  void resetImage(const int width, const int height)
  {
    image_ = cv::Mat::zeros(height, width, CV_32SC1);
  }

  void publishRate(
    const std_msgs::Header & header, double rate, double stddev,
    double offRatio)
  {
    Rate rateMsg;
    rateMsg.header = header;
    rateMsg.rate = rate * 1e-6;
    rateMsg.mean_rate = rateMeanSum_ * 1e-6;
    rateMsg.std_rate = stddev * 1e-6;
    rateMsg.off_ratio = offRatio;
    ratePub_.publish(rateMsg);
  }

  bool updateImage(const MsgType & msg)
  {
    const double decay = exp(-binIntervalSec_ * decayRate_);
    const bool wasIntegratingAtStart = integrateEventsIntoImage_;
    for (const auto & ev : msg.events) {
      const auto t = ev.ts;
      while (t > lastBinTime_ + binInterval_) {
        // t has crossed into new time bin, process old one
        lastBinTime_ += binInterval_;
        const size_t totCount = eventCount_[0];  // count OFF only
        const double rate = totCount / binIntervalSec_;
        const size_t totIntegratedCount = intCount_[0] + intCount_[1];
        const double offRatio =
          intCount_[0] / (double)std::max(totIntegratedCount, 1UL);
        // discount running sums and add new rate
        rateMeanSum_ = rateMeanSum_ * decay + rate * (1 - decay);
        rateCovSum_ = rateCovSum_ * decay + rate * rate * (1 - decay);
        const double stddev = std::sqrt(rateCovSum_);
        const uint32_t DELAY_INTEGRATE_NUM_BINS = 0;  // 5 gives inverted
        const uint32_t INTEGRATE_NUM_BINS = 3;
        if (integrateEventsIntoImage_) {
          // decide if image integration should be stopped
          if (rate < rateMeanSum_ || numBinsIntegrated_ > INTEGRATE_NUM_BINS) {
            if (
              numBinsWhereRateBelowThreshold_++ > 5 ||
              numBinsIntegrated_ > INTEGRATE_NUM_BINS) {
              integrateEventsIntoImage_ = false;
              numBinsWhereRateBelowThreshold_ = 0;
            }
          }
          numBinsIntegrated_++;
        } else {
          // decide if image integration should be started
          if ((rate - rateMeanSum_) > peakDetectionRatio_ * stddev) {
            if (numBinsIntegrated_ == 0) {
              // start integration, but with some delay
              if (numBinsDelayIntegration_++ > DELAY_INTEGRATE_NUM_BINS) {
                integrateEventsIntoImage_ = true;
                numBinsDelayIntegration_ = 0;
              }
            }
          } else {
            numBinsIntegrated_ = 0;
          }
        }
        publishRate(msg.header, rate, stddev, offRatio);
#ifdef DEBUG
        rateDebugFile_ << lastBinTime_ - startTime_ << " " << totCount << " "
                       << rate << " " << rateMeanSum_ << " " << stddev << " "
                       << offRatio << " " << integrateEventsIntoImage_
                       << std::endl;
#endif
        eventCount_[0] = eventCount_[1] = 0;
      }
      eventCount_[ev.polarity]++;
      if (integrateEventsIntoImage_) {
        if (ev.polarity == 0) {  // OFF event
          image_.at<int32_t>(ev.y, ev.x) += 1;
        }
        intCount_[ev.polarity]++;
        statisticsIntCount_[ev.polarity]++;
      }
    }
    if (!integrateEventsIntoImage_) {
      intCount_[0] = intCount_[1] = 0;
    }
    // image is ready to be published if the integration of
    // events has stopped
    return (wasIntegratingAtStart && !integrateEventsIntoImage_);
  }
  // ------- variables
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher ratePub_;
  cv::Mat image_;
  image_transport::Publisher imagePub_;
  // -- related to statistics
  ros::Duration statisticsPrintInterval_{1.0};
  float maxRate_{0};
  uint64_t totalEvents_{0};
  float totalTime_{0};
  uint32_t totalMsgs_{0};
  ros::Time lastPrintTime_{0};
  uint32_t lastSequence_{0};
  uint32_t dropped_{0};
  uint64_t statisticsIntCount_[2];
  // -- related to peak detection
  double decayRate_{0.5};            // inverse of averaging time
  ros::Duration binInterval_{1e-3};  // time slice for rate binning
  double binIntervalSec_{1e-3};      // time slice for rate binning
  double peakDetectionRatio_{0.25};  // how many stddev from avg
  size_t eventCount_[2];             // number of events occured in bin
  size_t intCount_[2];               // number of integrated events
  uint32_t numBinsWhereRateBelowThreshold_{0};
  uint32_t numBinsDelayIntegration_{0};
  uint32_t numBinsIntegrated_{0};
  ros::Time lastBinTime_;
  ros::Time startTime_;
  double rateMeanSum_{0};
  double rateCovSum_{0};
  bool integrateEventsIntoImage_ = false;
#ifdef DEBUG
  std::ofstream rateDebugFile_;
#endif
};
}  // namespace event_ros_tools
#endif
