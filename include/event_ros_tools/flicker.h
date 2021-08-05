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

#include <chrono>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>

#ifdef DEBUG
#include <fstream>
#endif

#define MAKE_FULL_FRAME

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
    intDebugFile_[0].open("int_debug_OFF.txt");
    intDebugFile_[1].open("int_debug_ON.txt");
    winDebugFile_[0].open("win_debug_OFF.txt");
    winDebugFile_[1].open("win_debug_ON.txt");
    tearingFile_.open("tearing.txt");
#endif
  }
  ~Flicker() { shutdown(); }

  void shutdown()
  {
    if (thread_) {
      keepRunning_ = false;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.notify_all();
      }
      thread_->join();
      thread_.reset();
    }
  }

  bool initialize()
  {
    thread_ = std::make_shared<std::thread>(&Flicker::publishingThread, this);
    decayRate_ = 1.0 / nh_.param<double>("averaging_time", 0.5);
    binInterval_ = nh_.param<double>("bin_interval", 1e-3);
    warmupBinCount_ = (int)(4.0 / decayRate_ / binInterval_);
    warmupPeriodCount_ = 10;
    binIntervalInv_ = 1.0 / binInterval_;
    binDuration_ = ros::Duration(binInterval_);
    deadTime_ = ros::Duration(nh_.param<double>("initial_dead_time", 5e-3));
    int qs = nh_.param<int>("event_queue_size", 1000);
    sub_ = nh_.subscribe("events", qs, &Flicker::eventCallback, this);
    image_transport::ImageTransport it(nh_);
    imagePub_ = it.advertise("image", 1);
    ratePub_ = nh_.advertise<Rate>("rate", 10);
    double printInterval;
    nh_.param<double>("statistics_print_interval", printInterval, 1.0);
    statisticsPrintInterval_ = ros::Duration(printInterval);
    for (int i = 0; i < 2; i++) {
      eventCount_[i] = 0;
      intCount_[i] = 0;
      rateMean_[i] = 1e3;  // assume 1kev rate to start
      dt_[i] = 0;
      isHigh_[i] = false;
      lead_[i] = ros::Duration(nh_.param<double>("lead_time", 1e-3));
      duration_[i] = ros::Duration(nh_.param<double>("duration", 5e-3));
      periodsCounted_[i] = 0;
    }
    statisticsIntCount_[0] = statisticsIntCount_[1] = 0;
    ROS_INFO_STREAM("flicker initialized!");
    return (true);
  }

private:
  struct StampedImage
  {
    StampedImage(const ros::Time & t, const std::shared_ptr<cv::Mat> & img)
    : time(t), image(img)
    {
    }
    ros::Time time;
    std::shared_ptr<cv::Mat> image;
  };

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
        "drop: %3d, qs: %2zu, dT: %5.3f",
        (int)avgSize, avgRate, maxRate_, integ, dropped_, maxQueueSize_,
        dt_[0]);
      maxRate_ = 0;
      lastPrintTime_ += statisticsPrintInterval_;
      totalEvents_ = 0;
      totalMsgs_ = 0;
      totalTime_ = 0;
      statisticsIntCount_[0] = statisticsIntCount_[1] = 0;
      dropped_ = 0;
      maxQueueSize_ = 0;
    }
  }

  void publishingThread()
  {
    const std::chrono::microseconds timeout((int64_t)(1000000LL));
    while (ros::ok() && keepRunning_) {
      StampedImage si(ros::Time(0), std::shared_ptr<cv::Mat>());
      size_t qs = 0;
      {  // critical section, no processing done here
        std::unique_lock<std::mutex> lock(mutex_);
        while (ros::ok() && keepRunning_ && imageQueue_.empty()) {
          cv_.wait_for(lock, timeout);
        }
        if (!imageQueue_.empty()) {
          qs = imageQueue_.size();
          si = imageQueue_.back();
          imageQueue_.pop_back();
        }
      }
      if (si.image) {
        header_.seq++;
        header_.stamp = si.time;
        publishImage(header_, si.image);
        maxQueueSize_ = std::max(maxQueueSize_, qs);
      }
    }
    ROS_INFO("publishing thread exited!");
  }

  void eventCallback(const MsgType & msg)
  {
    if (msg.events.empty()) {
      return;
    }
    if (!image_) {
      resetImage(msg.width, msg.height);
      ROS_INFO_STREAM("image has size " << msg.width << " x " << msg.height);
      header_ = msg.header;
      lastSequence_ = msg.header.seq;
      lastPrintTime_ = msg.header.stamp;
      lastBinTime_ = msg.header.stamp;
      startTime_ = msg.header.stamp;
      for (int i = 0; i < 2; i++) {
        flipTime_[0][i] = flipTime_[1][i] = msg.header.stamp;
        timeWindowStart_[i] = timeWindowEnd_[i] = msg.header.stamp;
      }
    }
    updateStatistics(msg);
    if (imagePub_.getNumSubscribers() != 0) {
      if (updateImage(msg)) {
        {
          std::unique_lock<std::mutex> lock(mutex_);
          imageQueue_.push_front(StampedImage(msg.header.stamp, image_));
          cv_.notify_all();
        }
        resetImage(image_->cols, image_->rows);
      }
    }
  }

  void publishImage(
    const std_msgs::Header & header, const std::shared_ptr<cv::Mat> & image)
  {
#ifdef DEBUG_IMAGE
    double min, max;
    cv::minMaxLoc(*image, &min, &max);
    std::cout << "min: " << min << " max: " << max << std::endl;
#endif
    cv::Mat normImg;
    cv::normalize(*image, normImg, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::Mat imgU8, eqImg;
    cv::equalizeHist(normImg, eqImg);
    imagePub_.publish(cv_bridge::CvImage(header, "mono8", eqImg).toImageMsg());
  }

  void resetImage(const int width, const int height)
  {
    image_.reset(new cv::Mat(height, width, CV_32SC1, cv::Scalar(0)));
  }

  void publishRate(
    const std_msgs::Header & header, double rate, double stddev,
    double offRatio)
  {
    Rate rateMsg;
    rateMsg.header = header;
    rateMsg.rate = rate * 1e-6;
    rateMsg.mean_rate = rateMean_[0] * 1e-6;
    rateMsg.std_rate = stddev * 1e-6;
    rateMsg.off_ratio = offRatio;
    ratePub_.publish(rateMsg);
  }

  void updatePeriod(const ros::Time & t)
  {
    // the rate and mean are now established, start detecting
    // up and down flanks
    for (int i = 0; i < 2; i++) {  // loop through ON/OFF
      const double rate = eventCount_[i] * binIntervalInv_;
      const double rdiff = rate - rateMean_[i];
      const double rateStd =
        std::sqrt(rateCov_[i] - rateMean_[i] * rateMean_[i]);
      const bool isHighNow = rdiff > rateMean_[i] + 1.0 * rateStd;
      if (!isHigh_[i] && isHighNow) {
        // crossed the peak threshold
        const double dt = (t - flipTime_[1][i]).toSec();
        const double alpha = periodEstablished_ ? 0.1 : 0.2;
        // cannot ask for err less than 2 * time bin
        const double minErr = 4 * binInterval_ * binInterval_;
        const double dtCov = std::max(dtCov_ - dt_[0] * dt_[0], minErr);
        const double dtErr = dt - dt_[0];
        if (dtErr * dtErr < 1.0 * dtCov || !periodEstablished_) {
          dt_[0] = dt_[0] * (1.0 - alpha) + dt * alpha;
          dtCov_ = dtCov_ * (1.0 - alpha) + dt * dt * alpha;
        } else {
          ROS_WARN_STREAM(
            "ignoring dt update " << (i == 0 ? "OFF" : "ON") << " " << dt
                                  << " vs avg: " << dt_[0]
                                  << " stddev: " << std::sqrt(dtCov));
        }
        if (!periodEstablished_) {
          if (warmupPeriodCount_ > 0) {
            warmupPeriodCount_--;
          }
          if (warmupPeriodCount_ == 0) {
            // don't start until the covariance is reasonable
            const double stddev =
              std::max(std::sqrt(dtCov_ - dt_[0] * dt_[0]), binInterval_);
            if (stddev < 0.05 * dt_[0]) {
              ROS_INFO_STREAM("period established as " << dt_[0] << "s");
              periodEstablished_ = true;
            } else {
              ROS_INFO(
                "period: %5.3fs, waiting for stddev (%5.3fs) to decrease",
                dt_[0], stddev);
            }
          }
        }
        flipTime_[1][i] = t;  // remember time of up flank
        isHigh_[i] = true;
      }
      const bool isLowNow = rate < rateMean_[i] - 0.1 * rateStd;
      if (isHigh_[i] && isLowNow) {
        isHigh_[i] = false;  // reset high indicator
      }
    }
  }
  void updateRate(const ros::Time & t)
  {
#ifdef DEBUG
    rateDebugFile_ << t - startTime_;
#endif
    const double decay = exp(-binInterval_ * decayRate_);
    const double om_decay = 1.0 - decay;
    // Update the rate mean and covariance first,
    // even during the warmup phase
    for (int i = 0; i < 2; i++) {  // loop through ON/OFF
      const double rate = eventCount_[i] * binIntervalInv_;
      // discount running sums and add new rate
      rateMean_[i] = rateMean_[i] * decay + rate * om_decay;
      rateCov_[i] = rateCov_[i] * decay + rate * rate * om_decay;
#ifdef DEBUG
      rateDebugFile_ << " " << rate << " " << rateMean_[i] << " "
                     << std::sqrt(rateCov_[i] - rateMean_[i] * rateMean_[i]);
#endif
    }
    if (warmupBinCount_ == 0) {
      // the mean and variance of the rate are now established, start detecting
      // up and down flanks, periods, integration windows
      updatePeriod(t);
    } else {
      warmupBinCount_--;
      if (warmupBinCount_ == 0) {
        ROS_INFO(
          "established rate averages OFF: %6.3f(+-%6.3f)Mevs, ON: "
          "%6.3f(+-%6.3f)Mevs",
          rateMean_[0] * 1e-6,
          std::sqrt(rateCov_[0] - rateMean_[0] * rateMean_[0]) * 1e-6,
          rateMean_[1] * 1e-6,
          std::sqrt(rateCov_[1] - rateMean_[1] * rateMean_[1]) * 1e-6);
        ROS_INFO("now waiting for period to be established");
        // initialize flip time and window start
        for (int i = 0; i < 2; i++) {
          flipTime_[0][i] = flipTime_[1][i] = t;
          timeWindowStart_[i] = timeWindowEnd_[i] = t;
        }
      }
    }
#ifdef DEBUG
    rateDebugFile_ << std::endl;
#endif
    eventCount_[0] = eventCount_[1] = 0;
  }

  bool updateImage(const MsgType & msg)
  {
#ifdef DEBUG
    int y_min = msg.height;
    int y_max = -1;
#else
#ifdef MAKE_FULL_FRAME
    int y_max = -1;
#endif
#endif
    bool imageComplete(false);
    for (const auto & ev : msg.events) {
      const auto t = ev.ts;
      while (t > lastBinTime_ + binDuration_) {
        // t has crossed into new time bin, process old one
        lastBinTime_ += binDuration_;
        // update the current period estimate and the
        // next integration window
        updateRate(t);
      }  // while interval loop
      const auto p = ev.polarity;
      eventCount_[p]++;
      if (p == 0) {
        if (t > timeWindowStart_[p] || keepIntegrating_) {
          if (
            t < timeWindowEnd_[p] ||
            keepIntegrating_) {  // we are within the window
            image_->at<int32_t>(ev.y, ev.x) += 1;
            intCount_[p]++;
            statisticsIntCount_[p]++;
#ifdef DEBUG
            y_min = std::min(y_min, (int)ev.y);
            y_max = std::max(y_max, (int)ev.y);
#endif
#ifdef MAKE_FULL_FRAME
            if ((int)ev.y > msg.height - 2 && keepIntegrating_) {
              keepIntegrating_ = false;
              imageComplete = true;
            }
#endif
          } else {
            // we are beyond the end of the integration window,
            // move it up by the time period if necessary
            const ros::Duration period(dt_[p]);
            // time of new start window
            const auto tws = flipTime_[1][p] + period - lead_[p];
            if (t < tws) {  // only roll if flip time has updated
              // rolling over to next integration window
              timeWindowStart_[p] = tws;
              timeWindowEnd_[p] = timeWindowStart_[p] + duration_[p];
              winDebugFile_[p] << (t - startTime_) << " "
                               << (timeWindowStart_[p] - t) << " "
                               << (timeWindowEnd_[p] - t) << std::endl;
#ifdef MAKE_FULL_FRAME
              keepIntegrating_ = true;
#else
              imageComplete = true;
#endif
            }
          }
        }
      }
    }  // loop over events
#ifdef DEBUG
    const auto msgTime = msg.header.stamp;
    if (y_min < (int)msg.height && y_max > -1) {
      tearingFile_ << (msgTime - startTime_).toSec() << " " << y_min << " "
                   << y_max << std::endl;
    }
#endif

    return (imageComplete);
  }

  // ------- variables
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher ratePub_;
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
  size_t maxQueueSize_{0};
  // -- related to peak detection
  uint32_t warmupBinCount_{0};       // number of time bins to wait for warmup
  uint32_t warmupPeriodCount_{0};    // number of periods to wait for warmup
  bool periodEstablished_{false};    // true if initial period has been found
  double decayRate_{0.5};            // inverse of averaging time
  double binInterval_{1e-3};         // time slice for rate binning
  ros::Duration binDuration_{1e-3};  // time slice for rate binning
  double binIntervalInv_{1e3};       // inverse of time slice for rate binning
  size_t eventCount_[2];             // number of events occured in bin
  size_t intCount_[2];               // number of integrated events
  ros::Duration duration_[2];        // time window of ON and OFF events to use
  ros::Duration lead_[2];            // lead time for capture window
  ros::Time timeWindowStart_[2];
  ros::Time timeWindowEnd_[2];
  uint32_t numBinsWhereRateBelowThreshold_{0};
  uint32_t numBinsDelayIntegration_{0};
  uint32_t numBinsIntegrated_{0};
  bool keepIntegrating_{false};  // control integration beyond time slice
  ros::Time lastBinTime_;
  ros::Time startTime_;
  double rateMean_[2];
  double rateCov_[2];
  bool isHigh_[2];
  double dt_[2];              // mean time between on and off
  double dtCov_{0};           // tracks covariance (sum of squares)
  ros::Time flipTime_[2][2];  // [HIGH->LOW,LOW->HIGH][ON/OFF]
  uint32_t periodsCounted_[2];
  ros::Duration deadTime_;
  std_msgs::Header header_;
  // -- image integration
  std::shared_ptr<cv::Mat> image_;  // working image being integrated
  // -- related to multithreading
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<StampedImage> imageQueue_;
  std::shared_ptr<std::thread> thread_;
  bool keepRunning_{true};

#ifdef DEBUG
  std::ofstream rateDebugFile_;
  std::ofstream intDebugFile_[2];
  std::ofstream winDebugFile_[2];
  std::ofstream tearingFile_;
#endif
};
}  // namespace event_ros_tools
#endif
