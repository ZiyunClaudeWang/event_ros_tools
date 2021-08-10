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

//#define DEBUG

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
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

#include "event_ros_tools/FlickerDynConfig.h"
#include "event_ros_tools/denoiser.h"
#include "event_ros_tools/stamped_image.h"

#ifdef DEBUG
#include <fstream>
#endif

namespace event_ros_tools
{
template <class MsgType>
class Flicker
{
public:
  using Config = FlickerDynConfig;

  Flicker(const ros::NodeHandle & nh) : nh_(nh)
  {
#ifdef DEBUG
    rateDebugFile_.open("rate_debug_file.txt");
    intDebugFile_[0].open("int_debug_OFF.txt");
    intDebugFile_[1].open("int_debug_ON.txt");
    winDebugFile_[0].open("win_debug_OFF.txt");
    winDebugFile_[1].open("win_debug_ON.txt");
    highLowDebugFile_[0].open("high_low_OFF.txt");
    highLowDebugFile_[1].open("high_low_ON.txt");
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
    thread_ =
      std::make_shared<std::thread>(&Flicker::imageProcessingThread, this);
    binInterval_ = nh_.param<double>("bin_interval", 1e-3);
    binIntervalInv_ = 1.0 / binInterval_;
    warmupPeriodCount_ = nh_.param<int>("number_of_warmup_periods", 10);
    binDuration_ = ros::Duration(binInterval_);
    denoiser_ = std::make_shared<Denoiser>();
    // NOTE: must hook up the dyn reconfig server here!
    // It will issue a callback in-thread so subsequently all parameters
    // (e.g. decayRate_) are initialized properly BEFORE subscribing to
    // events
    configServer_.reset(new dynamic_reconfigure::Server<Config>(nh_));
    configServer_->setCallback(boost::bind(&Flicker::configure, this, _1, _2));

    image_transport::ImageTransport it(nh_);
    imagePub_ = it.advertise("image", 1);
    double printInterval;
    nh_.param<double>("statistics_print_interval", printInterval, 1.0);
    printInterval_ = ros::Duration(printInterval);
    for (int i = 0; i < 2; i++) {
      eventCount_[i] = 0;
      rate_[i] = 1e3;  // assume 1kev rate to start
      rateCov_[i] = 0;
      dt_ = 0;
      isHigh_[i] = false;
    }
    // NOTE: must wait with subscribing until init is complete!
    int qs = nh_.param<int>("event_queue_size", 1000);
    sub_ = nh_.subscribe("events", qs, &Flicker::eventCallback, this);
    ROS_INFO_STREAM("flicker initialized!");
    return (true);
  }

private:
  int make_odd(int i) { return (i % 2 == 0 ? i + 1 : i); }
  void configure(Config & config, int level)
  {
    decayRate_ = 1.0 / config.rate_averaging_time;
    if (level < 0) {
      // only set warmup bin count on initial call
      warmupBinCount_ = (int)(4.0 / decayRate_ / binInterval_);
    }
    for (int i = 0; i < 2; i++) {
      lead_[i] = ros::Duration(config.lead_time);
      duration_[i] = ros::Duration(config.duration);
    }
    avoidTearing_ = config.avoid_tearing;
    integrateEventType_ = config.use_on_events ? 1 : 0;
    if (denoiser_) {
      denoise_ = config.denoise;
      const int nf = make_odd(config.denoise_num_frames);
      denoiser_->setNumberOfFrames(nf);
      config.denoise_num_frames = nf;
      const int tws = make_odd(config.denoise_template_window_size);
      denoiser_->setTemplateWindowSize(tws);
      config.denoise_template_window_size = tws;
      const int sws = make_odd(config.denoise_search_window_size);
      denoiser_->setSearchWindowSize(sws);
      config.denoise_search_window_size = sws;
      denoiser_->setH(config.denoise_h);
    }
  }

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
    if (t_end > lastPrintTime_ + printInterval_) {
      const float avgRate =
        1e-6 * totalEvents_ * (totalTime_ > 0 ? 1.0 / totalTime_ : 0);
      const float avgSize = totalEvents_ / (float)totalMsgs_;
      ROS_INFO(
        "rcv msg sz: %4d ev,  rt[Mevs] avg: %7.3f, max: %7.3f, "
        "drop: %3d, qs: %2zu, dT: %5.3f",
        (int)avgSize, avgRate, maxRate_, dropped_, maxQueueSize_, dt_);
      maxRate_ = 0;
      lastPrintTime_ += printInterval_;
      totalEvents_ = 0;
      totalMsgs_ = 0;
      totalTime_ = 0;
      dropped_ = 0;
      maxQueueSize_ = 0;
    }
  }

  void imageProcessingThread()
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
#ifdef DEBUG
      startTime_ = msg.header.stamp;
#endif
      for (int i = 0; i < 2; i++) {
        highTime_[i] = msg.header.stamp;
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
    if (denoise_) {
      StampedImage orig(header.stamp, std::make_shared<cv::Mat>());
      cv::equalizeHist(normImg, *(orig.image));
      StampedImage denoised = denoiser_->addFrame(orig);
      if (denoised.image) {
        imagePub_.publish(
          cv_bridge::CvImage(header, "mono8", *denoised.image).toImageMsg());
      }
    } else {
      cv::Mat ei;
      cv::equalizeHist(normImg, ei);
      imagePub_.publish(cv_bridge::CvImage(header, "mono8", ei).toImageMsg());
    }
  }

  void resetImage(const int width, const int height)
  {
    image_.reset(new cv::Mat(height, width, CV_32SC1, cv::Scalar(0)));
  }

  void maintainIntegrationWindow(const ros::Time & t)
  {
    for (int i = 0; i < 2; i++) {  // loop through ON/OFF
      const double rate = eventCount_[i] * binIntervalInv_;
      const double rdiff = rate - rate_[i];
      const double rateStd = std::sqrt(rateCov_[i] - rate_[i] * rate_[i]);
      const bool isHighNow = rdiff > 0.5 * rateStd;
      const double dt = (t - highTime_[i]).toSec();
      const bool isUpSlope = !isHigh_[i] && isHighNow;
      if (isUpSlope && (!periodEstablished_ || dt > 0.25 * dt_)) {
        // crossed the peak threshold
        const double alpha = periodEstablished_ ? 0.1 : 0.2;
        // cannot ask for err less than 2 * time bin
        const double minErr = 4 * binInterval_ * binInterval_;
        const double dtCov = std::max(dtSumSq_ - dt_ * dt_, minErr);
        const double dtErr = dt - dt_;
        if (dtErr * dtErr < 1.0 * dtCov || !periodEstablished_) {
          dt_ = dt_ * (1.0 - alpha) + dt * alpha;
          dtSumSq_ = dtSumSq_ * (1.0 - alpha) + dt * dt * alpha;
        } else {
          ROS_WARN(
            "dropped dt update: %7.4f vs avg: %7.4f+-%7.4f", dt, dt_,
            std::sqrt(dtCov));
        }
        if (!periodEstablished_) {
          if (warmupPeriodCount_ > 0) {
            warmupPeriodCount_--;
          }
          if (warmupPeriodCount_ == 0) {
            // wait until covariance is reasonable
            const double sd =
              std::max(std::sqrt(dtSumSq_ - dt_ * dt_), binInterval_);
            if (sd < 0.05 * dt_) {
              ROS_INFO_STREAM("period established as " << dt_ << "s");
              periodEstablished_ = true;
            } else {
              ROS_INFO(
                "period: %5.3fs, waiting for lower noise (%5.3fs)", dt_, sd);
            }
          }
        }
        highTime_[i] = t;   // remember time of up slope
        isHigh_[i] = true;  // mark as being high
        // when an ON event spike happens it is safe to advance
        // the OFF integration window, and vice versa.
        const int i_oth = (i + 1) % 2;
        const ros::Duration period(dt_);
        // time of new start window
        const auto tws = highTime_[i_oth] + period - lead_[i_oth];
        timeWindowStart_[i_oth] = tws;
        timeWindowEnd_[i_oth] = timeWindowStart_[i_oth] + duration_[i_oth];
#ifdef DEBUG
        winDebugFile_[i_oth]
          << (t - startTime_) << " " << (highTime_[i_oth] - startTime_) << " "
          << (timeWindowStart_[i_oth] - startTime_) << " "
          << (timeWindowEnd_[i_oth] - startTime_) << std::endl;
#endif
      }  // if switched from low to high
      const bool isLowNow = rdiff < -0.1 * rateStd;
      if (isHigh_[i] && isLowNow) {
        isHigh_[i] = false;  // reset high indicator
      }
#ifdef DEBUG
      highLowDebugFile_[i] << (t - startTime_) << " " << (int)isHighNow << " "
                           << (int)isLowNow << " " << (int)isHigh_[i] << " "
                           << (int)periodEstablished_ << std::endl;
#endif
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
      rate_[i] = rate_[i] * decay + rate * om_decay;
      rateCov_[i] = rateCov_[i] * decay + rate * rate * om_decay;
#ifdef DEBUG
      rateDebugFile_ << " " << rate << " " << rate_[i] << " "
                     << std::sqrt(rateCov_[i] - rate_[i] * rate_[i]);
#endif
    }
    if (warmupBinCount_ == 0) {
      // the mean and variance of the rate are now established, start detecting
      // period, up and down slopes, and update integration window
      maintainIntegrationWindow(t);
    } else {
      warmupBinCount_--;
      if (warmupBinCount_ == 0) {
        ROS_INFO(
          "established rate averages OFF: %6.3f(+-%6.3f)Mevs, ON: "
          "%6.3f(+-%6.3f)Mevs",
          rate_[0] * 1e-6, std::sqrt(rateCov_[0] - rate_[0] * rate_[0]) * 1e-6,
          rate_[1] * 1e-6, std::sqrt(rateCov_[1] - rate_[1] * rate_[1]) * 1e-6);
        ROS_INFO("now waiting for period to be established");
        // initialize flip time and window start
        for (int i = 0; i < 2; i++) {
          highTime_[i] = t;
          timeWindowStart_[i] = timeWindowEnd_[i] = t;
        }
      }
    }
#ifdef DEBUG
    rateDebugFile_ << std::endl;
#endif
    eventCount_[0] = eventCount_[1] = 0;
  }

  inline void integrateIntoImage(int x, int y)
  {
    image_->at<int32_t>(y, x) += 1;
#ifdef DEBUG
    y_min_ = std::min(y_min_, y);
    y_max_ = std::max(y_max_, y);
#endif
  }

  bool updateImage(const MsgType & msg)
  {
#ifdef DEBUG
    y_min_ = msg.height;
    y_max_ = -1;
#endif
    const int EVENT_TYPE_TO_USE = integrateEventType_;
    bool imageComplete(false);
    for (const auto & ev : msg.events) {
      const auto t = ev.ts;
      while (t > lastBinTime_ + binDuration_) {
        lastBinTime_ += binDuration_;
        // t has crossed into new time bin, process old one
        // update rate estimate and maintain integration window
        updateRate(t);
      }
      const int p = (int)ev.polarity;
      eventCount_[p]++;
      if (p != EVENT_TYPE_TO_USE || !periodEstablished_) {
        continue;
      }
      if (keepIntegrating_) {  // currently integrating beyond time window
        integrateIntoImage((int)ev.x, (int)ev.y);
        if ((int)ev.y > msg.height - 2) {  // event from bottom of image,
          keepIntegrating_ = false;        // "frame" is complete, stop it.
          imageComplete = true;            // done with image.
          isIntegrating_ = false;
        }
        continue;
      }
      if (t > timeWindowStart_[p]) {
        if (t < timeWindowEnd_[p]) {  // t is inside integration window
          if (!isIntegrating_) {
            isIntegrating_ = true;
          }
          integrateIntoImage((int)ev.x, (int)ev.y);
        } else {  // t is beyond integration window
          if (isIntegrating_) {
            if (avoidTearing_) {
              keepIntegrating_ = true;
            } else {
              isIntegrating_ = false;
              imageComplete = true;
            }
          }
        }
      }

    }  // loop over events
#ifdef DEBUG
    const auto msgTime = msg.header.stamp;
    if (y_min_ < (int)msg.height && y_max_ > -1) {
      tearingFile_ << (msgTime - startTime_) << " " << y_min_ << " " << y_max_
                   << std::endl;
    }
    intDebugFile_[0] << (msgTime - startTime_) << " "
                     << (timeWindowStart_[EVENT_TYPE_TO_USE] - startTime_)
                     << " " << (timeWindowEnd_[EVENT_TYPE_TO_USE] - startTime_)
                     << " " << (int)isIntegrating_ << " "
                     << (int)keepIntegrating_ << " " << (int)imageComplete
                     << std::endl;
#endif
    return (imageComplete);
  }

  // ------- variables
  // -- related to statistics
  ros::Duration printInterval_{1.0};  // time between statistics logging
  float maxRate_{0};                  // maximum total ON+OFF event rate
  uint64_t totalEvents_{0};           // total events during stats interval
  float totalTime_{0};                // total time during interval
  uint32_t totalMsgs_{0};             // total msgs received during interval
  uint32_t dropped_{0};               // number of ROS msg drops detected
  size_t maxQueueSize_{0};            // maximum output queue size
  ros::Time lastPrintTime_{0};
  uint32_t lastSequence_{0};
  // -- related to rate and period computation
  uint32_t warmupBinCount_{0};       // number of time bins to wait for warmup
  uint32_t warmupPeriodCount_{0};    // number of periods to wait for warmup
  bool periodEstablished_{false};    // true if initial period has been found
  double decayRate_{0.5};            // inverse of averaging time
  double binInterval_{1e-3};         // time slice for rate binning
  double binIntervalInv_{1e3};       // inverse of time slice for rate binning
  ros::Duration binDuration_{1e-3};  // time slice for rate binning
  ros::Time lastBinTime_;            // time when last bin started
  size_t eventCount_[2];             // number of events occured in bin
  double rate_[2];                   // current estimate of total rate
  double rateCov_[2];                // current estimate of rate covariance
  double dt_;                        // current estimated signal period
  double dtSumSq_{0};                // avg sum of sq of signal period
  // -- related to integration time window management and slope detection
  ros::Duration duration_[2];     // time window width for integration
  ros::Duration lead_[2];         // lead time for capture window
  ros::Time highTime_[2];         // last time signal went high
  ros::Time timeWindowStart_[2];  // start of integration window
  ros::Time timeWindowEnd_[2];    // end of integration window
  bool isHigh_[2];                // whether rate is currently high or low
  // -- related to actual integration
  int integrateEventType_{1};       // use 0=OFF or 1=ON events
  bool isIntegrating_{false};       // true while image integration happens
  bool keepIntegrating_{false};     // control integration beyond time slice
  bool avoidTearing_{true};         // keep integrating until frame complete
  std::shared_ptr<cv::Mat> image_;  // working image being integrated
  // -- denoising
  bool denoise_{false};                 // true if denoising should happen
  std::shared_ptr<Denoiser> denoiser_;  // denoiser object
  // -- related to multithreading
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<StampedImage> imageQueue_;
  std::shared_ptr<std::thread> thread_;
  bool keepRunning_{true};
  // -- ROS related stuff
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  image_transport::Publisher imagePub_;
  std_msgs::Header header_;  // prepared message header
  std::shared_ptr<dynamic_reconfigure::Server<Config>> configServer_;
#ifdef DEBUG
  std::ofstream rateDebugFile_;
  std::ofstream intDebugFile_[2];
  std::ofstream winDebugFile_[2];
  std::ofstream highLowDebugFile_[2];
  std::ofstream tearingFile_;
  ros::Time startTime_;  // time stamp on first incoming message
  int y_min_;            // for frame debugging
  int y_max_;
#endif
};
}  // namespace event_ros_tools
#endif
