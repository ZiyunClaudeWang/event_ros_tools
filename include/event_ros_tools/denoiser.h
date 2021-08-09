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

#ifndef EVENT_ROS_TOOLS_DENOISER_H_
#define EVENT_ROS_TOOLS_DENOISER_H_

#include <ros/ros.h>

#include <deque>
#include <memory>
#include <opencv2/core/core.hpp>

#include "event_ros_tools/stamped_image.h"

namespace event_ros_tools
{
class Denoiser
{
public:
  Denoiser(){};
  void setTemplateWindowSize(int s) { templateWindowSize_ = s; }
  void setSearchWindowSize(int s) { searchWindowSize_ = s; }
  void setNumberOfFrames(size_t s);
  void setH(float h) { h_ = h; }
  // will return denoised frame if available, else will return
  // null pointer for image field
  StampedImage addFrame(const StampedImage & si);

  // return denoised frames from buffer, will return
  // null pointer for image field if buffer is empty
  StampedImage drainBuffer();

private:
  StampedImage denoiseSingleImage(const StampedImage & si);
  StampedImage denoiseCenterImage();
  // ------ variables ---------
  std::deque<StampedImage> imageQueue_;
  float h_{3.0};
  int templateWindowSize_{7};
  int searchWindowSize_{21};
  size_t maxQueueSize_{3};
};
}  // namespace event_ros_tools

#endif
