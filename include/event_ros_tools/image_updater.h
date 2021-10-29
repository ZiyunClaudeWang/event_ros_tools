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

#ifndef EVENT_ROS_TOOLS__IMAGE_UPDATER_H_
#define EVENT_ROS_TOOLS__IMAGE_UPDATER_H_

#include <memory>
#include <opencv2/core/core.hpp>

namespace event_ros_tools
{
class ImageUpdater
{
public:
  virtual ~ImageUpdater() {}
  virtual void doUpdate(uint64_t t, uint16_t x, uint16_t y, bool p) = 0;
  inline void update(uint64_t t, uint16_t x, uint16_t y, bool p) { doUpdate(t, x, y, p); }

  virtual void resetImage(uint32_t width, uint32_t height)
  {
    image_ = cv::Mat::zeros(height, width, CV_32SC1);
  }
  virtual void resetImage() { image_ = cv::Mat::zeros(image_.rows, image_.cols, CV_32SC1); }
  const cv::Mat & getImage() const { return (image_); }
  bool hasValidImage() const { return (image_.rows > 0); }

  // factory method to create image updaters. allowed values for mode are:
  //
  // ignore_polarities
  // ignore_polarities_inv
  // respect_polarities
  // respect_polarities_inv
  //
  static std::shared_ptr<ImageUpdater> make(const std::string & mode);

protected:
  // ------- variables
  cv::Mat image_;
};  // namespace event_ros_tools

}  // namespace event_ros_tools
#endif  // EVENT_ROS_TOOLS__IMAGE_UPDATER_H_
