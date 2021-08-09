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

#include "event_ros_tools/denoiser.h"

#include <opencv2/photo/photo.hpp>

namespace event_ros_tools
{
StampedImage Denoiser::addFrame(const StampedImage & si)
{
  imageQueue_.push_front(si);
  while (imageQueue_.size() > maxQueueSize_) {
    imageQueue_.pop_back();
  }
  if (imageQueue_.size() <= maxQueueSize_ / 2) {
    // just do single image denoising while the queue
    // is being built up
    return (denoiseSingleImage(*(imageQueue_.rbegin())));
  } else if (imageQueue_.size() == maxQueueSize_) {
    // got enough in the queue to denoise the center image
    return (denoiseCenterImage());
  }
  // cannot denoise yet because queue is not full yet, return null pointer
  return (StampedImage(si.time, std::shared_ptr<cv::Mat>()));
}

StampedImage Denoiser::denoiseSingleImage(const StampedImage & si)
{
  StampedImage denoised(si.time, std::make_shared<cv::Mat>());
  cv::fastNlMeansDenoising(
    *(si.image), *(denoised.image), h_, templateWindowSize_, searchWindowSize_);
  return (denoised);
}

StampedImage Denoiser::denoiseCenterImage()
{
  const int centerIdx = imageQueue_.size() / 2;
  const auto & centerImage = imageQueue_[centerIdx];
  StampedImage denoised(centerImage.time, std::make_shared<cv::Mat>());
  std::vector<cv::Mat> images(imageQueue_.size());
  for (size_t i = 0; i < imageQueue_.size(); i++) {
    images[i] = *(imageQueue_[i].image);
  }
  cv::fastNlMeansDenoisingMulti(
    images, *(denoised.image), centerIdx, (int)imageQueue_.size(), h_,
    templateWindowSize_, searchWindowSize_);
  return (denoised);
}

void Denoiser::setNumberOfFrames(size_t n) { maxQueueSize_ = n; }

}  // namespace event_ros_tools
