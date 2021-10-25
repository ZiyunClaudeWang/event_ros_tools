// -*-c++-*---------------------------------------------------------------------------------------
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

#include "event_ros_tools/image_updater.h"

namespace event_ros_tools
{
class IgnorePolarities : public ImageUpdater
{
  void doUpdate(uint64_t t, uint16_t x, uint16_t y, bool p) override
  {
    (void)t;
    (void)p;
    image_.at<int32_t>(y, x) += 1;
  }
};

class IgnorePolaritiesInv : public ImageUpdater
{
  void doUpdate(uint64_t t, uint16_t x, uint16_t y, bool p) override
  {
    (void)t;
    (void)p;
    image_.at<int32_t>(y, x) -= 1;
  }
};

class RespectPolarities : public ImageUpdater
{
  void doUpdate(uint64_t t, uint16_t x, uint16_t y, bool p) override
  {
    (void)t;
    image_.at<int32_t>(y, x) += p * 2 - 1;
  }
};

class RespectPolaritiesInv : public ImageUpdater
{
  void doUpdate(uint64_t t, uint16_t x, uint16_t y, bool p) override
  {
    (void)t;
    image_.at<int32_t>(y, x) -= p * 2 - 1;
  }
};

// --------------- factory

std::shared_ptr<ImageUpdater> ImageUpdater::make(const std::string & mode)
{
  if (mode == "ignore_polarities") {
    return (std::make_shared<IgnorePolarities>());
  } else if (mode == "ignore_polarities_inv") {
    return (std::make_shared<IgnorePolaritiesInv>());
  } else if (mode == "respect_polarities") {
    return (std::make_shared<RespectPolarities>());
  } else if (mode == "respect_polarities_inv") {
    return (std::make_shared<RespectPolaritiesInv>());
  } else {
    throw(std::runtime_error("invalid mode!"));
  }
}
}  // namespace event_ros_tools
