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

#include <dvs_msgs/EventArray.h>
#include <event_array2_msgs/EventArray2.h>
#include <event_array2_msgs/decode.h>
#include <prophesee_event_msgs/EventArray.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <unistd.h>

#include <chrono>
#include <fstream>

using namespace std::chrono;

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "bag_to_text -i name_of_bag_file "
            << "-o name_of_text_file [-t topic] [-m message_type] [-a "
               "aperture] [-x x_center] [-y y_center]"
            << std::endl;
}

template <typename MsgType>
size_t write_events(
  std::ofstream & out, typename MsgType::ConstPtr s, int a, int x, int y, const ros::Time & t0)
{
  for (const auto e : s->events) {
    if (a < 0) {
      out << (e.ts - t0).toSec() << " " << e.x << " " << e.y << " " << static_cast<int>(e.polarity)
          << std::endl;
    } else {
      if (std::abs(e.x - x) <= a && std::abs(e.y - y) <= a) {
        out << (e.ts - t0).toSec() << " " << (a + e.x - x) << " " << (a + e.y - y) << " "
            << static_cast<int>(e.polarity) << std::endl;
      }
    }
  }
  return (s->events.size());
}

template <>
size_t write_events<event_array2_msgs::EventArray2>(
  std::ofstream & out, event_array2_msgs::EventArray2::ConstPtr s, int a, int x, int y,
  const ros::Time & t0)
{
  const uint64_t time_base = s->time_base;
  for (const auto e : s->p_y_x_t) {
    uint16_t ex, ey;
    uint64_t etu;
    bool p = event_array2_msgs::decode_t_x_y_p(e, time_base, &etu, &ex, &ey);
    ros::Time et;
    et.fromNSec(etu);
    if (a < 0) {  // no aperture given
      out << (et - t0).toSec() << " " << ex << " " << ey << " " << static_cast<int>(p) << std::endl;
    } else {
      if (std::abs(ex - x) <= a && std::abs(ey - y) <= a) {
        out << (et - t0).toSec() << " " << (a + ex - x) << " " << (a + ey - y) << " "
            << static_cast<int>(p) << std::endl;
      }
    }
  }
  return (s->p_y_x_t.size());
}

template <typename MsgType>
size_t process_bag(rosbag::View & view, std::ofstream & out, int a, int x, int y)
{
  size_t num_events = 0;
  bool writeHeader(true);
  ros::Time t0;
  for (rosbag::MessageInstance const m : view) {
    typename MsgType::ConstPtr s = m.instantiate<MsgType>();
    if (s) {
      if (writeHeader) {
        t0 = s->header.stamp;
        writeHeader = false;
        if (a >= 0) {
          x = x < 0 ? s->width / 2 : x;
          y = y < 0 ? s->height / 2 : y;
          out << (2 * a + 1) << " " << (2 * a + 1) << std::endl;
        } else {
          out << s->width << " " << s->height << std::endl;
        }
      }
      num_events += write_events<MsgType>(out, s, a, x, y, t0);
    }
  }
  return (num_events);
}

int main(int argc, char ** argv)
{
  int opt;

  std::string inFile;
  std::string outFile;
  std::string topic("/metavision_driver/events");
  std::string messageType("dvs");
  int aperture(0);
  int x(-1), y(-1);
  while ((opt = getopt(argc, argv, "i:o:t:m:a:x:y:")) != -1) {
    switch (opt) {
      case 'i':
        inFile = optarg;
        break;
      case 'o':
        outFile = optarg;
        break;
      case 't':
        topic = optarg;
        break;
      case 'm':
        messageType = optarg;
        break;
      case 'a':
        aperture = std::stoi(optarg);
        break;
      case 'x':
        x = std::stoi(optarg);
        break;
      case 'y':
        y = std::stoi(optarg);
        break;
      default:
        std::cout << "unknown option: " << opt << std::endl;
        usage();
        return (-1);
        break;
    }
  }
  if (inFile.empty() || outFile.empty()) {
    std::cout << "missing input or output file name!" << std::endl;
    usage();
    return (-1);
  }
  rosbag::Bag bag;
  bag.open(inFile, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(topic);
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  std::ofstream out(outFile);
  // Get ending timepoint
  auto start = high_resolution_clock::now();
  size_t numEvents = 0;
  if (messageType == "prophesee") {
    numEvents += process_bag<prophesee_event_msgs::EventArray>(view, out, aperture, x, y);
  } else if (messageType == "dvs") {
    numEvents += process_bag<dvs_msgs::EventArray>(view, out, aperture, x, y);
  } else if (messageType == "event_array2") {
    numEvents += process_bag<event_array2_msgs::EventArray2>(view, out, aperture, x, y);
  } else {
    std::cout << "invalid message type: " << messageType << std::endl;
    return (-1);
  }
  auto final = high_resolution_clock::now();
  auto total_duration = duration_cast<microseconds>(final - start);

  std::cout << "number of events read: " << numEvents * 1e-6 << " Mev in "
            << total_duration.count() * 1e-6 << " seconds" << std::endl;
  std::cout << "event rate: " << static_cast<double>(numEvents) / total_duration.count() << " Mevs"
            << std::endl;
  bag.close();
  return (0);
}
