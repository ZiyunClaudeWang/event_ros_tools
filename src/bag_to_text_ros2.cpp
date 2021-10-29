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

#include <event_array_msgs/decode.h>
#include <unistd.h>

#include <chrono>
#include <dvs_msgs/msg/event_array.hpp>
#include <event_array_msgs/msg/event_array.hpp>
#include <fstream>
#include <iostream>
#include <prophesee_event_msgs/msg/event_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "bag_to_text -i name_of_bag_file "
            << "-o name_of_text_file [-t topic] [-m message_type] [-a "
               "aperture] [-x x_center] [-y y_center]"
            << std::endl;
}

template <typename MsgType>
size_t write_event(
  std::ofstream & out, const MsgType & s, int a, int x, int y, const rclcpp::Time & t0)
{
  for (const auto e : s.events) {
    const auto dt = (rclcpp::Time(e.ts) - t0).seconds();
    if (a < 0) {
      out << dt << " " << e.x << " " << e.y << " " << static_cast<int>(e.polarity) << std::endl;
    } else {
      if (std::abs(e.x - x) <= a && std::abs(e.y - y) <= a) {
        out << dt << " " << (a + e.x - x) << " " << (a + e.y - y) << " "
            << static_cast<int>(e.polarity) << std::endl;
      }
    }
  }
  return (s.events.size());
}

template <>
size_t write_event<event_array_msgs::msg::EventArray>(
  std::ofstream & out, const event_array_msgs::msg::EventArray & s, int a, int x, int y,
  const rclcpp::Time & t0)
{
  if (s.encoding != "mono") {
    std::cerr << "unknown event encoding: " << s.encoding << std::endl;
    throw std::runtime_error("unknown encoding!");
  }

  const uint64_t time_base = s.time_base;
  uint8_t const * p = &(s.events[0]);
  for (size_t i = 0; i < s.events.size(); i += 8, p += 8) {
    uint16_t ex, ey;
    uint64_t etu;
    bool pol = event_array_msgs::mono::decode_t_x_y_p(p, time_base, &etu, &ex, &ey);
    rclcpp::Time et(etu, t0.get_clock_type());

    const double dt = (et - t0).seconds();
    if (a <= 0) {  // no aperture given
      out << dt << " " << ex << " " << ey << " " << static_cast<int>(pol) << std::endl;
    } else {
      if (std::abs(ex - x) <= a && std::abs(ey - y) <= a) {
        out << dt << " " << (a + ex - x) << " " << (a + ey - y) << " " << static_cast<int>(pol)
            << std::endl;
      }
    }
  }
  const size_t numEvents = s.events.size() / 8;
  return (numEvents);
}

template <class MsgType>
size_t translate_bag(
  const std::string & inFile, const std::string & outFile, const std::string & topic, int aperture,
  int x, int y)
{
  bool writeHeader(true);
  std::ofstream out(outFile);
  size_t numEvents(0);
  {
    rclcpp::Time t0;
    rosbag2_cpp::Reader reader;
    reader.open(inFile);
    rclcpp::Serialization<MsgType> serialization;
    while (reader.has_next()) {
      auto msg = reader.read_next();
      if (msg->topic_name == topic) {
        rclcpp::SerializedMessage serializedMsg(*msg->serialized_data);
        MsgType m;
        serialization.deserialize_message(&serializedMsg, &m);
        if (writeHeader) {
          t0 = rclcpp::Time(m.header.stamp);
          if (aperture >= 0) {
            x = x < 0 ? m.width / 2 : x;
            y = y < 0 ? m.height / 2 : y;
            out << (2 * aperture + 1) << " " << (2 * aperture + 1) << std::endl;
          } else {
            out << m.width << " " << m.height << std::endl;
          }
          writeHeader = false;
        }
        numEvents += write_event<MsgType>(out, m, aperture, x, y, t0);
      }
    }
  }  // close reader when out of scope
  return (numEvents);
}

int main(int argc, char ** argv)
{
  int opt;

  std::string inFile;
  std::string outFile;
  std::string topic("/event_camera/events");
  std::string messageType("event_array");
  int aperture(-1);
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
  auto start = std::chrono::high_resolution_clock::now();
  size_t numEvents(0);
  if (messageType == "dvs") {
    numEvents += translate_bag<dvs_msgs::msg::EventArray>(inFile, outFile, topic, aperture, x, y);
  } else if (messageType == "prophesee") {
    numEvents +=
      translate_bag<prophesee_event_msgs::msg::EventArray>(inFile, outFile, topic, aperture, x, y);
  } else if (messageType == "event_array") {
    numEvents +=
      translate_bag<event_array_msgs::msg::EventArray>(inFile, outFile, topic, aperture, x, y);
  } else {
    std::cout << "invalid message type: " << messageType << std::endl;
  }
  auto final = std::chrono::high_resolution_clock::now();
  auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(final - start);

  std::cout << "number of events read: " << numEvents * 1e-6 << " Mev in "
            << total_duration.count() * 1e-6 << " seconds" << std::endl;
  std::cout << "event rate: " << static_cast<double>(numEvents) / total_duration.count() << " Mevs"
            << std::endl;
  return (0);
}
