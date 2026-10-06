// Copyright (c) 2025 Tanneguy de Villemagne
// Copyright (c) 2026 Philip Keller
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef TF_TREE_TERMINAL_CPP__TREE_NODE_HPP_
#define TF_TREE_TERMINAL_CPP__TREE_NODE_HPP_

#include <array>
#include <deque>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rmw/types.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "tf_tree_terminal_cpp/tf_tree.hpp"

namespace tf_tree_terminal_cpp
{

struct CliOptions
{
  std::string profile{"auto"};
  /// Write each analysis to this file, empty for none.
  std::string output_file;
  /// Keep running and print every 5 s instead of once after 3 s.
  bool keep_alive{false};
  bool light{false};
  bool clear_screen{false};
  bool color{true};
};

/// Listens to /tf and /tf_static and prints the tree.
class TfTreeNode : public rclcpp::Node
{
public:
  using Gid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;

  explicit TfTreeNode(
    const CliOptions & cli,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Take in one TF message. Called by the subscriptions.
  void ingest(const tf2_msgs::msg::TFMessage & message, bool is_static, const Gid & publisher);
  /// The same with the time of reception given, in seconds of any steady clock.
  void ingest(
    const tf2_msgs::msg::TFMessage & message, bool is_static, const Gid & publisher,
    double received);

  /// The current state of the tree.
  Snapshot snapshot();

  /// Text of the analysis for the current state.
  std::string analyse();

  /// Write the text without colours, with the time it was generated. False if the file cannot
  /// be written.
  bool save_to_file(const std::string & content) const;

private:
  struct Edge
  {
    std::string parent;
    bool is_static{false};
    bool conflicting_parents{false};
    /// Who published this edge and when the last message of each was received.
    std::map<Gid, double> publishers;
    /// Stamps of the last seconds, in seconds, newest at the back.
    std::deque<double> stamps;
  };

  void on_timer();
  static std::set<std::string> publisher_names(
    const std::vector<rclcpp::TopicEndpointInfo> & on_topic, const std::map<Gid, double> & seen);

  CliOptions cli_;
  std::mutex mutex_;
  std::map<std::string, Edge> edges_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tf_tree_terminal_cpp

#endif  // TF_TREE_TERMINAL_CPP__TREE_NODE_HPP_
