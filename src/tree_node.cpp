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

#include "tf_tree_terminal_cpp/tree_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace tf_tree_terminal_cpp
{

namespace
{

/// Stamps older than this, measured from the newest one, do not count for the rate.
constexpr double kRateWindowS = 10.0;

std::string wall_time()
{
  const std::time_t now = std::time(nullptr);
  std::tm local{};
  localtime_r(&now, &local);
  char text[32];
  std::strftime(text, sizeof(text), "%Y-%m-%d %H:%M:%S", &local);
  return text;
}

}  // namespace

TfTreeNode::TfTreeNode(const CliOptions & cli, const rclcpp::NodeOptions & options)
: rclcpp::Node("tf_tree_cli_helper", options), cli_(cli)
{
  using tf2_msgs::msg::TFMessage;
  const auto subscribe =
    [this](const std::string & topic, const rclcpp::QoS & qos, bool is_static) {
      return create_subscription<TFMessage>(
        topic, qos,
        [this, is_static](TFMessage::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
          Gid gid{};
          const auto & raw = info.get_rmw_message_info().publisher_gid;
          std::copy(std::begin(raw.data), std::end(raw.data), gid.begin());
          ingest(*message, is_static, gid);
        });
    };
  tf_sub_ = subscribe("/tf", rclcpp::QoS(100), false);
  tf_static_sub_ = subscribe("/tf_static", rclcpp::QoS(100).transient_local(), true);

  if (!cli_.clear_screen) {
    std::cout << kBanner << std::endl;
    RCLCPP_INFO(get_logger(), "Mode: %s", profile_description(cli_.profile).c_str());
  }
  if (cli_.keep_alive) {
    if (!cli_.clear_screen) {
      RCLCPP_INFO(get_logger(), "Alive mode: Updating every 5s. Ctrl+C to stop.");
    }
    timer_ = create_wall_timer(std::chrono::seconds(5), std::bind(&TfTreeNode::on_timer, this));
  } else {
    RCLCPP_INFO(get_logger(), "Single-shot mode: Buffering (3s)...");
    timer_ = create_wall_timer(std::chrono::seconds(3), std::bind(&TfTreeNode::on_timer, this));
  }
}

void TfTreeNode::ingest(
  const tf2_msgs::msg::TFMessage & message, bool is_static, const Gid & publisher)
{
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  ingest(message, is_static, publisher, std::chrono::duration<double>(now).count());
}

void TfTreeNode::ingest(
  const tf2_msgs::msg::TFMessage & message, bool is_static, const Gid & publisher,
  double received)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & transform : message.transforms) {
    Edge & edge = edges_[transform.child_frame_id];
    if (!edge.parent.empty() && edge.parent != transform.header.frame_id) {
      edge.conflicting_parents = true;
    }
    edge.parent = transform.header.frame_id;
    if (edge.is_static != is_static) {
      edge.publishers.clear();
    }
    edge.is_static = is_static;
    if (is_static) {
      edge.stamps.clear();
      edge.publishers[publisher] = received;
      continue;
    }
    const double stamp = rclcpp::Time(transform.header.stamp).seconds();
    edge.stamps.push_back(stamp);
    while (edge.stamps.front() < stamp - kRateWindowS || edge.stamps.front() > stamp) {
      // also drops everything after a jump back in time, e.g. a bag that loops
      edge.stamps.pop_front();
    }
    // a publisher that went quiet for the length of the window no longer counts. This goes by
    // the time of reception, as rival publishers need not agree on their stamps.
    edge.publishers[publisher] = received;
    const double oldest = received - kRateWindowS;
    for (auto it = edge.publishers.begin(); it != edge.publishers.end(); ) {
      it = it->second < oldest ? edge.publishers.erase(it) : std::next(it);
    }
  }
}

Snapshot TfTreeNode::snapshot()
{
  Snapshot result;
  result.time = wall_time();
  for (const auto & info : get_publishers_info_by_topic("/joint_states")) {
    result.joint_state_publishers.insert(info.node_name());
  }

  const auto tf_publishers = get_publishers_info_by_topic("/tf");
  const auto tf_static_publishers = get_publishers_info_by_topic("/tf_static");

  const double now = get_clock()->now().seconds();
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & [child, edge] : edges_) {
    Frame & frame = result.frames[child];
    frame.parent = edge.parent;
    frame.is_static = edge.is_static;
    frame.conflicting_parents = edge.conflicting_parents;
    frame.publishers = publisher_names(
      edge.is_static ? tf_static_publishers : tf_publishers, edge.publishers);
    if (edge.is_static || edge.stamps.empty()) {
      continue;
    }
    frame.age_ms = (now - edge.stamps.back()) * 1e3;
    const double span = edge.stamps.back() - edge.stamps.front();
    if (span > 0.0) {
      frame.rate_hz = static_cast<double>(edge.stamps.size() - 1) / span;
    }
  }
  return result;
}

std::set<std::string> TfTreeNode::publisher_names(
  const std::vector<rclcpp::TopicEndpointInfo> & on_topic, const std::map<Gid, double> & seen)
{
  std::set<std::string> names;
  std::size_t unknown = 0;
  for (const auto & entry : seen) {
    const auto match = std::find_if(
      on_topic.begin(), on_topic.end(),
      [&](const rclcpp::TopicEndpointInfo & info) {return info.endpoint_gid() == entry.first;});
    if (match != on_topic.end()) {
      names.insert(match->node_name());
    } else {
      ++unknown;
    }
  }
  if (names.empty() && unknown == 1 && on_topic.size() == 1) {
    // the middleware did not tell who sent it, but there is only one candidate
    names.insert(on_topic.front().node_name());
  } else {
    // publishers that are gone by now, or that the middleware does not identify
    for (std::size_t i = 0; i < unknown; ++i) {
      names.insert(unknown == 1 ? "Unknown" : "Unknown " + std::to_string(i + 1));
    }
  }
  return names;
}

std::string TfTreeNode::analyse()
{
  RenderOptions options;
  options.profile = cli_.profile;
  options.light = cli_.light;
  options.color = cli_.color;
  return render(snapshot(), options);
}

bool TfTreeNode::save_to_file(const std::string & content) const
{
  std::ofstream file(cli_.output_file);
  file << strip_ansi(content) << "\nGenerated on: " << wall_time() << "\n";
  return static_cast<bool>(file.flush());
}

void TfTreeNode::on_timer()
{
  if (cli_.clear_screen) {
    std::cout << "\033[2J\033[H" << kBanner << std::endl;
  }

  const std::string text = analyse();
  std::cout << text << std::endl;
  if (!cli_.output_file.empty()) {
    if (save_to_file(text)) {
      std::cout << "[INFO] Saved to: " << cli_.output_file << std::endl;
    } else {
      std::cout << "[ERROR] Save failed: " << cli_.output_file << std::endl;
    }
  }

  if (!cli_.keep_alive) {
    RCLCPP_INFO(get_logger(), "Analysis complete. Shutting down...");
    timer_->cancel();
    rclcpp::shutdown();
  }
}

}  // namespace tf_tree_terminal_cpp
