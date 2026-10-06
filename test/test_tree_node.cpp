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

#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "tf_tree_terminal_cpp/tree_node.hpp"

using tf_tree_terminal_cpp::CliOptions;
using tf_tree_terminal_cpp::TfTreeNode;

namespace
{

tf2_msgs::msg::TFMessage transform(
  const std::string & parent, const std::string & child, const rclcpp::Time & stamp)
{
  tf2_msgs::msg::TFMessage message;
  message.transforms.resize(1);
  message.transforms[0].header.frame_id = parent;
  message.transforms[0].header.stamp = stamp;
  message.transforms[0].child_frame_id = child;
  message.transforms[0].transform.rotation.w = 1.0;
  return message;
}

class TreeNode : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  static CliOptions cli(const std::string & profile = "auto", bool light = true)
  {
    CliOptions options;
    options.profile = profile;
    options.light = light;
    options.color = false;
    return options;
  }
};

}  // namespace

TEST_F(TreeNode, rate_and_age_from_stamps)
{
  TfTreeNode node(cli());
  const rclcpp::Time now = node.get_clock()->now();
  // 50 Hz for 2 s, the newest stamp 40 ms old
  for (int i = 100; i >= 0; --i) {
    const rclcpp::Time stamp = now - rclcpp::Duration::from_seconds(0.04 + 0.02 * i);
    node.ingest(transform("odom", "base_link", stamp), false, {});
  }
  node.ingest(transform("base_link", "camera", rclcpp::Time(0, 0, RCL_ROS_TIME)), true, {});

  const auto snapshot = node.snapshot();
  ASSERT_EQ(snapshot.frames.size(), 2u);
  const auto & base = snapshot.frames.at("base_link");
  EXPECT_EQ(base.parent, "odom");
  EXPECT_FALSE(base.is_static);
  EXPECT_NEAR(base.rate_hz, 50.0, 0.01);
  ASSERT_TRUE(base.age_ms.has_value());
  EXPECT_NEAR(*base.age_ms, 40.0, 20.0);
  EXPECT_TRUE(snapshot.frames.at("camera").is_static);
  EXPECT_FALSE(snapshot.frames.at("camera").age_ms.has_value());
}

TEST_F(TreeNode, rate_uses_only_the_last_seconds)
{
  TfTreeNode node(cli());
  const rclcpp::Time start = node.get_clock()->now() - rclcpp::Duration::from_seconds(60.0);
  // 1 Hz for 30 s, then 10 Hz for 30 s
  for (int i = 0; i < 30; ++i) {
    node.ingest(transform("a", "b", start + rclcpp::Duration::from_seconds(i)), false, {});
  }
  for (int i = 0; i <= 300; ++i) {
    const rclcpp::Time stamp = start + rclcpp::Duration::from_seconds(30.0 + 0.1 * i);
    node.ingest(transform("a", "b", stamp), false, {});
  }
  EXPECT_NEAR(node.snapshot().frames.at("b").rate_hz, 10.0, 0.01);
}

TEST_F(TreeNode, conflicting_parents_are_flagged)
{
  TfTreeNode node(cli());
  const rclcpp::Time now = node.get_clock()->now();
  node.ingest(transform("a", "c", now), false, {});
  node.ingest(transform("b", "c", now), false, {});
  EXPECT_TRUE(node.snapshot().frames.at("c").conflicting_parents);
}

TEST_F(TreeNode, receives_published_transforms)
{
  auto node = std::make_shared<TfTreeNode>(cli());
  auto talker = std::make_shared<rclcpp::Node>("talker");
  auto tf_pub = talker->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
  auto static_pub = talker->create_publisher<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(100).transient_local());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(talker);
  static_pub->publish(transform("base_link", "camera", rclcpp::Time(0, 0, RCL_ROS_TIME)));
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < deadline && node->snapshot().frames.size() < 2) {
    tf_pub->publish(transform("map", "base_link", talker->get_clock()->now()));
    executor.spin_some(std::chrono::milliseconds(20));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  const std::string text = node->analyse();
  EXPECT_NE(text.find("map (ROOT)"), std::string::npos) << text;
  EXPECT_NE(text.find("└── base_link ("), std::string::npos) << text;
  EXPECT_NE(text.find("    └── camera (STATIC)"), std::string::npos) << text;
  EXPECT_EQ(node->snapshot().frames.at("base_link").publishers, std::set<std::string>{"talker"});

  // a second node publishing the same transform
  auto rival = std::make_shared<rclcpp::Node>("rival");
  auto rival_pub = rival->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
  executor.add_node(rival);
  const auto both = std::set<std::string>{"rival", "talker"};
  const auto rival_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < rival_deadline &&
    node->snapshot().frames.at("base_link").publishers != both)
  {
    tf_pub->publish(transform("map", "base_link", talker->get_clock()->now()));
    rival_pub->publish(transform("map", "base_link", rival->get_clock()->now()));
    executor.spin_some(std::chrono::milliseconds(20));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_EQ(node->snapshot().frames.at("base_link").publishers, both);
  EXPECT_NE(node->analyse().find("⚠ 2 publishers"), std::string::npos);
}

TEST_F(TreeNode, quiet_publisher_is_dropped)
{
  TfTreeNode node(cli());
  TfTreeNode::Gid first{}, second{};
  first[0] = 1;
  second[0] = 2;
  const rclcpp::Time stamp = node.get_clock()->now();
  node.ingest(transform("a", "b", stamp), false, first, 100.0);
  node.ingest(transform("a", "b", stamp), false, second, 101.0);
  EXPECT_EQ(node.snapshot().frames.at("b").publishers.size(), 2u);
  node.ingest(transform("a", "b", stamp), false, second, 120.0);
  EXPECT_EQ(node.snapshot().frames.at("b").publishers, std::set<std::string>{"Unknown"});
}

TEST_F(TreeNode, compliance_of_the_mobile_profile)
{
  TfTreeNode node(cli("mobile", false));
  node.ingest(transform("map", "base_link", node.get_clock()->now()), false, {});
  const std::string text = node.analyse();
  EXPECT_NE(text.find("--- COMPLIANCE DIAGNOSTIC ---"), std::string::npos);
  EXPECT_NE(text.find("✔ Recommendation: map"), std::string::npos);
  EXPECT_NE(text.find("✖ Recommendation: odom"), std::string::npos);
}

TEST_F(TreeNode, save_to_file)
{
  CliOptions options = cli();
  options.output_file = ::testing::TempDir() + "tf_report.txt";
  TfTreeNode node(options);
  ASSERT_TRUE(node.save_to_file("\033[90mTEST_TF_DATA_123\033[0m"));

  std::ifstream file(options.output_file);
  std::stringstream saved;
  saved << file.rdbuf();
  EXPECT_EQ(saved.str().rfind("TEST_TF_DATA_123\nGenerated on: ", 0), 0u) << saved.str();
  std::remove(options.output_file.c_str());

  options.output_file = "/nonexistent_dir/tf_report.txt";
  EXPECT_FALSE(TfTreeNode(options).save_to_file("x"));
}
