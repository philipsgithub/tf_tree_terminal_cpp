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

#include <string>

#include "gtest/gtest.h"

#include "tf_tree_terminal_cpp/tf_tree.hpp"

using tf_tree_terminal_cpp::Frame;
using tf_tree_terminal_cpp::RenderOptions;
using tf_tree_terminal_cpp::Snapshot;
using tf_tree_terminal_cpp::render;

namespace
{

Frame dynamic(const std::string & parent, double rate_hz, double age_ms)
{
  Frame frame;
  frame.parent = parent;
  frame.rate_hz = rate_hz;
  frame.age_ms = age_ms;
  frame.publishers = {"state_publisher"};
  return frame;
}

Frame fixed(const std::string & parent)
{
  Frame frame;
  frame.parent = parent;
  frame.is_static = true;
  frame.publishers = {"robot_state_publisher"};
  return frame;
}

Snapshot robot()
{
  Snapshot snapshot;
  snapshot.time = "2026-01-03 23:30:12";
  snapshot.frames["base_footprint"] = fixed("base_link");
  snapshot.frames["camera_link"] = fixed("base_link");
  snapshot.frames["camera_link_optical"] = fixed("camera_link");
  snapshot.frames["left_wheel"] = dynamic("base_link", 12.3, 81.5);
  snapshot.frames["right_wheel"] = dynamic("base_link", 50.0, 700.0);
  return snapshot;
}

RenderOptions plain(bool light = false, const std::string & profile = "auto")
{
  RenderOptions options;
  options.color = false;
  options.light = light;
  options.profile = profile;
  return options;
}

}  // namespace

TEST(TfTree, light_tree_structure)
{
  EXPECT_EQ(
    render(robot(), plain(true)),
    "\n--- TF SNAPSHOT: 2026-01-03 23:30:12 ---\n"
    "base_link (ROOT)\n"
    "├── base_footprint (STATIC)\n"
    "├── camera_link (STATIC)\n"
    "│   └── camera_link_optical (STATIC)\n"
    "├── left_wheel (12.3 Hz)\n"
    "└── right_wheel (50.0 Hz)\n");
}

TEST(TfTree, full_output)
{
  Snapshot snapshot = robot();
  snapshot.joint_state_publishers = {"joint_state_publisher"};
  EXPECT_EQ(
    render(snapshot, plain(false, "mobile")),
    "\n--- TF SNAPSHOT: 2026-01-03 23:30:12 ---\n"
    "⚯ Link: base_link [ROOT]\n"
    "├── ⚯ Link: base_footprint [STATIC]\n"
    "│   ⚙  Joint: to_base_footprint [TF: robot_state_publisher]\n"
    "├── ⚯ Link: camera_link [STATIC]\n"
    "│   ⚙  Joint: to_camera_link [TF: robot_state_publisher]\n"
    "│   └── ⚯ Link: camera_link_optical [STATIC]\n"
    "│       ⚙  Joint: to_camera_link_optical [TF: robot_state_publisher]\n"
    "├── ⚯ Link: left_wheel [12.3 Hz] [LIVE: 81.5ms]\n"
    "│   ⚙  Joint: to_left_wheel [TF: state_publisher]\n"
    "└── ⚯ Link: right_wheel [50.0 Hz] [STALE: 0.7s]\n"
    "    ⚙  Joint: to_right_wheel [TF: state_publisher]\n"
    "✔ /joint_states topic [joint_state_publisher]\n"
    "\n--- COMPLIANCE DIAGNOSTIC ---\n"
    "✖ Recommendation: map\n"
    "✖ Recommendation: odom\n"
    "✔ Recommendation: base_link\n"
    "\n--- TF RATE SUMMARY ---\n"
    "Min TF rate : 12.3 Hz (left_wheel)\n"
    "Max TF rate : 50.0 Hz (right_wheel)\n");
}

TEST(TfTree, empty)
{
  EXPECT_EQ(render(Snapshot(), plain()), "\n✖ No TF tree detected.\n");
}

TEST(TfTree, disjointed_trees)
{
  Snapshot snapshot = robot();
  snapshot.frames["test_orphan"] = fixed("elsewhere");
  const std::string text = render(snapshot, plain(true));
  EXPECT_NE(
    text.find("⚠ ALERT: 2 DISJOINTED TREES!\n└─ Roots: base_link, elsewhere\n"),
    std::string::npos);
  EXPECT_NE(text.find("elsewhere (ROOT)\n└── test_orphan (STATIC)\n"), std::string::npos);
}

TEST(TfTree, stamp_ahead_of_clock)
{
  Snapshot snapshot;
  snapshot.frames["hand"] = dynamic("body", 50.0, -16.04);
  EXPECT_NE(render(snapshot, plain()).find("[FUTURE: 16.0ms]"), std::string::npos);
}

TEST(TfTree, unknown_rate_and_missing_joint_states)
{
  Snapshot snapshot;
  snapshot.frames["hand"] = dynamic("body", 0.0, 5.0);
  const std::string text = render(snapshot, plain());
  EXPECT_NE(text.find("hand [? Hz]"), std::string::npos);
  EXPECT_NE(text.find("✖ /joint_states topic\n"), std::string::npos);
  EXPECT_EQ(text.find("TF RATE SUMMARY"), std::string::npos);
}

TEST(TfTree, loop_terminates)
{
  Snapshot snapshot;
  snapshot.frames["a"] = fixed("b");
  snapshot.frames["b"] = fixed("a");
  const std::string text = render(snapshot, plain(true));
  EXPECT_NE(text.find("⚠ loop"), std::string::npos);
  EXPECT_EQ(tf_tree_terminal_cpp::find_roots(snapshot).size(), 1u);
}

TEST(TfTree, conflicting_parents)
{
  Snapshot snapshot = robot();
  snapshot.frames["left_wheel"].conflicting_parents = true;
  const std::string text = render(snapshot, plain(true));
  EXPECT_NE(text.find("left_wheel (12.3 Hz) ⚠ several parents"), std::string::npos);
}

TEST(TfTree, several_publishers)
{
  Snapshot snapshot = robot();
  snapshot.frames["left_wheel"].publishers.insert("ekf_node");
  const std::string text = render(snapshot, plain());
  EXPECT_NE(
    text.find(
      "├── ⚯ Link: left_wheel [12.3 Hz] [LIVE: 81.5ms] ⚠ 2 publishers\n"
      "│   ⚙  Joint: to_left_wheel [TF: ekf_node, state_publisher]\n"),
    std::string::npos) << text;
  EXPECT_NE(
    render(snapshot, plain(true)).find("left_wheel (12.3 Hz) ⚠ 2 publishers"),
    std::string::npos);
}

TEST(TfTree, colour_can_be_stripped)
{
  RenderOptions options;
  options.light = true;
  const std::string coloured = render(robot(), options);
  EXPECT_NE(coloured.find("\033[90m"), std::string::npos);
  EXPECT_EQ(tf_tree_terminal_cpp::strip_ansi(coloured), render(robot(), plain(true)));
}

TEST(TfTree, profiles)
{
  EXPECT_TRUE(tf_tree_terminal_cpp::is_valid_profile("arm"));
  EXPECT_FALSE(tf_tree_terminal_cpp::is_valid_profile("boat"));
  EXPECT_EQ(tf_tree_terminal_cpp::profile_description("mobile"), "REP 105 (Mobile)");
  EXPECT_EQ(tf_tree_terminal_cpp::required_frames("arm").back(), "tool0");
}
