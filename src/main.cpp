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

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "tf_tree_terminal_cpp/tree_node.hpp"

namespace
{

const char * const kUsage =
  R"(usage: tf-tree [-h] [-p {mobile,arm,auto}] [-s FILE] [-a] [-l] [-c] [-nc] [--ros-args ...]

ROS2 TF Tree CLI Helper

options:
  -h, --help            show this help message and exit
  -p, --profile {mobile,arm,auto}
                        Diagnostic mode
  -s, --save FILE       Export the analysis to a file
  -a, --alive           Keep node alive; refreshes terminal every 5 seconds.
  -l, --light           Light mode: display only the TF tree structure
  -c, --clear           Clear terminal screen before each refresh in alive mode
  -nc, --no-color       Disable ANSI colors in output

Ages are measured against the node's clock. For a bag played with --clock add
  --ros-args -p use_sim_time:=true
)";

}  // namespace

int main(int argc, char ** argv)
{
  const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);

  tf_tree_terminal_cpp::CliOptions cli;
  for (std::size_t i = 1; i < args.size(); ++i) {
    const std::string & arg = args[i];
    const auto value = [&]() -> std::string {
        if (i + 1 >= args.size()) {
          std::cerr << "tf-tree: " << arg << " needs a value\n" << kUsage;
          std::exit(2);
        }
        return args[++i];
      };
    if (arg == "-h" || arg == "--help") {
      std::cout << kUsage;
      return 0;
    } else if (arg == "-p" || arg == "--profile") {
      cli.profile = value();
      if (!tf_tree_terminal_cpp::is_valid_profile(cli.profile)) {
        std::cerr << "tf-tree: unknown profile '" << cli.profile << "'\n" << kUsage;
        return 2;
      }
    } else if (arg == "-s" || arg == "--save") {
      cli.output_file = value();
    } else if (arg == "-a" || arg == "--alive") {
      cli.keep_alive = true;
    } else if (arg == "-l" || arg == "--light") {
      cli.light = true;
    } else if (arg == "-c" || arg == "--clear") {
      cli.clear_screen = true;
    } else if (arg == "-nc" || arg == "--no-color") {
      cli.color = false;
    } else {
      std::cerr << "tf-tree: unknown argument '" << arg << "'\n" << kUsage;
      return 2;
    }
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tf_tree_terminal_cpp::TfTreeNode>(cli));
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
