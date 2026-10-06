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

#ifndef TF_TREE_TERMINAL_CPP__TF_TREE_HPP_
#define TF_TREE_TERMINAL_CPP__TF_TREE_HPP_

#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace tf_tree_terminal_cpp
{

/// Banner printed above the analysis.
extern const char * const kBanner;

/// What is known about one frame that has a parent, i.e. one edge of the tree.
struct Frame
{
  std::string parent;
  bool is_static{false};
  /// Publish rate from the stamps seen so far, 0 if unknown or static.
  double rate_hz{0.0};
  /// Clock time minus newest stamp. Empty for static frames.
  std::optional<double> age_ms;
  /// Nodes publishing this transform. More than one is usually a mistake.
  std::set<std::string> publishers;
  /// More than one parent was published for this frame.
  bool conflicting_parents{false};
};

/// Everything the renderer needs, keyed by child frame id.
struct Snapshot
{
  std::map<std::string, Frame> frames;
  std::set<std::string> joint_state_publishers;
  /// Wall clock time of the snapshot, already formatted.
  std::string time;
};

struct RenderOptions
{
  /// "mobile", "arm" or "auto"
  std::string profile{"auto"};
  /// Only the tree structure with rates.
  bool light{false};
  bool color{true};
};

/// Transforms older than this are reported as stale.
constexpr double kStaleAfterMs = 500.0;

bool is_valid_profile(const std::string & profile);
std::string profile_description(const std::string & profile);
/// Frames the profile expects to exist.
std::vector<std::string> required_frames(const std::string & profile);

/// Frames without a parent, sorted. A loop without any root is entered at its first frame.
std::vector<std::string> find_roots(const Snapshot & snapshot);

/// The full text for one snapshot.
std::string render(const Snapshot & snapshot, const RenderOptions & options);

/// Remove ANSI colour sequences, for writing to a file.
std::string strip_ansi(const std::string & text);

}  // namespace tf_tree_terminal_cpp

#endif  // TF_TREE_TERMINAL_CPP__TF_TREE_HPP_
