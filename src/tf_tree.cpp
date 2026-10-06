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

#include "tf_tree_terminal_cpp/tf_tree.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace tf_tree_terminal_cpp
{

const char * const kBanner =
  R"(
░▀█▀░█▀▀░░░░░▀█▀░█▀▄░█▀▀░█▀▀
░░█░░█▀▀░▄▄▄░░█░░█▀▄░█▀▀░█▀▀
░░▀░░▀░░░░░░░░▀░░▀░▀░▀▀▀░▀▀▀
TF-TREE CLI DEBUGGER
)";

namespace
{

struct Profile
{
  std::vector<std::string> required;
  std::string description;
};

const std::map<std::string, Profile> & profiles()
{
  static const std::map<std::string, Profile> kProfiles{
    {"mobile", {{"map", "odom", "base_link"}, "REP 105 (Mobile)"}},
    {"arm", {{"base_link", "tool0"}, "REP 199 (Articulated)"}},
    {"auto", {{"base_link"}, "Auto-Detection"}},
  };
  return kProfiles;
}

std::string fixed(double value, int decimals)
{
  char text[64];
  std::snprintf(text, sizeof(text), "%.*f", decimals, value);
  return text;
}

std::string join(const std::set<std::string> & items)
{
  std::string out;
  for (const auto & item : items) {
    out += (out.empty() ? "" : ", ") + item;
  }
  return out;
}

class Renderer
{
public:
  Renderer(const Snapshot & snapshot, const RenderOptions & options)
  : snapshot_(snapshot), options_(options)
  {
    for (const auto & [child, frame] : snapshot_.frames) {
      children_[frame.parent].push_back(child);
      all_frames_.insert(child);
      all_frames_.insert(frame.parent);
    }
  }

  std::string run()
  {
    if (snapshot_.frames.empty()) {
      return "\n✖ No TF tree detected.\n";
    }

    out_ << "\n--- TF SNAPSHOT: " << snapshot_.time << " ---\n";
    const auto roots = find_roots(snapshot_);
    if (roots.size() > 1) {
      out_ << "⚠ ALERT: " << roots.size() << " DISJOINTED TREES!\n";
      out_ << "└─ Roots: " << join(std::set<std::string>(roots.begin(), roots.end())) << "\n\n";
    }
    for (const auto & root : roots) {
      frame(root, "", true, true);
    }

    if (!options_.light) {
      diagnostic();
      rate_summary();
    }
    return out_.str();
  }

private:
  std::string grey(const std::string & text) const
  {
    return options_.color ? "\033[90m" + text + "\033[0m" : text;
  }

  void frame(const std::string & name, const std::string & indent, bool is_last, bool is_root)
  {
    const auto found = snapshot_.frames.find(name);
    const Frame * info = found == snapshot_.frames.end() ? nullptr : &found->second;
    const bool loop = !visited_.insert(name).second;

    const double rate = info != nullptr && !info->is_static ? info->rate_hz : 0.0;
    // a frame without a parent has no transform of its own, a single stamp gives no rate
    const std::string rate_label = info == nullptr ? "ROOT" :
      (info->is_static ? "STATIC" : (rate > 0.0 ? fixed(rate, 1) + " Hz" : "? Hz"));
    const std::string marker = is_root ? "" : (is_last ? "└── " : "├── ");
    const std::string child_indent = indent + (is_root ? "" : (is_last ? "    " : "│   "));

    if (options_.light) {
      out_ << indent << marker << name << " (" << grey(rate_label) << ")";
    } else {
      out_ << indent << marker << "⚯ Link: " << name << " [" << grey(rate_label) << "]";
      if (info != nullptr && info->age_ms) {
        out_ << " " << grey(age_label(*info->age_ms));
      }
    }
    if (info != nullptr && info->conflicting_parents) {
      out_ << " ⚠ several parents";
    }
    if (info != nullptr && info->publishers.size() > 1) {
      out_ << " ⚠ " << info->publishers.size() << " publishers";
    }
    if (loop) {
      out_ << " ⚠ loop\n";
      return;
    }
    out_ << "\n";

    if (info != nullptr) {
      if (rate > 0.0) {
        rates_.emplace_back(rate, name);
      }
      if (!options_.light) {
        out_ << child_indent << "⚙  Joint: to_" << name << " [TF: " <<
          grey(info->publishers.empty() ? "Unknown" : join(info->publishers)) << "]\n";
      }
    }

    const auto kids = children_.find(name);
    if (kids == children_.end()) {
      return;
    }
    for (std::size_t i = 0; i < kids->second.size(); ++i) {
      frame(kids->second[i], child_indent, i + 1 == kids->second.size(), false);
    }
  }

  static std::string age_label(double age_ms)
  {
    if (age_ms < 0.0) {
      // a stamp ahead of the clock: clocks or the stamp source are off
      return "[FUTURE: " + fixed(-age_ms, 1) + "ms]";
    }
    if (age_ms < kStaleAfterMs) {
      return "[LIVE: " + fixed(age_ms, 1) + "ms]";
    }
    return "[STALE: " + fixed(age_ms / 1000.0, 1) + "s]";
  }

  void diagnostic()
  {
    const auto & js = snapshot_.joint_state_publishers;
    out_ << (js.empty() ? "✖" : "✔") << " /joint_states topic";
    if (!js.empty()) {
      out_ << " [" << grey(join(js)) << "]";
    }
    out_ << "\n\n--- COMPLIANCE DIAGNOSTIC ---\n";
    for (const auto & required : required_frames(options_.profile)) {
      out_ << (all_frames_.count(required) ? "✔" : "✖") << " Recommendation: " << required << "\n";
    }
  }

  void rate_summary()
  {
    if (rates_.empty()) {
      return;
    }
    const auto [low, high] = std::minmax_element(rates_.begin(), rates_.end());
    out_ << "\n--- TF RATE SUMMARY ---\n";
    out_ << "Min TF rate : " << grey(fixed(low->first, 1) + " Hz") << " (" << low->second <<
      ")\n";
    out_ << "Max TF rate : " << grey(fixed(high->first, 1) + " Hz") << " (" << high->second <<
      ")\n";
  }

  const Snapshot & snapshot_;
  const RenderOptions & options_;
  // std::map and the sorted snapshot keep the children in alphabetical order
  std::map<std::string, std::vector<std::string>> children_;
  std::set<std::string> all_frames_;
  std::set<std::string> visited_;
  std::vector<std::pair<double, std::string>> rates_;
  std::ostringstream out_;
};

}  // namespace

bool is_valid_profile(const std::string & profile)
{
  return profiles().count(profile) > 0;
}

std::string profile_description(const std::string & profile)
{
  return profiles().at(profile).description;
}

std::vector<std::string> required_frames(const std::string & profile)
{
  return profiles().at(profile).required;
}

std::vector<std::string> find_roots(const Snapshot & snapshot)
{
  std::set<std::string> roots;
  for (const auto & entry : snapshot.frames) {
    if (snapshot.frames.count(entry.second.parent) == 0) {
      roots.insert(entry.second.parent);
    }
  }

  // frames that hang in a loop are reachable from no root, enter each loop once
  std::map<std::string, std::vector<std::string>> children;
  for (const auto & [child, frame] : snapshot.frames) {
    children[frame.parent].push_back(child);
  }
  std::set<std::string> reached;
  std::vector<std::string> open(roots.begin(), roots.end());
  const auto flood = [&]() {
      while (!open.empty()) {
        const std::string name = open.back();
        open.pop_back();
        if (!reached.insert(name).second) {
          continue;
        }
        const auto kids = children.find(name);
        if (kids != children.end()) {
          open.insert(open.end(), kids->second.begin(), kids->second.end());
        }
      }
    };
  flood();
  for (const auto & entry : snapshot.frames) {
    if (reached.count(entry.first) == 0) {
      roots.insert(entry.first);
      open.push_back(entry.first);
      flood();
    }
  }
  return {roots.begin(), roots.end()};
}

std::string render(const Snapshot & snapshot, const RenderOptions & options)
{
  return Renderer(snapshot, options).run();
}

std::string strip_ansi(const std::string & text)
{
  std::string out;
  out.reserve(text.size());
  for (std::size_t i = 0; i < text.size(); ++i) {
    if (text[i] == '\033' && i + 1 < text.size() && text[i + 1] == '[') {
      std::size_t end = i + 2;
      while (end < text.size() && (std::isdigit(static_cast<unsigned char>(text[end])) ||
        text[end] == ';'))
      {
        ++end;
      }
      if (end < text.size() && text[end] == 'm') {
        i = end;
        continue;
      }
    }
    out += text[i];
  }
  return out;
}

}  // namespace tf_tree_terminal_cpp
