### ROS2 CI

[![Humble Status](https://github.com/philipsgithub/tf_tree_terminal_cpp/actions/workflows/ros2_ci_humble.yml/badge.svg?branch=master)](https://github.com/philipsgithub/tf_tree_terminal_cpp/actions/workflows/ros2_ci_humble.yml)
[![Jazzy Status](https://github.com/philipsgithub/tf_tree_terminal_cpp/actions/workflows/ros2_ci_jazzy.yml/badge.svg?branch=master)](https://github.com/philipsgithub/tf_tree_terminal_cpp/actions/workflows/ros2_ci_jazzy.yml)
[![Kilted Status](https://github.com/philipsgithub/tf_tree_terminal_cpp/actions/workflows/ros2_ci_kilted.yml/badge.svg?branch=master)](https://github.com/philipsgithub/tf_tree_terminal_cpp/actions/workflows/ros2_ci_kilted.yml)
[![Rolling Status](https://github.com/philipsgithub/tf_tree_terminal_cpp/actions/workflows/ros2_ci_rolling.yml/badge.svg?branch=master)](https://github.com/philipsgithub/tf_tree_terminal_cpp/actions/workflows/ros2_ci_rolling.yml)

# TF-Tree CLI Debugger

```text
░▀█▀░█▀▀░░░░░▀█▀░█▀▄░█▀▀░█▀▀
░░█░░█▀▀░▄▄▄░░█░░█▀▄░█▀▀░█▀▀
░░▀░░▀░░░░░░░░▀░░▀░▀░▀▀▀░▀▀▀
      TF-TREE CLI DEBUGGER
```

C++ port of [tf_tree_terminal](https://github.com/Tanneguydv/tf_tree_terminal) by Tanneguy de Villemagne.

A lightweight ROS 2 utility for auditing, visualizing, and validating your Coordinate Transform (TF) tree. This tool tries to go beyond simple visualization by providing deep-source auditing and standard compliance checking (REP 105/199).

## 🚀 Key Features

* **Deep Source Auditing**: Shows for every link the node that publishes its transform, and which nodes publish `/joint_states`.
* **Live vs. Static Analysis**: Automatically distinguishes between truly `STATIC` latched transforms and `LIVE` high-frequency streams.
* **Diagnostic Profiles**: Built-in validation for specific robot types:
* **Mobile**: Validates the `map` → `odom` → `base_link` chain.
* **Arm**: Validates articulated chains from `base_link` to `tool0`.


* **Precision Latency**: Reports the "age" of transforms in milliseconds to detect jitter or stale data, and flags stamps that lie ahead of the clock.
* **Disjointed Tree Alerts**: Visually flags if your tree is broken into multiple disconnected components, if a frame is published with several parents or by several nodes, or if frames form a loop.

---

## 🛠 Usage

Once built and sourced, you can use the `tf-tree` command (or `ros2 run tf_tree_terminal_cpp show`):

### Command Flags

| Flag | Short | Description |
| --- | --- | --- |
| `--profile` | `-p` | Diagnostic mode: `mobile`, `arm`, or `auto` (default). |
| `--save` | `-s` | Export the analysis to a file (e.g., `-s robot_report.txt`). |
| `--alive` | `-a` | Keep node alive; refreshes terminal every 5 seconds. |
| `--light` | `-l` | Get a light TF-Tree as output |
| `--clear` | `-c` | Clear terminal screen before each refresh in alive mode |
| `--no-color` | `-nc` | Disable ANSI colors in output |
| `--help` | `-h` | Show help message and exit. |

### Examples

**Standard One-Shot Audit:**

```bash
tf-tree
```

**Save a mobile robot diagnostic to a file:**

```bash
tf-tree -p mobile -s my_robot.txt
```

**Live monitoring during a mission:**

```bash
tf-tree --alive
```

**A recorded bag:** ages are measured against the node's clock, so play the bag with `--clock` and use simulation time.

```bash
ros2 bag play my_bag --clock
tf-tree --ros-args -p use_sim_time:=true
```

#### Example outputs

running ```tf-tree -p mobile```

```text

░▀█▀░█▀▀░░░░░▀█▀░█▀▄░█▀▀░█▀▀
░░█░░█▀▀░▄▄▄░░█░░█▀▄░█▀▀░█▀▀
░░▀░░▀░░░░░░░░▀░░▀░▀░▀▀▀░▀▀▀
TF-TREE CLI DEBUGGER

[INFO] [1767483009.498731809] [tf_tree_cli_helper]: Mode: REP 105 (Mobile)
[INFO] [1767483009.499220178] [tf_tree_cli_helper]: Single-shot mode: Buffering (3s)...

--- TF SNAPSHOT: 2026-01-03 23:30:12 ---
⚯ Link: base_link [ROOT]
├── ⚯ Link: base_footprint [STATIC]
│   ⚙  Joint: to_base_footprint [TF: robot_state_publisher]
├── ⚯ Link: camera_link [STATIC]
│   ⚙  Joint: to_camera_link [TF: robot_state_publisher]
│   └── ⚯ Link: camera_link_optical [STATIC]
│       ⚙  Joint: to_camera_link_optical [TF: robot_state_publisher]
├── ⚯ Link: left_wheel [12.3 Hz] [LIVE: 81.5ms]
│   ⚙  Joint: to_left_wheel [TF: robot_state_publisher]
└── ⚯ Link: right_wheel [12.3 Hz] [LIVE: 81.6ms]
    ⚙  Joint: to_right_wheel [TF: robot_state_publisher]
✔ /joint_states topic [joint_state_publisher]

--- COMPLIANCE DIAGNOSTIC ---
✖ Recommendation: map
✖ Recommendation: odom
✔ Recommendation: base_link

--- TF RATE SUMMARY ---
Min TF rate : 12.3 Hz (left_wheel)
Max TF rate : 12.3 Hz (right_wheel)
```
and light mode, running ```tf-tree -l```

```text
--- TF SNAPSHOT: 2026-01-03 23:38:17 ---
base_link (ROOT)
├── base_footprint (STATIC)
├── camera_link (STATIC)
│   └── camera_link_optical (STATIC)
├── left_wheel (12.3 Hz)
└── right_wheel (12.3 Hz)
```

or broken, running ```tf-tree -l``` :

```text
--- TF SNAPSHOT: 2026-01-03 23:41:04 ---
⚠ ALERT: 2 DISJOINTED TREES!
└─ Roots: base_link, right_wheel

base_link (ROOT)
├── base_footprint (STATIC)
└── camera_link (STATIC)
    └── camera_link_optical (STATIC)
right_wheel (ROOT)
└── test_orphan (STATIC)
```

---

## 📊 Understanding the Output

* **`⚯ Link`**: The TF frame name with its frequency (Hz), `STATIC` for frames from `/tf_static`, `ROOT` for a frame without parent.
* **`[LIVE: 15ms]`**: Data is current. The age is the node's clock minus the newest stamp.
* **`[STALE: 2.1s]`**: The broadcaster has likely stopped or crashed.
* **`[FUTURE: 16ms]`**: The stamp lies ahead of the clock, so the clocks of publisher and listener disagree.
* **`⚙ Joint`**: The node(s) that published this transform within the last 10 seconds. `Unknown` stands for a publisher the middleware does not identify or that is gone.
* **`⚠ 2 publishers`**: Several nodes publish the same transform; all of them are listed on the `⚙ Joint` line.
* **`⚠ several parents`** / **`⚠ loop`**: The frame was published with more than one parent, or its chain leads back to itself.
* **`✔ Recommendation`**: Compliance status based on the selected profile.

---

## 🔧 Installation

1. **Clone to your workspace:**
```bash
cd ~/ros2_ws/src
git clone https://github.com/philipsgithub/tf_tree_terminal_cpp.git
```


2. **Build:**
```bash
cd ~/ros2_ws
colcon build --packages-select tf_tree_terminal_cpp
source install/setup.bash
```

3. **Test (optional):**
```bash
colcon test --packages-select tf_tree_terminal_cpp
colcon test-result --verbose
```

---

## 💡 How it works

The node subscribes to `/tf` and `/tf_static` itself and keeps, per child frame, the parent, the publisher and the stamps of the last 10 seconds. The rate is computed from these stamps, the age from the newest stamp and the node's clock. The publisher of each transform is resolved by matching the message's publisher id against the graph introspection API. The tree model and the text output (`tf_tree.hpp`) do not depend on ROS and are unit-tested on their own.
