^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf_tree_terminal_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Rewrite as C++ package ``tf_tree_terminal_cpp`` (ament_cmake, rclcpp); the Python package is removed
* Listen to ``/tf`` and ``/tf_static`` directly instead of going through a tf2 buffer
* Show the publishing node(s) per transform instead of all publishers of the topic, and flag
  transforms that several nodes publish
* Report stamps ahead of the clock (``FUTURE``), frames with several parents and loops
* Frames without parent are labelled ``ROOT``; static frames show ``STATIC`` once
* ``/joint_states`` publishers are listed once in the diagnostic instead of on every joint
* Unknown command line arguments are an error
* Unit tests with gtest, CI for Humble, Jazzy, Kilted and Rolling

Entries below are from the Python package ``tf_tree_terminal``.

2.0.0 (2026-01-06)
------------------
* update humble ubuntu version
* rename CI workflows
* fix typo []
* add distinct workflows
* update badges
* update ros2 ci
* updat ros tooling
* ros2 ci workflow
* Fix description for ROS2 TF Tree CLI Helper
* Add geometry_msgs dependency to package.xml
* add test for tree node
* Strip ANSI codes from content before saving
  Added ANSI escape code stripping to save_to_file method.
* Add --no-color option to command line arguments
* Add color support for terminal output
* Implement TF rate summary in tree_node.py
  Added functionality to track and summarize TF rates.
* Add frequency label to light tree node output
* Add --clear option to command line arguments
* Add clear_screen option to TFTreeCLI initialization
* Update TF snapshot formatting and compliance status symbols
* Update symbols for better clarity in output messages
* add helper comment
* update version
* update readme
* add light arg
* Hz, tf and joit_states status, helper with options
* linter changes
* add my name to package.xml
* Contributors: tanneguy, tanneguydv

1.0.1 (2025-12-23 14:36)
------------------------
* update version
* add Datetime to output txt and ASCII header
* add tf-tree global command
* Contributors: tanneguydv

1.0.0 (2025-12-23 10:12)
------------------------
* first commit
* Contributors: tanneguydv
