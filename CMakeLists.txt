cmake_minimum_required(VERSION 3.8)
project(tf_tree_terminal_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)

# tree model and text output, without ROS
add_library(tf_tree STATIC src/tf_tree.cpp)
target_include_directories(tf_tree PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set_target_properties(tf_tree PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(tree_node STATIC src/tree_node.cpp)
target_link_libraries(tree_node PUBLIC tf_tree rclcpp::rclcpp ${tf2_msgs_TARGETS})

# ros2 run tf_tree_terminal_cpp show
add_executable(show src/main.cpp)
target_link_libraries(show PRIVATE tree_node)
install(TARGETS show RUNTIME DESTINATION lib/${PROJECT_NAME})

# the same program as plain command on the PATH of a sourced workspace: tf-tree
add_executable(tf-tree src/main.cpp)
target_link_libraries(tf-tree PRIVATE tree_node)
install(TARGETS tf-tree RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tf_tree test/test_tf_tree.cpp)
  target_link_libraries(test_tf_tree tf_tree)
  ament_add_gtest(test_tree_node test/test_tree_node.cpp)
  target_link_libraries(test_tree_node tree_node)

  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_cpplint()
  ament_uncrustify()
endif()

ament_package()
