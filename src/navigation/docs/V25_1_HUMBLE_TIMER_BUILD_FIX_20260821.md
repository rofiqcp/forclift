# V25.1 ROS 2 Humble camera timer build fix

## Symptom
`astra_rgb_v4l2_node.cpp` fails in `rclcpp::function_traits` while instantiating `create_wall_timer()`.

## Root cause
The V25 statistics timer used a `mutable` lambda with an init-capture. In ROS 2 Humble, the `rclcpp::function_traits` implementation used by `Node::create_wall_timer()` does not handle this non-const lambda call operator correctly.

## Fix
Move the mutable timer state (`last_published`) into `AstraRGBNode` as `last_stats_published_`, and use an ordinary `[this]()` callback. Sensor and camera rates are unchanged; only the compile-time callback representation changes.

## Build environment warning
If `install/navigation` was removed while an old overlay was still sourced, `AMENT_PREFIX_PATH` and `CMAKE_PREFIX_PATH` can contain a stale path. Build from a fresh terminal and source only `/opt/ros/humble/setup.bash` before rebuilding the workspace.
