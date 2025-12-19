# vlm_bridge

Simple ROS2 (Humble) Python package to receive typed natural-language commands and publish Twist messages to `/cmd_vel`.

Quick start:

1. Ensure dependencies are installed (ROS2 Humble, turtlebot3_gazebo, etc.)
2. Build the workspace:

```bash
cd ~/vlm_ws
colcon build --symlink-install
source install/setup.bash
```

3. Launch the package (this will start the CLI and bridge):

```bash
ros2 launch vlm_bridge vlm_launch.py
```

4. In the CLI window, type `Go forward` and press Enter — the robot should move forward.

Notes:
- Gemini integration is left as a stub; set `USE_GEMINI=1` and implement `parse_with_gemini()` to enable it.
- You can also run nodes separately:
  - `ros2 run vlm_bridge vlm_bridge_node`
  - `ros2 run vlm_bridge vlm_cli`
