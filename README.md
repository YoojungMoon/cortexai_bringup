
<<<<<<< HEAD
# enyoai_bringup (C++-centric)

This package keeps the *bringup pipeline* launch in Python (ROS2 convention),
but moves the integration utilities (health/TF checks) to **C++ (rclcpp)**.

## Build
```bash
mkdir -p ~/correxai_bringup/src
cp -r enyoai_bringup ~/enyoai_ws/src/
cd ~/enyoai_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

## Run
```bash
ros2 launch enyoai_bringup enyoai_stack.launch.py
```

## Notes
- ROS2 params don't support maps. `config/enyoai_stack.yaml` uses parallel arrays for min_hz & TF checks.
- Replace `launch/ari_realsense_vslam_nvblox.launch.py` with your working version if placeholder was generated.
=======
# cortexai_bringup
Enyo AI Drone integrated launch for realsense, visual SLAM, nvBlox
>>>>>>> origin/main
