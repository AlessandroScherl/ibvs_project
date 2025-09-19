# Image-Based Visual Servoing (IBVS) Project

A ROS 2 project template for implementing Image-Based Visual Servoing with feature extraction algorithms.

## What is Visual Servoing?

Visual servoing uses camera feedback to control robot motion. The goal is to move the camera so the current view matches a desired reference view by minimizing the error between image features.

- **Input**: Visual features (corners, edges, keypoints)
- **Output**: Velocity commands to move the robot
- **Result**: Robot automatically aligns its view with the desired image

## Choose Your Implementation Language

**You can implement IBVS in either Python OR C++ - you don't need to do both!**

- **Python**: Easier to prototype, good for quick testing (`src/ibvs_controller/`)
- **C++**: Better performance, closer to real robotics systems (`src/ibvs_controller_cpp/`)

Pick the language you're most comfortable with. Both templates provide the same functionality.

## Setup Instructions

### Prerequisites

1. **ROS 2 Jazzy Installation**
   ```bash
   # Check if ROS 2 is installed
   echo $ROS_DISTRO  # Should output 'jazzy'

   # If not installed, follow: https://docs.ros.org/en/jazzy
   ```

2. **Gazebo Sim (formerly Ignition Gazebo)**
   ```bash
   # Should be installed with ROS 2 Jazzy desktop-full
   gz sim --version  # Should show version 8.x
   ```

3. **OpenCV Installation**
   ```bash
   # For Python users
   pip install opencv-python opencv-contrib-python

   # For C++ users (usually already installed with ROS 2)
   sudo apt install libopencv-dev
   ```

4. **Workspace Setup**
   ```bash
   # Clone or copy this project to your home directory
   cd ~
   # Project should be at: ~/ibvs_project
   ```

### Build
```bash
# Navigate to project
cd ~/ibvs_project

# Build all packages
colcon build

# For faster builds, build only what you need:
colcon build --packages-select ibvs_simulation  # Required for everyone
colcon build --packages-select ibvs_controller   # For Python users only
colcon build --packages-select ibvs_controller_cpp  # For C++ users only

# Source the workspace (REQUIRED after every build)
source install/setup.bash
```

### Run

You may need **3 terminals** to run the complete system:

**Terminal 1 - Launch Simulation:**
```bash
cd ~/ibvs_project
source install/setup.bash
ros2 launch ibvs_simulation ibvs_world.launch.py
```
This starts Gazebo with a camera robot at 1.5m height looking down at a target board.

**Terminal 2 - Run YOUR Controller:**
```bash
cd ~/ibvs_project
source install/setup.bash

# If you're using Python:
ros2 run ibvs_controller camera_controller

# If you're using C++:
ros2 run ibvs_controller_cpp camera_controller_node
```

**Terminal 3 - Monitor System (Optional but helpful):**
```bash
cd ~/ibvs_project
source install/setup.bash

# List all topics
ros2 topic list

# View velocity commands your controller sends
ros2 topic echo /ibvs/cmd_vel

# Check camera frame rate (should be ~30 Hz)
ros2 topic hz /ibvs/image_raw
```

## Implementation Tasks

**Remember: Choose either Python OR C++ - implement in one language only!**

### 1. Feature Extraction

**Python:** Edit `src/ibvs_controller/ibvs_controller/camera_controller.py`
**C++:** Edit `src/ibvs_controller_cpp/src/camera_controller.cpp`

Implement feature extraction in:
- Python: `extract_features()` method
- C++: `extractFeatures()` method

**Options:**
- ORB (recommended to start - fast and easy)
- SIFT (more robust, slower)
- AKAZE (good balance)

**Steps:**
1. Convert image to grayscale
2. Create feature detector
3. Detect keypoints
4. Return keypoints for control

### 2. Interaction Matrix
Compute the Image Jacobian that relates feature velocities to camera velocities:
```
L = [ -1/Z   0   x/Z   xy   -(1+x²)   y  ]
    [  0   -1/Z  y/Z  1+y²    -xy    -x  ]
```
Where (x,y) are normalized coordinates, Z is depth.

### 3. Control Law
Implement IBVS control:
```
v = -λ * L⁺ * e
```
- v = velocity command (6D) = output
- λ = gain (0.1 default)
- L⁺ = pseudoinverse of interaction matrix
- e = feature error (current - desired)

### 4. Desired Features
1. Load a desired/reference image
2. Extract features from desired image
3. Match current with desired features
4. Compute pixel errors

Use OpenCV's `imread()` to load images. Feature error: e = current_features - desired_features (pixels).

## ROS 2 Commands

### Robot Control
```bash
# Send velocity manually
ros2 topic pub --once /ibvs/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}"

# Respawn at new position
ros2 service call /delete gz_msgs/srv/DeleteEntity "{name: 'rgbd_camera_robot'}"
ros2 run ros_gz_sim create -name rgbd_camera_robot \
  -file src/ibvs_simulation/models/rgbd_camera_robot/model.sdf \
  -x 0.5 -y 0 -z 2.0 -R 0 -P 1.5708 -Y 0
```

### Velocity Reference
- `linear.x/y/z`: Forward/Left/Up motion (m/s)
- `angular.x/y/z`: Roll/Pitch/Yaw rotation (rad/s)

## Testing

### Step-by-Step
1. **Basic Movement**: Test velocity commands work
2. **Feature Detection**: Verify features are detected (aim for 10-20)
3. **Full IBVS**: Start close to goal, gradually increase distance

## Project Structure

```
ibvs_project/
├── src/
│   ├── ibvs_controller/        # Python IBVS implementation
│   ├── ibvs_controller_cpp/    # C++ IBVS implementation
│   └── ibvs_simulation/        # Gazebo simulation environment
└── README.md                   # This file
```

## Topics

- `/ibvs/image_raw` - RGB camera (sensor_msgs/Image)
- `/ibvs/depth/image_raw` - Depth camera (sensor_msgs/Image)
- `/ibvs/camera_info` - Camera calibration (sensor_msgs/CameraInfo)
- `/ibvs/cmd_vel` - Velocity commands (geometry_msgs/Twist)

## Important Notes

- **Depth Workaround**: Gazebo depth sensor has issues at steep angles. A workaround using last valid depth is already implemented.
- **Coordinates**: Camera frame is X-right, Y-down, Z-forward
- **Control Rate**: 10 Hz loop, Lambda gain = 0.1
- **Camera**: 640x480 resolution, ~60° FOV

## Troubleshooting

- **No movement**: Check feature detection and matching
- **Oscillations**: Reduce lambda gain to 0.05
- **Wrong direction**: Verify interaction matrix signs
- **Few features**: Adjust detector parameters

## Optional Extensions

Once basic IBVS works:
- Try different targets (modify `worlds/ibvs_world.sdf`)
- Compare feature detectors (ORB vs SIFT vs AKAZE)
- Add visualization of detected features
