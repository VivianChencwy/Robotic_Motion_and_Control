# Robotic Motion and Control

A comprehensive robotic motion planning and control system designed for Kinova robot arms. This project implements advanced path planning algorithms, trajectory optimization, and real-time control for robotic manipulation tasks.

## Overview

This project provides a complete solution for robotic motion planning and control, featuring:

- **RRT* Path Planning**: Implementation of the Rapidly-exploring Random Tree Star algorithm for optimal path generation
- **Trajectory Optimization**: B-spline based trajectory smoothing and optimization
- **Real-time Control**: PID-based angle control for precise robot movement
- **Collision Detection**: Advanced collision checking for safe robot operation
- **Kinematics**: Forward kinematics calculation for 6-DOF robot arms
- **Visualization**: 3D trajectory visualization and plotting tools
- **Grasping**: Fast grasping capabilities for object manipulation

## Features

### Core Components

- **Path Planning (`rrt.py`)**: RRT* algorithm implementation with collision avoidance
- **Control System (`pid_angle_control.py`)**: PID controller for joint angle control
- **Kinematics (`kinematics.py`)**: Forward kinematics for robot arm configuration
- **Collision Detection (`collision.py`)**: Real-time collision checking
- **Trajectory Optimization (`optimize.py`)**: B-spline based path smoothing
- **Visualization (`visualize.py`)**: 3D plotting and trajectory visualization
- **Grasping (`fast_grasp.py`)**: Advanced grasping functionality

### Key Capabilities

- Multi-degree-of-freedom robot arm control
- Obstacle avoidance path planning
- Smooth trajectory generation
- Real-time robot control via Kinova Kortex API
- ROS integration support
- Comprehensive logging and debugging

## Requirements

### Hardware
- Kinova robot arm (compatible with Kortex API)
- Computer with USB/Ethernet connection to robot

### Software Dependencies
- Python 3.7+
- ROS (Robot Operating System)
- NumPy
- Matplotlib
- SciPy
- Kinova Kortex API

### Python Packages
```bash
pip install numpy matplotlib scipy
```

### ROS Dependencies
```bash
sudo apt-get install ros-<distro>-roscpp ros-<distro>-rospy ros-<distro>-std-msgs
```

## Installation

1. **Clone the repository** into your ROS workspace:
```bash
cd ~/catkin_ws/src
git clone <repository-url>
```

2. **Install Kinova Kortex API** (follow Kinova's official documentation)

3. **Build the ROS package**:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

4. **Install Python dependencies**:
```bash
pip install -r requirements.txt  # if available
```

## Usage

### Basic Path Planning and Execution

1. **Configure robot parameters** in `main.py`:
   - Set start and goal joint angles
   - Adjust DH parameters for your robot configuration
   - Modify joint limits as needed

2. **Run the main application**:
```bash
python main.py
```

This will:
- Generate an optimal path using RRT*
- Optimize the trajectory using B-splines
- Execute the path on the real robot (if connected)
- Display trajectory visualization

### Individual Components

#### Path Planning Only
```bash
python -c "
from rrt import RRTStar
from collision import CollisionChecker
# Configure and run path planning
"
```

#### PID Control Testing
```bash
python pid_angle_control.py
```

#### Visualization
```bash
python visualize.py
```

#### Fast Grasping
```bash
python fast_grasp.py
```

### Configuration

#### Robot Parameters
Edit the DH parameters in `main.py` to match your robot:
```python
dh_params = [
    # [theta,      d,      a,          alpha]
    [0,           0,       0.2433,     0],          # Joint 1
    [math.pi/2,   0,       0.01,       math.pi/2],  # Joint 2
    # ... add remaining joints
]
```

#### Planning Parameters
Adjust RRT* parameters in `rrt.py`:
- `step_size`: Step size for tree expansion
- `max_iter`: Maximum planning iterations
- `radius`: Search radius for rewiring

#### Control Parameters
Tune PID gains in `pid_angle_control.py`:
```python
controller = PIDController(Kp=1.0, Ki=0.1, Kd=0.05)
```

## Project Structure

```
├── main.py              # Main application entry point
├── rrt.py               # RRT* path planning implementation
├── pid_angle_control.py # PID control system
├── kinematics.py        # Forward kinematics calculations
├── collision.py         # Collision detection module
├── optimize.py          # Trajectory optimization
├── visualize.py         # 3D visualization tools
├── fast_grasp.py        # Grasping functionality
├── utilities.py         # Utility functions
├── plot_log.py          # Data logging and plotting
├── package.xml          # ROS package configuration
├── CMakeLists.txt       # Build configuration
└── README.md           # This file
```


