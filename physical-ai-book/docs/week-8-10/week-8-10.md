---
title: "Module 3: The AI-Robot Brain (NVIDIA Isaac™)"
description: "Weeks 8-10 - NVIDIA Isaac Sim, Isaac ROS, and Nav2 for perception and path planning in humanoid robotics"
tags: ["NVIDIA Isaac", "Isaac Sim", "Isaac ROS", "Nav2", "Path Planning", "Humanoid Robotics", "Perception"]
---

# Module 3: The AI-Robot Brain (NVIDIA Isaac™)

## Overview

In this module, we'll explore the NVIDIA Isaac Platform, which consists of Isaac Sim for simulation and Isaac ROS for perception and navigation capabilities. We'll learn how to leverage these tools to build intelligent humanoid robots that can understand and navigate their environment.

## Learning Objectives

By the end of this module (Weeks 8-10), you will be able to:

- Install and set up the NVIDIA Isaac Sim environment
- Work with Isaac ROS packages for perception and hardware acceleration
- Implement navigation systems using Nav2 for bipedal humanoid movement
- Create perception pipelines to enable robot awareness
- Design path planning algorithms for humanoid robots
- Integrate perception and navigation for complete autonomous behavior

---

## Week 8: NVIDIA Isaac Sim Overview and Setup

NVIDIA Isaac Sim is a powerful robotics simulator built on NVIDIA Omniverse. It provides a highly realistic physics environment for developing, testing, and validating robotics applications before deploying to real hardware.

### 1. Introduction to NVIDIA Isaac Platform

The NVIDIA Isaac Platform is a comprehensive robotics development platform that includes:
- Isaac Sim: High-fidelity simulation environment
- Isaac ROS: ROS 2 packages for perception and navigation
- Isaac Navigation: Path planning and navigation solutions
- Isaac Manipulation: Tools for robot manipulation tasks
- Isaac Apps: Pre-built applications for common robotics tasks

### 2. Prerequisites and System Requirements

Before installing NVIDIA Isaac Sim, ensure your system meets the following requirements:

```bash
# Check if you have a compatible NVIDIA GPU
nvidia-smi

# Verify your current ROS 2 installation
ros2 --version
```

**Hardware Requirements:**
- NVIDIA GPU with CUDA Compute Capability 6.0 or higher (with real-time ray-tracing support strongly recommended)
- At least 16GB system RAM (32GB recommended)
- Ubuntu 20.04 LTS or 22.04 LTS
- At least 20GB of available disk space

**Software Requirements:**
- Ubuntu 20.04 LTS or 22.04 LTS
- ROS 2 Humble Hawksbill
- Docker and NVIDIA Container Toolkit (recommended)
- NVIDIA GPU drivers (version 520 or later)

### 3. Installing NVIDIA Isaac Sim

#### Option A: Using Docker (Recommended)

Docker provides the most reliable way to run Isaac Sim:

```bash
# Pull the latest Isaac Sim container
docker pull nvcr.io/nvidia/isaac-sim:latest

# Run Isaac Sim container (replace with your actual paths)
docker run --gpus all -it --rm \
  --network=host \
  --env NVIDIA_VISIBLE_DEVICES=all \
  --env NVIDIA_DRIVER_CAPABILITIES=all \
  --env DISPLAY \
  --env PYTHONPATH=/isaac_sim/python \
  --volume $HOME/.Xauthority:/root/.Xauthority \
  --volume $HOME/isaac-sim/projects:/projects \
  --volume $HOME/isaac-sim/cache/kit:/isaac-sim/kit/cache \
  --volume $HOME/isaac-sim/cache/ov:/root/.cache/ov \
  --volume $HOME/isaac-sim/logs:/root/.nvidia-omniverse/logs \
  --volume $HOME/isaac-sim/config:/root/.nvidia-omniverse/config \
  --volume $HOME/isaac-sim/data:/ov/data \
  --privileged \
  --pid=host \
  nvcr.io/nvidia/isaac-sim:latest
```

#### Option B: Native Installation

For native installation, follow these steps:

```bash
# 1. Clone the Isaac Sim repository
git clone https://github.com/NVIDIA-Omniverse/isaac-sim.git
cd isaac-sim

# 2. Install prerequisites
sudo apt update
sudo apt install -y python3 python3-pip python3-venv build-essential

# 3. Create a virtual environment
python3 -m venv ~/isaac-sim-env
source ~/isaac-sim-env/bin/activate

# 4. Install Isaac Sim
pip install -e .
```

### 4. Verifying Installation

After installation, verify that Isaac Sim is working correctly:

```bash
# If using native installation
source ~/isaac-sim-env/bin/activate
python -c "import omni; print('Isaac Sim Python API is available!')"

# If using Docker, run the container and execute:
# python -c "import omni; print('Isaac Sim Python API is available!')"
```

### 5. Basic Isaac Sim Concepts

#### 5.1 Scene Graph and USD

Isaac Sim uses Universal Scene Description (USD) as its core data model:
- USD is a 3D scene representation that enables complex scene composition
- It allows for layering, referencing, and instancing of complex scenes
- USD supports multiple backends (Physics, Rendering, etc.)

#### 5.2 Extensions

Isaac Sim provides various extensions for robotics development:
- `omni.isaac.ros_bridge`: Connects Isaac Sim with ROS 2
- `omni.isaac.range_sensor`: Provides range sensor support (lidar, depth camera)
- `omni.isaac.motion_generation`: Provides motion planning capabilities
- `omni.isaac.sensor`: Provides sensor support for cameras and IMUs

#### 5.3 Physics Simulation

Isaac Sim supports two physics engines:
- PhysX: NVIDIA's physics engine optimized for robotics
- Isaac Sim also provides accurate collision detection and response

### 6. Your First Isaac Sim Project

Let's create a simple scene in Isaac Sim:

```python
# First_isaac_sim.py
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import RigidPrim, XFormPrim
from omni.isaac.core.utils.prims import get_prim_at_path

# Initialize the world
my_world = World(stage_units_in_meters=1.0)

# Add a ground plane and a simple cube
my_world.scene.add_default_ground_plane()
cube = my_world.scene.add(XFormPrim(prim_path="/World/cube", name="my_cube", position=[0, 0, 1]))
cube.set_world_pose(position=[0, 0, 1], orientation=[0, 0, 0, 1])

# Play the simulation
my_world.reset()
for i in range(1000):
    my_world.step(render=True)

my_world.clear()
```

### 7. Exercises

1. **Basic Scene Creation**: Create a scene with multiple objects of different shapes (cube, sphere, cylinder) and colors.
2. **Simple Physics Simulation**: Create a scene with a static ground and a stack of falling cubes. Observe how they interact with the physics engine.
3. **Camera Integration**: Add a camera to your scene and capture images from different viewpoints.

### 8. Summary

This week, we covered the basics of NVIDIA Isaac Sim, including installation, core concepts, and creating your first simulation. Isaac Sim provides the foundation for physics-accurate simulations that bridge the gap between development and real-world deployment.

---

## Week 9: Isaac ROS - Perception and Hardware Acceleration

The NVIDIA Isaac ROS packages bring hardware acceleration to traditional robotics perception tasks. These packages leverage the GPU for real-time perception tasks such as depth estimation, SLAM, and computer vision.

### 1. Introduction to Isaac ROS

Isaac ROS is a collection of hardware-accelerated ROS 2 packages that perform perception, navigation, and manipulation tasks. These packages are optimized for NVIDIA GPUs and run much faster than their CPU equivalents.

### 2. Core Isaac ROS Packages

#### 2.1 Isaac ROS AprilTag

AprilTag detection optimized for NVIDIA GPUs:

```bash
# Run Isaac ROS AprilTag detection
ros2 launch isaac_ros_apriltag_apriltag.launch.py
```

#### 2.2 Isaac ROS Stereo DNN

Real-time stereo depth estimation using deep neural networks:

```bash
# Run Isaac ROS Stereo DNN
ros2 launch isaac_ros_stereo_dnn_stereo_dnn.launch.py
```

#### 2.3 Isaac ROS VSLAM

Visual Simultaneous Localization and Mapping:

```bash
# Run Isaac ROS VSLAM
ros2 launch isaac_ros_visual_slam_visual_slam.launch.py
```

#### 2.4 Isaac ROS Point Cloud

Converting depth images to point clouds:

```bash
# Run Isaac ROS Point Cloud
ros2 launch isaac_ros_point_cloud_point_cloud.launch.py
```

### 3. Perception Pipelines

Let's create an example perception pipeline with Isaac ROS:

```python
# perception_pipeline.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class PerceptionPipeline(Node):
    def __init__(self):
        super().__init__('perception_pipeline')
        
        # Create a subscription to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_rect_color',
            self.image_callback,
            10)
        
        # Create a publisher for processed images
        self.publisher = self.create_publisher(
            Image,
            '/camera/processed_image',
            10)
        
        self.bridge = CvBridge()
        self.get_logger().info('Perception Pipeline Node Started')
        
    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Process the image (example: edge detection)
        processed_image = cv2.Canny(cv_image, 100, 200)
        
        # Convert back to ROS Image message
        processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='mono8')
        processed_msg.header = msg.header
        
        # Publish the processed image
        self.publisher.publish(processed_msg)

def main(args=None):
    rclpy.init(args=args)
    perception_pipeline = PerceptionPipeline()
    rclpy.spin(perception_pipeline)
    perception_pipeline.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Hardware Acceleration Features

#### 4.1 GPU Acceleration

Isaac ROS packages leverage NVIDIA GPUs for accelerated processing:
- Tensor Cores for deep learning inference
- CUDA cores for general compute tasks
- Dedicated video processing units for video encoding/decoding

#### 4.2 CUDA Graphs

For real-time applications, CUDA Graphs can be used to optimize execution:

```bash
# Enable CUDA graphs for Isaac ROS packages (where supported)
export ISAAC_ROS_CUDAGRAPH_MODE=1
```

#### 4.3 Memory Management

Optimize memory usage in perception pipelines:
- Use zero-copy memory transfers where possible
- Allocate memory pools for repeated operations
- Implement efficient memory management for real-time requirements

### 5. Real-time Perception Example

Here's an example of a complete real-time perception system using Isaac ROS:

```python
# real_time_perception.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from vision_msgs.msg import Detection2DArray
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String

class RealTimePerception(Node):
    def __init__(self):
        super().__init__('real_time_perception')
        
        # Subscribe to camera topics
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_rect_color',
            self.image_callback,
            10)
        
        self.info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/rgb/camera_info',
            self.info_callback,
            10)
            
        # Publishers for processed data
        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            '/detections',
            10)
            
        self.debug_publisher = self.create_publisher(
            Image,
            '/debug_image',
            10)
        
        self.bridge = CvBridge()
        self.camera_info = None
        
        self.get_logger().info('Real Time Perception Node Started')
        
    def info_callback(self, msg):
        self.camera_info = msg
        
    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Process image to detect objects (simple example)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Detect circles using HoughCircles (for example)
        circles = cv2.HoughCircles(
            gray, 
            cv2.HOUGH_GRADIENT, 
            1, 
            20, 
            param1=50, 
            param2=30, 
            minRadius=10, 
            maxRadius=100
        )
        
        # Draw detected circles on debug image
        debug_image = cv_image.copy()
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                cv2.circle(debug_image, (x, y), r, (0, 255, 0), 4)
        
        # Create and publish detections
        detections = Detection2DArray()
        if circles is not None:
            for (x, y, r) in circles:
                detection = Detection2D()
                detection.bbox.center.x = x
                detection.bbox.center.y = y
                detection.bbox.size_x = 2 * r
                detection.bbox.size_y = 2 * r
                detections.detections.append(detection)
        
        detections.header = msg.header
        self.detection_publisher.publish(detections)
        
        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_publisher.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    real_time_perception = RealTimePerception()
    rclpy.spin(real_time_perception)
    real_time_perception.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 6. Performance Optimization

#### 6.1 Pipeline Optimization

To achieve real-time performance:

1. **Minimize data copying**: Use zero-copy mechanisms where possible
2. **Threading**: Use separate threads for different stages of the pipeline
3. **Batch processing**: Process multiple frames together when possible
4. **Memory management**: Reuse buffers and avoid memory allocation during processing

#### 6.2 GPU Utilization

Monitor GPU utilization during perception tasks:

```bash
# Monitor GPU usage
nvidia-smi -l 1

# Check GPU memory usage specifically for Isaac ROS
nvidia-ml-py3 # Python library for GPU monitoring
```

### 7. Exercises

1. **Object Detection Pipeline**: Implement a perception pipeline that detects and tracks objects using Isaac ROS packages.
2. **Depth Estimation**: Use Isaac ROS Stereo DNN to generate depth maps from stereo images.
3. **SLAM Integration**: Set up Isaac ROS VSLAM and visualize the resulting map.
4. **Performance Analysis**: Measure the performance of different perception tasks and identify bottlenecks.

### 8. Summary

This week, we explored Isaac ROS packages for perception tasks and learned how to leverage GPU acceleration for real-time robotics applications. These tools are essential for building intelligent robots that can perceive and understand their environment.

---

## Week 10: Nav2 - Path Planning for Bipedal Humanoid Movement

Navigation is a critical component for mobile robots, and the Navigation2 (Nav2) stack provides a comprehensive framework for path planning and execution. In this week, we'll focus specifically on adapting Nav2 for bipedal humanoid robots.

### 1. Introduction to Nav2

Nav2 is the next-generation navigation stack for ROS 2, designed to replace the ROS 1 navigation stack. It features a behavior tree-based architecture that allows for more complex navigation behaviors and better fault tolerance.

### 2. Nav2 Architecture

Nav2 consists of several key components:

- **Navigation Server**: Coordinates all navigation tasks
- **Planner Server**: Generates global path plans
- **Controller Server**: Generates local velocity commands
- **Recovery Server**: Handles failures with recovery behaviors
- **Lifecycle Manager**: Manages the lifecycle of navigation components

### 3. Nav2 for Humanoid Robots

Unlike wheeled robots, bipedal humanoid robots have unique navigation challenges:

- **Dynamic Balance**: The robot must maintain balance while navigating
- **ZMP (Zero-Moment Point)**: Consider balance point during movement
- **Step Planning**: Instead of continuous motion, humanoid robots move in discrete steps
- **Terrain Adaptability**: Ability to navigate uneven terrain that may require stepping

### 4. Nav2 Configuration for Humanoids

Here's a sample Nav2 configuration for a humanoid robot:

```yaml
# nav2_params_humanoid.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_delay: 0.5
    set_initial_pose: false
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

amcl_map_client:
  ros__parameters:
    use_sim_time: True

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: True

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_footprint
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    # Specify the path where the behavior tree XML files are located
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_assisted_teleop_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_drive_on_heading_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node
    - nav2_controller_cancel_bt_node
    - nav2_path_longer_on_approach_bt_node
    - nav2_wait_cancel_bt_node
    - nav2_spin_cancel_bt_node
    - nav2_back_up_cancel_bt_node
    - nav2_assisted_teleop_cancel_bt_node
    - nav2_drive_on_heading_cancel_bt_node

bt_navigator_rclcpp_node:
  ros__parameters:
    use_sim_time: True

velocity_smoother:
  ros__parameters:
    use_sim_time: True
    smoothing_frequency: 20.0
    scale_velocities: false
    feedback: "OPEN_LOOP"
    velocity_scaling_tolerance: 1.0
    velocity_threshold: 0.0
    stateful: true
    acceleration_limits: [2.5, 2.5, 3.2]
    acceleration_limits_dimensions: [0, 1, 2]
    deceleration_limits: [2.5, 2.5, 3.2]
    deceleration_limits_dimensions: [0, 1, 2]
    velocity_scaling: 1.0

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    # Use the humanoid-specific controller
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPathHumanoid"]

    # Humanoid controller
    FollowPathHumanoid:
      plugin: "nav2_mppi_controller::Controller"
      time_steps: 50
      control_frequency: 20.0
      dt: 0.05
      motion_samples: 100
      aux_distance: 1.0
      aux_speed: 0.4
      reference_speed: 0.3
      max_linear_speed: 0.5
      max_angular_speed: 0.3
      max_linear_accel: 0.3
      max_angular_accel: 0.3
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      debug_weights: false
      transform_tolerance: 0.3
      use_interpolation: true
      model_type: "nav2_mppi_controller::HumanoidModel"
      model_options:
        wheelbase: 0.3
        max_steer: 0.5

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_footprint
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.3  # Humanoid robot radius
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      obstacle_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        voxel_size: 0.05
        max_voxels: 10000
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      always_send_full_costmap: True
  local_costmap_client:
    ros__parameters:
      use_sim_time: True
  local_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_footprint
      use_sim_time: True
      robot_radius: 0.3  # Humanoid robot radius
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        voxel_size: 0.05
        max_voxels: 10000
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True
  global_costmap_client:
    ros__parameters:
      use_sim_time: True
  global_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

smoother_server:
  ros__parameters:
    smoother_frequency: 20.0
    max_angular_velocity: 0.3
    max_linear_velocity: 0.5
    max_angular_acceleration: 0.3
    max_linear_acceleration: 0.3
    use_interpolation: true
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: true

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "assisted_teleop"]
    spin:
      plugin: "nav2_behaviors/Spin"
      spin_dist: 1.57
    backup:
      plugin: "nav2_behaviors/BackUp"
      backup_dist: 0.15
      backup_speed: 0.025
    drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"
      drive_on_heading_max_linear_speed: 0.3
      drive_on_heading_max_angular_speed: 0.3
    assisted_teleop:
      plugin: "nav2_behaviors/AssistedTeleop"
      assisted_teleop_max_linear_speed: 0.3
      assisted_teleop_max_angular_speed: 0.3
      assisted_teleop_min_obstacle_dist: 0.1
    local_frame: odom
    global_frame: map
    robot_base_frame: base_footprint
    transform_tolerance: 0.3
    use_sim_time: True

robot_state_publisher:
  ros__parameters:
    use_sim_time: True
```

### 5. Humanoid-Specific Path Planning

For bipedal robots, navigation needs to consider:

- **Step Sequencing**: Generating a sequence of footstep positions
- **Balance Maintenance**: Ensuring the robot's center of mass remains stable
- **Terrain Analysis**: Detecting suitable footholds on uneven terrain
- **Multi-Contact Planning**: Planning for multiple contact points during movement

#### 5.1 Footstep Planning

Here's an example of footstep planning for humanoid navigation:

```python
#!/usr/bin/env python3
# humanoid_footstep_planner.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np

class HumanoidFootstepPlanner(Node):
    def __init__(self):
        super().__init__('humanoid_footstep_planner')
        
        self.path_sub = self.create_subscription(
            Path,
            'global_plan',
            self.path_callback,
            10)
        
        self.footstep_pub = self.create_publisher(
            MarkerArray,
            'footsteps',
            10)
        
        # Parameters
        self.step_length = 0.3  # distance between consecutive steps
        self.step_width = 0.2   # distance between left and right feet
        self.step_height = 0.1  # height of step motion (for stepping over obstacles)
        
        self.get_logger().info('Humanoid Footstep Planner Initialized')
    
    def path_callback(self, msg):
        """Convert global plan to footstep sequence"""
        if len(msg.poses) < 2:
            return
        
        # Generate footsteps based on the global path
        footsteps = self.generate_footsteps(msg.poses)
        
        # Publish footsteps as visualization markers
        self.publish_footsteps(footsteps)
    
    def generate_footsteps(self, poses):
        """Generate left and right footsteps alternating along the path"""
        footsteps = []
        
        # Start with left foot (arbitrary starting side)
        left_foot = True
        
        # Iterate through path poses and generate footsteps
        for i in range(len(poses) - 1):
            # Calculate direction vector
            dx = poses[i+1].pose.position.x - poses[i].pose.position.x
            dy = poses[i+1].pose.position.y - poses[i].pose.position.y
            dist = np.sqrt(dx*dx + dy*dy)
            
            # If the distance is significant, add a step
            if dist > self.step_length / 2:
                footstep = Pose()
                footstep.position = poses[i].pose.position
                
                # Alternate between left and right foot
                if left_foot:
                    # Offset left foot to the left (positive Y in robot frame)
                    footstep.position.y += self.step_width / 2
                else:
                    # Offset right foot to the right (negative Y in robot frame)
                    footstep.position.y -= self.step_width / 2
                
                footsteps.append({
                    'pose': footstep,
                    'left': left_foot
                })
                
                left_foot = not left_foot  # Alternate for next step
        
        return footsteps
    
    def publish_footsteps(self, footsteps):
        """Publish footsteps as visualization markers"""
        marker_array = MarkerArray()
        
        for i, footstep in enumerate(footsteps):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "footsteps"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose = footstep['pose']
            marker.pose.position.z = 0.02  # Slightly above ground
            
            # Different colors for left and right feet
            if footstep['left']:
                marker.color.r = 1.0  # Red for left foot
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0  # Blue for right foot
                marker.color.g = 0.0
                marker.color.b = 1.0
            marker.color.a = 1.0
            
            marker.scale.x = 0.1  # Foot radius
            marker.scale.y = 0.1  # Foot radius
            marker.scale.z = 0.01 # Thickness
            
            marker_array.markers.append(marker)
        
        self.footstep_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    planner = HumanoidFootstepPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 6. Humanoid Navigation Launch File

Create a launch file to bring up navigation for a humanoid robot:

```xml
<!-- humanoid_navigation.launch.py -->
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_arg = DeclareLaunchArgument(
        'autostart', 
        default_value='True',
        description='Automatically startup the nav2 stack')

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('my_humanoid_bringup'),
            'config',
            'nav2_params_humanoid.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='navigation',
        param_rewrites=param_substitutions,
        convert_types=True)

    # Navigation Server Node
    nav2_node = Node(
        package='nav2_navigation',
        executable='navigation_server',
        name='navigation_server',
        output='screen',
        parameters=[configured_params],
        remappings=[('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')])

    # Lifecycle Manager Node
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['navigation_server',
                                    'local_costmap_client',
                                    'global_costmap_client',
                                    'bt_navigator',
                                    'controller_server',
                                    'smoother_server',
                                    'planner_server',
                                    'behavior_server']}])

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_autostart_arg)
    ld.add_action(declare_params_file_arg)

    ld.add_action(nav2_node)
    ld.add_action(lifecycle_manager)

    return ld
```

### 7. Integration with Isaac Sim

To integrate Nav2 with Isaac Sim for simulation:

1. **Launch Isaac Sim** with your humanoid robot model
2. **Run the ROS bridge** to connect Isaac Sim with ROS 2
3. **Launch Nav2** with the humanoid-specific configuration
4. **Send navigation goals** to test the system

### 8. Exercises

1. **Basic Navigation**: Set up Nav2 with a simple humanoid robot model in Isaac Sim and navigate to different goals.
2. **Custom Path Planner**: Implement a custom path planner plugin for humanoid-specific navigation requirements.
3. **Step Planning**: Develop a footstep planner that generates stable footsteps for navigating uneven terrain.
4. **Balance Integration**: Integrate balance control with navigation to maintain stability during movement.

### 9. Troubleshooting Common Issues

#### 9.1 Navigation Failures
- **Check costmaps**: Ensure local and global costmaps are updating properly
- **Verify transforms**: Ensure all tf transforms are broadcasting correctly
- **Tune parameters**: Adjust navigation parameters based on your robot's capabilities

#### 9.2 Performance Issues
- **Reduce frequency**: Lower the update frequency of navigation components if needed
- **Optimize path planning**: Use more efficient path planning algorithms for real-time performance
- **Simplify costmaps**: Reduce the resolution or size of costmaps if performance is critical

### 10. Summary

In this week, we covered Nav2 setup and configuration for bipedal humanoid robots. We explored how to adapt the navigation stack for the specific requirements of humanoid locomotion, including footstep planning and balance considerations. This completes Module 3 on "The AI-Robot Brain", providing you with the tools to create intelligent robots that can perceive their environment and navigate through it safely.

---

## Module Summary: The AI-Robot Brain (NVIDIA Isaac™)

This module has covered the NVIDIA Isaac platform, which provides the "brain" for intelligent robotics systems. We learned:

1. **Isaac Sim**: High-fidelity simulation for robotics development
2. **Isaac ROS**: Hardware-accelerated perception capabilities
3. **Nav2**: Navigation and path planning for humanoid robots

These technologies form the core of modern robotics AI systems, enabling robots to perceive, understand, and navigate in complex environments. The integration of these tools with ROS 2 provides a powerful and unified platform for building sophisticated humanoid robots.

## Quiz/Reflection Questions

1. What are the main differences between Isaac Sim and traditional robotics simulators?
2. How does hardware acceleration in Isaac ROS improve robot perception capabilities?
3. What unique challenges must be addressed when adapting Nav2 for bipedal humanoid robots compared to wheeled robots?
4. How would you design a perception pipeline that combines Isaac ROS packages for a humanoid robot?
5. What factors would you consider when tuning Nav2 parameters specifically for humanoid robot navigation?

## References

1. NVIDIA Isaac Sim Documentation: https://docs.omniverse.nvidia.com/isaacsim/latest/isaac-sim.html
2. NVIDIA Isaac ROS Documentation: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_common/index.html
3. Navigation2 Documentation: https://navigation.ros.org/
4. ROS 2 Documentation: https://docs.ros.org/en/humble/index.html