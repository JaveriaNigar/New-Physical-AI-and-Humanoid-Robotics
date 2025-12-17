---
title: "Gazebo Physics, Collisions, Environment Design"
description: "Advanced Gazebo simulation focusing on physics, collisions, and environment design for humanoid robotics"
tags: ["Gazebo", "Physics", "Collisions", "Environment", "Simulation", "Humanoid"]
---

# Gazebo Physics, Collisions, Environment Design

## Learning Objectives

By the end of this chapter, students will be able to:
- Configure advanced physics parameters for humanoid robot simulation
- Design accurate collision models for robots and environments
- Create complex simulation environments with realistic physics
- Understand and implement contact modeling for humanoid interaction
- Optimize simulation performance for complex multi-body systems
- Design safe simulation environments that protect robot controllers

## Introduction to Advanced Gazebo Physics

In this chapter, we'll explore the advanced physics simulation capabilities of Gazebo that are essential for humanoid robotics. While basic simulation is useful for simple validation, humanoid robots require sophisticated physics modeling to accurately simulate their complex interactions with the environment.

Humanoid robots present unique challenges in simulation due to their multi-body structure, balance requirements, and interaction with environments designed for human activity. Properly configuring physics parameters is critical for achieving realistic behavior and reliable controller validation.

## Physics Engine Configuration

### Selecting the Right Physics Engine

Gazebo supports multiple physics engines through Ignition Physics, each with different strengths:

- **ODE (Open Dynamics Engine)**: Good general-purpose engine, used by default
- **Bullet**: Strong contact handling, good for complex interactions
- **DART (Dynamic Animation and Robotics Toolkit)**: Excellent for articulated systems and compliant contacts

For humanoid robotics, DART is often preferred due to its superior handling of complex articulated systems:

```xml
<!-- In world file -->
<physics type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_update_rate>1000</real_time_update_rate>
  <dart>
    <solver>
      <solver_type>SPARSE_CHOLESKY</solver_type>
      <collision_detector>fcl</collision_detector>
      <contact_surface_layer>0.001</contact_surface_layer>
      <friction_model>coulomb</friction_model>
    </solver>
  </dart>
</physics>
```

### Advanced Physics Parameters

The following parameters significantly impact humanoid simulation quality:

#### Time Step Configuration
```xml
<physics type="dart">
  <max_step_size>0.001</max_step_size>  <!-- Small for humanoid stability -->
  <real_time_update_rate>1000</real_time_update_rate>  <!-- Match control rate -->
</physics>
```

#### Solver Parameters
```xml
<physics type="dart">
  <dart>
    <solver>
      <solver_type>SPARSE_CHOLESKY</solver_type>
      <max_consecutive_collision_iterations>10</max_consecutive_collision_iterations>
      <max_var_solver_iterations>500</max_var_solver_iterations>
      <collision_detector>bullet</collision_detector>
    </solver>
  </dart>
</physics>
```

## Collision Modeling for Humanoids

### Collision Geometry Selection

Choosing appropriate collision geometries is crucial for humanoid simulation:

1. **Primitive Shapes**: Boxes, spheres, and cylinders for basic collision detection
2. **Convex Hulls**: For more accurate approximations of complex shapes
3. **Triangle Meshes**: For detailed collision in critical areas

### Multi-Collision Links

For better simulation accuracy, use multiple collision elements per link:

```xml
<link name="upper_arm">
  <collision name="upper_arm_collision_main">
    <geometry>
      <capsule>
        <radius>0.05</radius>
        <length>0.2</length>
      </capsule>
    </geometry>
  </collision>

  <collision name="upper_arm_collision_elbow">
    <pose>0.0 0.0 -0.1 0 0 0</pose>
    <geometry>
      <sphere>
        <radius>0.06</radius>
      </sphere>
    </geometry>
  </collision>
</link>
```

### Contact Stabilization

Stabilizing contacts is essential for humanoid balance:

```xml
<collision name="foot_collision">
  <surface>
    <contact>
      <ode>
        <max_vel>100.0</max_vel>
        <min_depth>0.001</min_depth>
        <soft_cfm>0.001</soft_cfm>
        <soft_erp>0.99</soft_erp>
      </ode>
    </contact>
    <friction>
      <ode>
        <mu>0.8</mu>
        <mu2>0.8</mu2>
        <fdir1>0 0 0</fdir1>
        <slip1>0.0</slip1>
        <slip2>0.0</slip2>
      </ode>
    </friction>
  </surface>
</collision>
```

## Environment Design for Humanoid Robots

### Creating Human-Scale Environments

Humanoid robots need environments designed for human dimensions:

```xml
<sdf version="1.7">
  <world name="humanoid_environment">
    <!-- Physics -->
    <physics type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Main floor -->
    <model name="floor">
      <static>true</static>
      <link name="floor_link">
        <collision name="floor_collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
        </collision>
        <visual name="floor_visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>10 10</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Doorway for navigation -->
    <model name="doorway">
      <pose>0 0 0 0 0 0</pose>
      <link name="doorway_frame">
        <collision name="left_jamb">
          <geometry>
            <box>
              <size>0.2 0.1 2.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="left_jamb_visual">
          <geometry>
            <box>
              <size>0.2 0.1 2.0</size>
            </box>
          </geometry>
        </visual>
      </link>

      <link name="doorway_top">
        <pose>0 0.6 1.0 0 0 0</pose>
        <collision name="top_jamb">
          <geometry>
            <box>
              <size>0.2 1.2 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="top_jamb_visual">
          <geometry>
            <box>
              <size>0.2 1.2 0.2</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Furniture and Obstacles

Humanoid robots interact with furniture designed for humans:

```xml
<sdf version="1.7">
  <model name="chair">
    <link name="seat">
      <pose>0 0 0.45 0 0 0</pose>
      <collision>
        <geometry>
          <box>
            <size>0.4 0.4 0.02</size>
          </box>
        </geometry>
      </collision>
      <visual>
        <geometry>
          <box>
            <size>0.4 0.4 0.02</size>
        </geometry>
      </visual>
    </link>

    <link name="back_rest">
      <pose>0 -0.15 0.7 0 0 0</pose>
      <collision>
        <geometry>
          <box>
            <size>0.4 0.02 0.4</size>
          </box>
        </geometry>
      </collision>
      <visual>
        <geometry>
          <box>
            <size>0.4 0.02 0.4</size>
          </geometry>
      </visual>
    </link>

    <link name="leg_front_left">
      <pose>-0.15 -0.15 0.23 0 0 0</pose>
      <collision>
        <geometry>
          <cylinder>
            <radius>0.02</radius>
            <length>0.46</length>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <!-- Additional legs... -->
  </model>
</sdf>
```

## Contact Modeling for Humanoid Interaction

### Foot-Ground Contact

The interaction between feet and ground is critical for humanoid locomotion:

```xml
<sdf version="1.7">
  <model name="flat_ground">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <surface>
          <friction>
            <ode>
              <mu>0.8</mu>      <!-- High friction for stable walking -->
              <mu2>0.8</mu2>    <!-- Same for secondary axis -->
            </ode>
          </friction>
          <contact>
            <ode>
              <max_vel>100.0</max_vel>    <!-- Allow fast impacts -->
              <min_depth>0.001</min_depth> <!-- Prevents unstable contacts -->
            </ode>
          </contact>
        </surface>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>100 100</size>
          </plane>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

### Manipulation Contacts

For manipulation tasks, contact parameters affect grasp stability:

```xml
<collision name="finger_collision">
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>   <!-- High friction for stable grasp -->
        <mu2>1.0</mu2>
      </ode>
      <torsional>
        <coefficient>1.0</coefficient>
        <use_patch_radius>false</use_patch_radius>
        <surface_radius>0.01</surface_radius>
      </torsional>
    </friction>
    <contact>
      <ode>
        <soft_cfm>0.001</soft_cfm>   <!-- Stiff contacts -->
        <soft_erp>0.99</soft_erp>    <!-- Good contact error correction -->
        <max_vel>10.0</max_vel>      <!-- Reasonable impact limits -->
        <min_depth>0.0001</min_depth> <!-- Very thin contact layer -->
      </ode>
    </contact>
  </surface>
</collision>
```

## Performance Optimization

### Efficient Collision Meshes

Balancing accuracy and performance:

```xml
<link name="complex_link">
  <!-- Detailed visual for rendering -->
  <visual>
    <geometry>
      <mesh>
        <uri>model://robot/meshes/complex_link.dae</uri>
      </mesh>
    </geometry>
  </visual>

  <!-- Simplified collision for physics -->
  <collision>
    <geometry>
      <mesh>
        <uri>model://robot/meshes/complex_link_collision.stl</uri>
      </mesh>
    </geometry>
  </collision>
</link>
```

### Multi-resolution Environments

For large environments, use level-of-detail approaches:

```xml
<!-- In Gazebo world plugin -->
<plugin name="lod_manager" filename="libgazebo_lod_manager.so">
  <high_detail_distance>5</high_detail_distance>
  <medium_detail_distance>15</medium_detail_distance>
  <low_detail_distance>30</low_detail_distance>
</plugin>
```

## Physics Debugging and Validation

### Checking Physics Stability

Use these commands to check physics behavior:

```bash
# Check for body penetrations
gz topic -t /stats -d 10

# Monitor contact forces
gz topic -t /gazebo/contact/contacts -e

# Use the Gazebo GUI to visualize contact forces
```

### Common Physics Issues and Solutions

1. **Robot falls through floor**: Check collision geometry and physics parameters
2. **Unstable joints**: Verify inertial properties and joint limits
3. **Explosive behavior**: Reduce time step, increase solver iterations
4. **Slipping feet**: Increase friction coefficients (mu, mu2)

## Integration with ROS Control

### Physics-aware Control

For simulation-to-reality transfer, consider physics effects:

```python
# Example of physics-aware controller tuning in simulation
class HumanoidController:
    def __init__(self):
        # Physics-aware PD gains for simulation
        self.kp = 100.0   # Proportional gain
        self.kd = 10.0    # Derivative gain
        self.max_effort = 100.0  # Effort limit

        # Adjust based on simulation characteristics
        self.time_step = 0.001  # Match simulation time step
```

## Exercises

1. Create a humanoid walking simulation with a sloped terrain and tune the physics parameters for stable locomotion.
2. Design a manipulation scenario with appropriate contact parameters for grasp stability.
3. Implement a multi-resolution environment with different detail levels based on robot proximity.
4. Experiment with different physics engines (ODE, Bullet, DART) and compare humanoid balance performance.

## Quiz

1. Which physics engine is often preferred for humanoid robotics simulation due to its superior handling of articulated systems?
   - A) ODE
   - B) Bullet
   - C) DART
   - D) Simbody

2. What is the purpose of the min_depth parameter in contact physics?
   - A) To set the maximum penetration allowed
   - B) To prevent unstable contacts by creating a soft contact layer
   - C) To determine the collision detection range
   - D) To set the visual material thickness

3. Why is it important to use multiple collision elements per link in humanoid robots?
   - A) To improve rendering quality
   - B) To achieve better simulation accuracy with complex shapes
   - C) To increase the robot's weight
   - D) To make the robot more stable

4. What is a typical value for friction coefficient (mu) for ground contact in humanoid simulation?
   - A) 0.1
   - B) 0.4
   - C) 0.8
   - D) 1.2

5. What should be the relationship between the physics simulation time step and the robot's control rate?
   - A) The physics time step should be much larger than the control rate
   - B) The physics time step should be equal to or smaller than the control step time
   - C) There is no relationship
   - D) The physics time step should be exactly half the control rate

## Reflection

Consider how the physics parameters in simulation influence the development of control algorithms for humanoid robots. How might overly idealistic physics (e.g., infinite friction, no delays) lead to controllers that fail on real robots? What are the benefits and drawbacks of using more computationally expensive but realistic physics parameters during development?