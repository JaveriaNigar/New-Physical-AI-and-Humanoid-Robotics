// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Physical AI & Humanoid Robotics Textbook ',
      items: [
        {
          type: 'category',
          label: 'Module 1: Weeks 1-2: Introduction to Physical AI',
          items: [
            'week-1-2/humanoid-robots',
            'week-1-2/physical-ai-intro',
            'week-1-2/sensors-overview'
          ]
        },
        {
          type: 'category',
          label: 'Module 2: Weeks 3-7: ROS 2 Fundamentals & Robot Simulation with Gazebo',
          items: [
            'week-3-5/humanoid-urdf-links-joints-sensors', // Need to create this
            'week-3-5/robot-data-flow', // Need to create this
            'week-3-5/ros2-architecture-nodes-topics-services-actions', // Need to create this
            'week-3-5/ros2-with-python-rclpy', // Need to create this
            'week-6-7/gazebo-physics-collisions-environment-design', // Need to create this
            'week-6-7/sensor-simulation-lidar-depth-imu' // Need to create this
          ]
        },
        {
          type: 'category',
          label: 'Module 3: Weeks 8-10: The AI-Robot Brain (NVIDIA Isaac™)',
          items: [
            'week-8-10/ai-robot-brain-overview', // Need to create this
            'week-8-10/nvidia-isaac-architecture', // Need to create this
            'week-8-10/cognitive-planning-actions' // Need to create this
          ]
        },
        {
          type: 'category',
          label: 'Module 4: Weeks 11-12: Humanoid Development & VLA Systems',
          items: [
            'week-11-13/humanoid-development-overview', // Need to create this
            'week-11-13/vision-language-action-systems', // Need to create this
            'week-11-13/context-aware-behavior' // Need to create this
          ]
        }
      ]
    }
  ],
};

module.exports = sidebars;