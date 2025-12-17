# Physical AI & Humanoid Robotics Textbook

This is a comprehensive Docusaurus textbook covering Physical AI and Humanoid Robotics from theory to full system implementation. The book is designed to allow students to learn the entire quarter independently, including all modules and weekly topics.

## Course Overview

This textbook covers 4 distinct modules across 13 weeks:

1. **Module 1 — The Robotic Nervous System (ROS 2)**: Covers nodes, topics, services, actions, data flow, and humanoid description using URDF.
2. **Module 2 — The Digital Twin (Gazebo & Unity)**: Explains simulation environments, physics, sensor simulation, and visualization.
3. **Module 3 — The AI-Robot Brain (NVIDIA Isaac)**: Details Isaac Sim, perception, navigation, and Nav2 for humanoids.
4. **Module 4 — Vision-Language-Action (VLA)**: Covers conversational robotics, Whisper integration, and LLM-ROS translation.

## Target Audience

This textbook is designed for CS/AI students with a basic understanding of programming concepts. All technical claims are cited with at least 40% from peer-reviewed or academic sources.

## Prerequisites

- Ubuntu operating system
- ROS 2 Humble or newer
- Node.js (for Docusaurus site)
- Basic programming knowledge

## Building the Textbook

Run the following commands to build and serve the textbook:

```bash
# Install dependencies
npm install

# Serve the textbook in development mode
npm start

# Build the static site
npm run build
```

## Repository Structure

- `/docs` - Main content organized by weeks/modules
- `/src` - Custom components and styling
- `/static` - Images and static assets
- `/blog` - Supplementary content

## License

This textbook is provided as an educational resource under the MIT License.