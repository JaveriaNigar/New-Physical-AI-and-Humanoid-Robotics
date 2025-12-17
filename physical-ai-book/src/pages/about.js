import React from 'react';
import Layout from '@theme/Layout';
import AuthorBox from '@site/src/components/AuthorBox';

export default function AboutPage() {
  return (
    <Layout
      title="About"
      description="Learn more about the Physical AI & Humanoid Robotics Textbook and its creator">
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1 className="hero__title">About This Textbook</h1>
            <p className="hero__subtitle">
              Welcome to the Physical AI & Humanoid Robotics Textbook, a comprehensive resource 
              covering everything from fundamental ROS 2 concepts to advanced Vision-Language-Action systems.
            </p>
            
            <div className="margin-vert--lg">
              <h2>Book Overview</h2>
              <p>
                This textbook covers 4 distinct modules across 13 weeks, designed to allow students 
                to learn the entire quarter independently. The content includes:
              </p>
              <ul>
                <li><strong>Module 1:</strong> The Robotic Nervous System (ROS 2)</li>
                <li><strong>Module 2:</strong> The Digital Twin (Gazebo & Unity)</li>
                <li><strong>Module 3:</strong> The AI-Robot Brain (NVIDIA Isaac)</li>
                <li><strong>Module 4:</strong> Vision-Language-Action (VLA)</li>
              </ul>
            </div>
            
            <div className="margin-vert--lg">
              <h2>About the Author</h2>
              <AuthorBox />
            </div>
            
            <div className="margin-vert--lg">
              <h2>Target Audience</h2>
              <p>
                This textbook is designed for CS/AI students with a basic understanding of programming concepts. 
                All technical claims are cited with at least 40% from peer-reviewed or academic sources.
              </p>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}