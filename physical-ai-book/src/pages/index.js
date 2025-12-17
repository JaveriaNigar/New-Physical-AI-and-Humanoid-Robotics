import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import AuthorBox from '@site/src/components/AuthorBox';
import styles from './index.module.css';

function HomepageHeader() {
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">Physical AI & Humanoid Robotics Textbook</h1>
        <p className="hero__subtitle">A comprehensive guide covering Physical AI and Humanoid Robotics from theory to full system implementation</p>
      </div>
    </header>
  );
}

export default function Home() {
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics Textbook`}
      description="A comprehensive Docusaurus textbook covering Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className={styles.homepageLayout}>
              <div className={styles.bookOverview}>
                <h2>Book Overview</h2>
                <h3>Physical AI & Humanoid Robotics Textbook</h3>
                <p>
                  This comprehensive textbook covers 4 distinct modules across 13 weeks,
                  designed to allow students to learn the entire quarter independently.
                </p>
                <h4>Key Points:</h4>
                <ul>
                  <li><strong>Module 1:</strong> The Robotic Nervous System (ROS 2)</li>
                  <li><strong>Module 2:</strong> The Digital Twin (Gazebo & Unity)</li>
                  <li><strong>Module 3:</strong> The AI-Robot Brain (NVIDIA Isaac)</li>
                  <li><strong>Module 4:</strong> Vision-Language-Action (VLA)</li>
                </ul>
                <p>
                  Target audience: CS/AI students with basic programming knowledge. All
                  technical claims are cited with at least 40% from peer-reviewed or
                  academic sources.
                </p>
              </div>
              <div className={styles.authorIntroduction}>
                <h2>Author Introduction</h2>
                <AuthorBox />
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}