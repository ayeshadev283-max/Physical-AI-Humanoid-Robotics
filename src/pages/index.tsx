import React, { useEffect, useState } from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    setIsVisible(true);
  }, []);

  return (
    <header className={`${styles.heroBanner} ${isVisible ? styles.visible : ''}`}>
      <div className="container">
        <div className={styles.heroContent}>
          <h1 className={`${styles.heroTitle} ${isVisible ? styles.fadeInUp : ''}`}>
            {siteConfig.title}
          </h1>
          <p className={`${styles.heroSubtitle} ${isVisible ? styles.fadeInUp : ''} ${styles.delay1}`}>
            {siteConfig.tagline}
          </p>
          <p className={`${styles.heroDescription} ${isVisible ? styles.fadeInUp : ''} ${styles.delay2}`}>
            An open-source, comprehensive guide to building intelligent, embodied systems.
            Bridge the gap between theoretical AI and physical robot implementation.
          </p>
          <div className={`${styles.buttons} ${isVisible ? styles.fadeInUp : ''} ${styles.delay3}`}>
            <Link
              className={`button button--primary button--lg ${styles.primaryButton}`}
              to="/docs/">
              Get Started
            </Link>
            <Link
              className={`button button--secondary button--lg ${styles.secondaryButton}`}
              to="https://github.com/ayeshadev283-max/Physical-AI-Humanoid-Robotics">
              View on GitHub
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

interface FeatureItem {
  title: string;
  description: string;
  icon: string;
}

const FeatureList: FeatureItem[] = [
  {
    title: 'ROS 2 Integration',
    description: 'Master the robotic nervous system with nodes, topics, and services. Build scalable robot software architectures.',
    icon: '🤖',
  },
  {
    title: 'Digital Twin Simulation',
    description: 'Simulate robots in Gazebo & Unity for rapid iteration. Bridge the reality gap with advanced sim-to-real techniques.',
    icon: '🎮',
  },
  {
    title: 'NVIDIA Isaac Platform',
    description: 'Leverage GPU-accelerated robotics for AI-native workflows. Train RL policies with massively parallel environments.',
    icon: '⚡',
  },
  {
    title: 'Vision-Language-Action Models',
    description: 'Deploy foundation models like OpenVLA and SmolVLA for generalizable robot control and task planning.',
    icon: '👁️',
  },
  {
    title: 'Hands-On Learning',
    description: 'Tested, runnable code examples in Docker. Learn by building real humanoid robot systems.',
    icon: '💻',
  },
  {
    title: 'Open Source',
    description: 'Content licensed under CC BY-SA 4.0. Code under Apache 2.0. Freely modifiable and shareable.',
    icon: '📖',
  },
];

function Feature({ title, description, icon }: FeatureItem) {
  return (
    <div className={`col col--4 ${styles.feature}`}>
      <div className={styles.featureCard}>
        <div className={styles.featureIcon}>{icon}</div>
        <h3 className={styles.featureTitle}>{title}</h3>
        <p className={styles.featureDescription}>{description}</p>
      </div>
    </div>
  );
}

function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <h2 className={styles.featuresHeading}>What You'll Learn</h2>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

function HomepageStats() {
  return (
    <section className={styles.stats}>
      <div className="container">
        <div className="row">
          <div className="col col--3">
            <div className={styles.statCard}>
              <div className={styles.statNumber}>6</div>
              <div className={styles.statLabel}>Modules</div>
            </div>
          </div>
          <div className="col col--3">
            <div className={styles.statCard}>
              <div className={styles.statNumber}>100+</div>
              <div className={styles.statLabel}>Code Examples</div>
            </div>
          </div>
          <div className="col col--3">
            <div className={styles.statCard}>
              <div className={styles.statNumber}>50+</div>
              <div className={styles.statLabel}>Diagrams</div>
            </div>
          </div>
          <div className="col col--3">
            <div className={styles.statCard}>
              <div className={styles.statNumber}>∞</div>
              <div className={styles.statLabel}>Possibilities</div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function LearningPath() {
  return (
    <section className={styles.learningPath}>
      <div className="container">
        <h2 className={styles.pathHeading}>Your Learning Journey</h2>
        <div className={styles.pathContainer}>
          <div className={styles.pathItem}>
            <div className={styles.pathNumber}>01</div>
            <h3>Foundations</h3>
            <p>Embodied AI, probabilistic robotics, and subsumption architecture</p>
          </div>
          <div className={styles.pathArrow}>→</div>
          <div className={styles.pathItem}>
            <div className={styles.pathNumber}>02</div>
            <h3>ROS 2</h3>
            <p>Build the computational graph for robot software systems</p>
          </div>
          <div className={styles.pathArrow}>→</div>
          <div className={styles.pathItem}>
            <div className={styles.pathNumber}>03</div>
            <h3>Simulation</h3>
            <p>Master Gazebo, Unity, and NVIDIA Isaac platforms</p>
          </div>
          <div className={styles.pathArrow}>→</div>
          <div className={styles.pathItem}>
            <div className={styles.pathNumber}>04</div>
            <h3>VLA Models</h3>
            <p>Deploy foundation models for generalizable control</p>
          </div>
          <div className={styles.pathArrow}>→</div>
          <div className={styles.pathItem}>
            <div className={styles.pathNumber}>05</div>
            <h3>Capstone</h3>
            <p>Build a complete humanoid robot system</p>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Graduate-level technical textbook on Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <HomepageStats />
        <LearningPath />
      </main>
    </Layout>
  );
}
