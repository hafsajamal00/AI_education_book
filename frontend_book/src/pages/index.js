import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Footer from './components/Footer';
import Card from '../components/Card';
import styles from './index.module.css';

// Icons for the cards
const RobotIcon = () => <i className="fas fa-robot" />;
const GearsIcon = () => <i className="fas fa-cogs" />;
const BrainIcon = () => <i className="fas fa-brain" />;
const RocketIcon = () => <i className="fas fa-rocket" />;

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">Physical AI & Humanoid Robotics Learning Platform</h1>
        <p className="hero__subtitle">Step into the world of humanoid robotics, AI, and digital twin simulations.</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg gradient-button"
            to="#modules">
            Start Learning
          </Link>
          <Link
            className="button button--outline button--lg github-button"
            to="https://github.com">
            View GitHub
          </Link>
        </div>
      </div>
    </header>
  );
}


export default function Home() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Learning Platform">
      <HomepageHeader />
      <main>
        {/* Features Section */}
        <section className={styles.features}>
          <div className="container">
            <h2 className={styles.sectionTitle}>Key Features</h2>
            <div className="row">
              <div className={clsx('col col--3', styles.cardContainer)}>
                <Card
                  title="ROS 2 & Humanoid Control"
                  description="Learn to control humanoid robots using ROS 2 framework"
                  to="/docs/module1/chapter1-ros2-fundamentals"
                  ariaLabel="Learn about ROS 2 & Humanoid Control"
                  icon={<RobotIcon />}
                  progress={75}
                  status="in-progress"
                />
              </div>
              <div className={clsx('col col--3', styles.cardContainer)}>
                <Card
                  title="Digital Twin Simulation"
                  description="Create and interact with digital twins in simulated environments"
                  to="/docs/module2/chapter1-digital-twins"
                  ariaLabel="Learn about Digital Twin Simulation"
                  icon={<GearsIcon />}
                  progress={40}
                  status="in-progress"
                />
              </div>
              <div className={clsx('col col--3', styles.cardContainer)}>
                <Card
                  title="AI for Robotics"
                  description="Apply artificial intelligence techniques to robotics problems"
                  to="/docs/module3/chapter1-nvidia-isaac-sim-fundamentals"
                  ariaLabel="Learn about AI for Robotics"
                  icon={<BrainIcon />}
                  progress={20}
                  status="in-progress"
                />
              </div>
              <div className={clsx('col col--3', styles.cardContainer)}>
                <Card
                  title="Real-World Deployment"
                  description="Deploy your solutions on actual robotic hardware"
                  to="/docs/module-4-vision-language-action/chapter-3-capstone-project"
                  ariaLabel="Learn about Real-World Deployment"
                  icon={<RocketIcon />}
                  progress={5}
                  status="in-progress"
                />
              </div>
            </div>
          </div>
        </section>

        {/* Modules Section */}
        <section id="modules" className={styles.modules}>
          <div className="container">
            <h2 className={styles.sectionTitle}>Learning Modules</h2>
            <div className="row">
              <div className={clsx('col col--3', styles.cardContainer)}>
                <Card
                  title="Physical AI Foundations"
                  description="Core concepts of AI applied to physical systems"
                  to="/docs/intro"
                  ariaLabel="Learn about Physical AI Foundations"
                  icon={<BrainIcon />}
                  progress={100}
                  status="completed"
                />
              </div>
              <div className={clsx('col col--3', styles.cardContainer)}>
                <Card
                  title="Digital Twins (Gazebo & Unity)"
                  description="Simulation environments for robot development"
                  to="/docs/module2/chapter1-digital-twins"
                  ariaLabel="Learn about Digital Twins (Gazebo & Unity)"
                  icon={<GearsIcon />}
                  progress={40}
                  status="in-progress"
                />
              </div>
              <div className={clsx('col col--3', styles.cardContainer)}>
                <Card
                  title="Perception & Vision-Language Models"
                  description="Understanding robot perception and multimodal AI"
                  to="/docs/module-4-vision-language-action/chapter-1-voice-to-action"
                  ariaLabel="Learn about Perception & Vision-Language Models"
                  icon={<BrainIcon />}
                  progress={10}
                  status="in-progress"
                />
              </div>
              <div className={clsx('col col--3', styles.cardContainer)}>
                <Card
                  title="Humanoid Control & Deployment"
                  description="Controlling humanoid robots in real-world scenarios"
                  to="/docs/module-4-vision-language-action/chapter-3-capstone-project"
                  ariaLabel="Learn about Humanoid Control & Deployment"
                  icon={<RobotIcon />}
                  progress={5}
                  status="in-progress"
                />
              </div>
            </div>
          </div>
        </section>
      </main>
      <Footer />
    </Layout>
  );
}