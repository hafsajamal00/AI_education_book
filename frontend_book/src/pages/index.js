import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Footer from './components/Footer';
import Card from '../components/Card';
import styles from './index.module.css';

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
            <div className="row">
              <div className={clsx('col col--3')}>
                <Card
                  title="ROS 2 & Humanoid Control"
                  description="Learn to control humanoid robots using ROS 2 framework"
                  to="/docs/category/modules"
                />
              </div>
              <div className={clsx('col col--3')}>
                <Card
                  title="Digital Twin Simulation"
                  description="Create and interact with digital twins in simulated environments"
                  to="/docs/category/modules"
                />
              </div>
              <div className={clsx('col col--3')}>
                <Card
                  title="AI for Robotics"
                  description="Apply artificial intelligence techniques to robotics problems"
                  to="/docs/category/modules"
                />
              </div>
              <div className={clsx('col col--3')}>
                <Card
                  title="Real-World Deployment"
                  description="Deploy your solutions on actual robotic hardware"
                  to="/docs/category/modules"
                />
              </div>
            </div>
          </div>
        </section>

        {/* Modules Section */}
        <section id="modules" className={styles.modules}>
          <div className="container">
            <h2>Learning Modules</h2>
            <div className="row">
              <div className={clsx('col col--3')}>
                <Card
                  title="Physical AI Foundations"
                  description="Core concepts of AI applied to physical systems"
                  to="/docs/category/modules"
                />
              </div>
              <div className={clsx('col col--3')}>
                <Card
                  title="Digital Twins (Gazebo & Unity)"
                  description="Simulation environments for robot development"
                  to="/docs/category/modules"
                />
              </div>
              <div className={clsx('col col--3')}>
                <Card
                  title="Perception & Vision-Language Models"
                  description="Understanding robot perception and multimodal AI"
                  to="/docs/category/modules"
                />
              </div>
              <div className={clsx('col col--3')}>
                <Card
                  title="Humanoid Control & Deployment"
                  description="Controlling humanoid robots in real-world scenarios"
                  to="/docs/category/modules"
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