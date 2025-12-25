import React from 'react';
import clsx from 'clsx';
import Card from './Card';
import styles from './ModuleCards.module.css';

// Icons for the cards
const RobotIcon = () => <i className="fas fa-robot" />;
const GearsIcon = () => <i className="fas fa-cogs" />;
const BrainIcon = () => <i className="fas fa-brain" />;
const RocketIcon = () => <i className="fas fa-rocket" />;

/**
 * A reusable module cards component for displaying related modules in docs pages
 * @param {Object} props - Component properties
 * @param {string} props.className - Additional CSS classes
 * @param {Object} props.style - Additional inline styles
 * @returns {JSX.Element} ModuleCards component
 */
function ModuleCards({ className, style }) {
  const modules = [
    {
      title: "ROS 2 & Humanoid Control",
      description: "Learn to control humanoid robots using ROS 2 framework",
      to: "/docs/module1/chapter1-ros2-fundamentals",
      icon: <RobotIcon />,
      progress: 75,
      status: "in-progress"
    },
    {
      title: "Digital Twin Simulation",
      description: "Create and interact with digital twins in simulated environments",
      to: "/docs/module2/chapter1-digital-twins",
      icon: <GearsIcon />,
      progress: 40,
      status: "in-progress"
    },
    {
      title: "AI for Robotics",
      description: "Apply artificial intelligence techniques to robotics problems",
      to: "/docs/module3/chapter1-nvidia-isaac-sim-fundamentals",
      icon: <BrainIcon />,
      progress: 20,
      status: "in-progress"
    },
    {
      title: "Real-World Deployment",
      description: "Deploy your solutions on actual robotic hardware",
      to: "/docs/module-4-vision-language-action/chapter-3-capstone-project",
      icon: <RocketIcon />,
      progress: 5,
      status: "in-progress"
    }
  ];

  return (
    <div className={clsx(styles.moduleCardsContainer, className)} style={style}>
      <h2 className={styles.sectionTitle}>Learning Modules</h2>
      <div className={styles.moduleCardsGrid}>
        {modules.map((module, index) => (
          <div key={index} className={styles.moduleCardItem}>
            <Card
              title={module.title}
              description={module.description}
              to={module.to}
              icon={module.icon}
              progress={module.progress}
              status={module.status}
              ariaLabel={`Learn about ${module.title}`}
            />
          </div>
        ))}
      </div>
    </div>
  );
}

export default ModuleCards;