// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Getting Started',
      items: ['getting-started/installation', 'getting-started/basic-concepts'],
    },
    {
      type: 'category',
      label: 'ROS2 Basics',
      items: ['ros2-basics/nodes', 'ros2-basics/topics', 'ros2-basics/services'],
    },
    {
      type: 'category',
      label: 'Advanced Topics',
      items: ['advanced/tf-transforms', 'advanced/actions', 'advanced/launch-files'],
    },
    {
      type: 'category',
      label: 'Projects',
      items: ['projects/turtlebot-simulation', 'projects/autonomous-robot'],
    },
  ],
};

export default sidebars;