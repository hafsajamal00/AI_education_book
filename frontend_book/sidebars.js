// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'intro',
        'module1/chapter1-ros2-fundamentals',
        'module1/chapter2-ros2-communication',
        'module1/chapter3-robot-structure',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module2/chapter1-digital-twins',
        'module2/chapter2-gazebo-physics',
        'module2/chapter3-unity-integration',
      ],
    },
  ],
};

module.exports = sidebars;