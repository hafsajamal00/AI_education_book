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
  ],
};

module.exports = sidebars;