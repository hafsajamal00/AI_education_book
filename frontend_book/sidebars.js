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
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module3/chapter1-nvidia-isaac-sim-fundamentals',
        'module3/chapter2-isaac-ros-perception-navigation',
        'module3/chapter3-nav2-bipedal-humanoid-movement',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4-vision-language-action/intro',
        'module-4-vision-language-action/quickstart',
        'module-4-vision-language-action/chapter-1-voice-to-action',
        'module-4-vision-language-action/chapter-2-cognitive-planning',
        'module-4-vision-language-action/chapter-3-capstone-project',
        'module-4-vision-language-action/edge-cases',
        'module-4-vision-language-action/validation',
      ],
    },
  ],
};

module.exports = sidebars;