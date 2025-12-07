/**
 * Creating a sidebar enables you to:
 * - create an ordered group of docs
 * - render a sidebar for each doc of that group
 * - provide next/previous navigation
 *
 * The sidebars can be generated from the filesystem, or explicitly defined here.
 *
 * Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // Main book sidebar with 6-module structure
  bookSidebar: [
    {
      type: 'doc',
      id: 'index',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'Module 0: Physical AI Foundations',
      link: {
        type: 'generated-index',
        title: 'Module 0: Physical AI Foundations',
        description: 'Fundamental concepts of embodied AI, morphological computation, sensing, and locomotion.',
        slug: '/module-0-foundations',
      },
      items: [
        {
          type: 'doc',
          id: 'chapters/module-0-foundations/introduction',
          label: 'Chapter 1: Introduction to Physical AI',
        },
        {
          type: 'doc',
          id: 'chapters/module-0-foundations/embodied-intelligence',
          label: 'Chapter 2: Embodied Intelligence',
        },
        {
          type: 'doc',
          id: 'chapters/module-0-foundations/sensing-perception',
          label: 'Chapter 3: Sensing and Perception',
        },
        {
          type: 'doc',
          id: 'chapters/module-0-foundations/locomotion-motor-control',
          label: 'Chapter 4: Locomotion and Motor Control',
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      link: {
        type: 'generated-index',
        title: 'Module 1: ROS 2 Ecosystem',
        description: 'Robot Operating System 2 - nodes, topics, services, actions, TF, and URDF.',
        slug: '/module-1-ros2',
      },
      items: [
        {
          type: 'doc',
          id: 'chapters/module-1-ros2/core-concepts',
          label: 'Chapter 1: Core Concepts',
        },
        {
          type: 'doc',
          id: 'chapters/module-1-ros2/nodes-topics-services-actions',
          label: 'Chapter 2: Nodes, Topics, Services, Actions',
        },
        {
          type: 'doc',
          id: 'chapters/module-1-ros2/tf-urdf',
          label: 'Chapter 3: TF and URDF',
        },
        {
          type: 'doc',
          id: 'chapters/module-1-ros2/simulation-pipeline',
          label: 'Chapter 4: Simulation Pipeline',
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin',
      link: {
        type: 'generated-index',
        title: 'Module 2: Gazebo & Unity',
        description: 'Robot simulation with Gazebo Harmonic and Unity for digital twin development.',
        slug: '/module-2-digital-twin',
      },
      items: [
        {
          type: 'doc',
          id: 'chapters/module-2-digital-twin/basics',
          label: 'Chapter 1: Digital Twin Basics',
        },
        {
          type: 'doc',
          id: 'chapters/module-2-digital-twin/gazebo-physics',
          label: 'Chapter 2: Gazebo Physics',
        },
        {
          type: 'doc',
          id: 'chapters/module-2-digital-twin/unity-animation',
          label: 'Chapter 3: Unity Animation',
        },
        {
          type: 'doc',
          id: 'chapters/module-2-digital-twin/ros2-integration',
          label: 'Chapter 4: ROS 2 Integration',
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac',
      link: {
        type: 'generated-index',
        title: 'Module 3: NVIDIA Isaac',
        description: 'Isaac Sim, Isaac Gym, and Isaac ROS for AI-enabled robotics.',
        slug: '/module-3-isaac',
      },
      items: [
        {
          type: 'doc',
          id: 'chapters/module-3-isaac/overview',
          label: 'Chapter 1: NVIDIA Isaac Overview',
        },
        {
          type: 'doc',
          id: 'chapters/module-3-isaac/perception-planning',
          label: 'Chapter 2: Isaac ROS Perception and Planning',
        },
        {
          type: 'doc',
          id: 'chapters/module-3-isaac/sim-to-real',
          label: 'Chapter 3: Sim-to-Real Transfer',
        },
        {
          type: 'doc',
          id: 'chapters/module-3-isaac/control-loops',
          label: 'Chapter 4: Control Loops with RL and IL',
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA Models',
      link: {
        type: 'generated-index',
        title: 'Module 4: Vision-Language-Action',
        description: 'Foundation models for robotics: OpenVLA, SmolVLA, and RT-2.',
        slug: '/module-4-vla',
      },
      items: [
        {
          type: 'doc',
          id: 'chapters/module-4-vla/fundamentals',
          label: 'Chapter 1: VLA Fundamentals',
        },
        {
          type: 'doc',
          id: 'chapters/module-4-vla/rt2-models',
          label: 'Chapter 2: RT-2 and Open-Source VLA Models',
        },
        {
          type: 'doc',
          id: 'chapters/module-4-vla/policy-integration',
          label: 'Chapter 3: Policy Integration',
        },
        {
          type: 'doc',
          id: 'chapters/module-4-vla/humanoid-skills',
          label: 'Chapter 4: Humanoid Skills with VLA',
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 5: Capstone',
      link: {
        type: 'generated-index',
        title: 'Module 5: Autonomous Humanoid',
        description: 'End-to-end humanoid robot system integrating all modules.',
        slug: '/module-5-capstone',
      },
      items: [
        {
          type: 'doc',
          id: 'chapters/module-5-capstone/system-overview',
          label: 'Chapter 1: System Overview',
        },
        {
          type: 'doc',
          id: 'chapters/module-5-capstone/perception-stack',
          label: 'Chapter 2: Perception Stack',
        },
        {
          type: 'doc',
          id: 'chapters/module-5-capstone/control-stack',
          label: 'Chapter 3: Control Stack',
        },
        {
          type: 'doc',
          id: 'chapters/module-5-capstone/vla-autonomy',
          label: 'Chapter 4: VLA Autonomy',
        },
      ],
    },
    {
      type: 'category',
      label: 'Appendices',
      link: {
        type: 'generated-index',
        title: 'Appendices',
        description: 'Reference materials, prerequisites, and solutions.',
        slug: '/appendices',
      },
      items: [
        {
          type: 'doc',
          id: 'appendices/A-prerequisites',
          label: 'A: Prerequisites',
        },
        {
          type: 'doc',
          id: 'appendices/B-mathematics-review',
          label: 'B: Mathematics Review',
        },
        {
          type: 'doc',
          id: 'appendices/C-setup-instructions',
          label: 'C: Setup Instructions',
        },
        {
          type: 'doc',
          id: 'appendices/D-solutions',
          label: 'D: Exercise Solutions',
        },
        {
          type: 'doc',
          id: 'glossary',
          label: 'Glossary',
        },
        {
          type: 'doc',
          id: 'references/bibliography',
          label: 'Bibliography',
        },
      ],
    },
  ],
};

module.exports = sidebars;
