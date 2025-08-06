import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * 🚀 LIMO Robotics Project Documentation Structure
 * Clean template ready for your content
 */
const sidebars: SidebarsConfig = {
  // Main documentation sidebar
  tutorialSidebar: [
    'intro',
    
    // ========================================
    // 🚀 LIMO ROBOTICS PROJECT DOCUMENTATION STRUCTURE
    // ========================================
    
    {
      type: 'category',
      label: '🚀 Welcome & Orientation',
      collapsed: false,
      items: [
        'welcome/what-is-this-project',
        'welcome/who-should-use',
        'welcome/how-to-get-help',
      ],
    },
    
    {
      type: 'category',
      label: '📖 Introduction',
      collapsed: false,
      items: [
        'introduction/project-overview',
        'introduction/key-features',
        'introduction/system-architecture',
        'introduction/workflow-diagram',
        'introduction/demo',
      ],
    },
    
    {
      type: 'category',
      label: '⚡ Getting Started',
      collapsed: false,
      items: [
        'getting-started/prerequisites',
        'getting-started/hardware-unboxing',
        'getting-started/computer-setup',
        'getting-started/installing-code',
        'getting-started/one-command-install',
        'getting-started/first-run',
      ],
    },
    
    {
      type: 'category',
      label: '🏗️ Environment Setup',
      collapsed: true,
      items: [
        'environment-setup/workspace-structure',
        'environment-setup/ros2-workspace',
        'environment-setup/configuration',
        'environment-setup/backup-restore',
      ],
    },
    
    {
      type: 'category',
      label: '🤖 Core Concepts',
      collapsed: true,
      items: [
        'core-concepts/ros2-basics',
        'core-concepts/system-integration',
        'core-concepts/slam',
        'core-concepts/navigation',
        'core-concepts/manipulation',
        'core-concepts/object-detection',
      ],
    },
    
    {
      type: 'category',
      label: '🔧 System Components',
      collapsed: true,
      items: [
        'system-components/overview',
        'system-components/object-detection-node',
        'system-components/mission-manager',
        'system-components/pick-node',
        'system-components/drop-node',
        'system-components/pose-setter',
        'system-components/launch-files',
        'system-components/hardware-integration',
      ],
    },
    
    {
      type: 'category',
      label: '📋 Usage Guide',
      collapsed: true,
      items: [
        'usage-guide/system-startup',
        'usage-guide/verification',
        'usage-guide/full-workflow',
        'usage-guide/monitoring',
      ],
    },
    
    {
      type: 'category',
      label: '🚀 Advanced Usage',
      collapsed: true,
      items: [
        'advanced-usage/custom-maps',
        'advanced-usage/custom-objects',
        'advanced-usage/waypoint-management',
        'advanced-usage/hardware-upgrades',
        'advanced-usage/web-interface',
      ],
    },
    
    {
      type: 'category',
      label: '🆘 Troubleshooting',
      collapsed: true,
      items: [
        'troubleshooting/common-errors',
        'troubleshooting/debugging-guide',
        'troubleshooting/reset-recovery',
        'troubleshooting/faq',
      ],
    },
    
    {
      type: 'category',
      label: '👨‍💻 Developer Corner',
      collapsed: true,
      items: [
        'developer/contributing',
        'developer/code-style',
        'developer/adding-nodes',
        'developer/launch-config-files',
      ],
    },
    
    {
      type: 'category',
      label: '📚 Lessons Learned',
      collapsed: true,
      items: [
        'lessons-learned/project-story',
        'lessons-learned/common-mistakes',
        'lessons-learned/tips-tricks',
        'lessons-learned/war-stories',
      ],
    },
    
    {
      type: 'category',
      label: '🔮 Future Work',
      collapsed: true,
      items: [
        'future-work/roadmap',
        'future-work/feature-requests',
        'future-work/improvements',
      ],
    },
    
    {
      type: 'category',
      label: '📎 Appendix',
      collapsed: true,
      items: [
        'appendix/hardware-specs',
        'appendix/wiring-diagrams',
        'appendix/glossary',
        'appendix/references',
        'appendix/acknowledgements',
      ],
    },

    // ========================================
    // 📚 LEGACY TUTORIALS (preserved)
    // ========================================
    {
      type: 'category',
      label: '📚 Tutorial - Basics',
      collapsed: true,
      items: [
        'tutorial-basics/create-a-document',
        'tutorial-basics/create-a-page',
        'tutorial-basics/create-a-blog-post',
        'tutorial-basics/markdown-features',
        'tutorial-basics/deploy-your-site',
        'tutorial-basics/congratulations',
      ],
    },
    {
      type: 'category',
      label: '🔧 Tutorial - Extras',
      collapsed: true,
      items: [
        'tutorial-extras/manage-docs-versions',
        'tutorial-extras/translate-your-site',
      ],
    },

    // ========================================
    // 🎯 TEMPLATE SECTIONS (ready to activate)
    // ========================================
    // 
    // Just uncomment and create your markdown files:
    //
    // {
    //   type: 'category',
    //   label: '🚀 Welcome & Orientation',
    //   collapsed: false,
    //   items: [
    //     'welcome/overview',
    //     'welcome/who-should-use',
    //     'welcome/getting-help',
    //   ],
    // },
    // 
    // {
    //   type: 'category',
    //   label: '📖 Introduction',
    //   collapsed: false,
    //   items: [
    //     'introduction/project-overview',
    //     'introduction/key-features',
    //     'introduction/system-architecture',
    //   ],
    // },
    // 
    // {
    //   type: 'category',
    //   label: '🏗️ Environment Setup',
    //   collapsed: true,
    //   items: [
    //     'environment/workspace-structure',
    //     'environment/ros2-setup',
    //     'environment/configuration',
    //   ],
    // },
    // 
    // {
    //   type: 'category',
    //   label: '🔧 System Components',
    //   collapsed: true,
    //   items: [
    //     'components/overview',
    //     'components/hardware-interface',
    //     'components/software-architecture',
    //   ],
    // },
    // 
    // {
    //   type: 'category',
    //   label: '📋 Usage Guide',
    //   collapsed: true,
    //   items: [
    //     'usage/basic-operations',
    //     'usage/full-workflow',
    //     'usage/monitoring',
    //   ],
    // },
    // 
    // {
    //   type: 'category',
    //   label: '🚀 Advanced Usage',
    //   collapsed: true,
    //   items: [
    //     'advanced/custom-configurations',
    //     'advanced/integration-guide',
    //     'advanced/performance-optimization',
    //   ],
    // },
    // 
    // {
    //   type: 'category',
    //   label: '🆘 Troubleshooting',
    //   collapsed: true,
    //   items: [
    //     'troubleshooting/common-issues',
    //     'troubleshooting/error-messages',
    //     'troubleshooting/faq',
    //   ],
    // },
    // 
    // {
    //   type: 'category',
    //   label: '👨‍💻 Developer Corner',
    //   collapsed: true,
    //   items: [
    //     'developer/contributing',
    //     'developer/code-style',
    //     'developer/development-setup',
    //   ],
    // },
    // 
    // {
    //   type: 'category',
    //   label: '📚 Lessons Learned',
    //   collapsed: true,
    //   items: [
    //     'lessons/best-practices',
    //     'lessons/common-pitfalls',
    //     'lessons/project-insights',
    //   ],
    // },
    // 
    // {
    //   type: 'category',
    //   label: '🔮 Future Work',
    //   collapsed: true,
    //   items: [
    //     'future/planned-features',
    //     'future/research-directions',
    //     'future/roadmap',
    //   ],
    // },
    // 
    // {
    //   type: 'category',
    //   label: '📎 Appendix',
    //   collapsed: true,
    //   items: [
    //     'appendix/technical-specifications',
    //     'appendix/glossary',
    //     'appendix/references',
    //   ],
    // },
  ],
};

export default sidebars;
