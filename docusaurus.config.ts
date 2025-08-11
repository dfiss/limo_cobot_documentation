import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'LIMO Cobot Documentation',
  tagline: 'Comprehensive guide for LIMO collaborative robotics platform',
  favicon: 'img/ipm_logo.png',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://krish-rRay23.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/limo_cobot_documentation/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'krish-rRay23', // Usually your GitHub org/user name.
  projectName: 'limo_cobot_documentation', // Usually your repo name.

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          // editUrl:
          //   'https://github.com/krish-rRay23/limo_cobot_documentation/tree/main/',
          // Enhanced table of contents configuration
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
        },
        blog: false, // Disable blog functionality
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themes: ['@docusaurus/theme-mermaid'],
  
  markdown: {
    mermaid: true,
  },

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    metadata: [
      {name: 'keywords', content: 'limo, cobot, robotics, documentation, api'},
      {name: 'twitter:card', content: 'summary_large_image'},
    ],
    docs: {
      sidebar: {
        hideable: true,
      },
    },
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'DOCS',
      logo: {
        alt: 'LIMO Cobot Logo',
        src: 'img/ipm_logo.png', 
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Documentation',
        },
        {
          type: 'dropdown',
          label: 'Guides',
          position: 'left',
          items: [
            {
              label: 'Quick Start',
              to: '/docs/intro',
            },
            {
              label: 'Hardware Setup',
              to: '/docs/getting-started/hardware-unboxing',
            },
            {
              label: 'Software Installation',
              to: '/docs/getting-started/installing-code',
            },
            {
              label: 'Environment Setup',
              to: '/docs/environment-setup/configuration',
            },
            {
              label: 'ROS2 Workspace',
              to: '/docs/environment-setup/ros2-workspace',
            },
            {
              label: 'Core Concepts',
              to: '/docs/core-concepts/ros2-basics',
            },
            {
              label: 'System Components',
              to: '/docs/system-components/overview',
            },
            {
              label: 'Usage Guide',
              to: '/docs/usage-guide/basic-operations',
            },
            {
              label: 'Troubleshooting',
              to: '/docs/troubleshooting/common-errors',
            },
          ],
        },
        {to: '/demo', label: 'Demo', position: 'left'},
        {
          type: 'search',
          position: 'right',
        },
        {
          href: 'https://github.com/krish-rRay23/limo_cobot_documentation',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Documentation',
          items: [
            {
              label: 'Getting Started',
              to: '/docs/intro',
            },
            {
              label: 'API Reference',
              to: '/docs/system-components/overview',
            },
            {
              label: 'Tutorials',
              to: '/docs/usage-guide/basic-operations',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Support Forum',
              href: 'https://github.com/krish-rRay23/limo_cobot_documentation/discussions',
            },
            {
              label: 'Discord',
              href: 'https://discord.gg/your-discord',
            },
            {
              label: 'Issues',
              href: 'https://github.com/krish-rRay23/limo_cobot_documentation/issues',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'Live Demo',
              to: '/demo',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/krish-rRay23/limo_cobot_documentation',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} LIMO Cobot Documentation. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['bash', 'diff', 'json', 'python', 'javascript'],
    },
    algolia: {
      // The application ID provided by Algolia
      appId: 'YOUR_APP_ID',
      // Public API key: it is safe to commit it
      apiKey: 'YOUR_SEARCH_API_KEY',
      indexName: 'YOUR_INDEX_NAME',
      // Optional: see doc section below
      contextualSearch: true,
      // Optional: Specify domains where the navigation should occur through window.location instead on history.push
      externalUrlRegex: 'external\\.com|domain\\.com',
      // Optional: Replace parts of the item URLs from Algolia
      replaceSearchResultPathname: {
        from: '/docs/', // or as RegExp: /\/docs\//
        to: '/',
      },
      // Optional: Algolia search parameters
      searchParameters: {},
      // Optional: path for search page that enabled by default (`false` to disable it)
      searchPagePath: 'search',
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
