// @ts-check
const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;
const math = require('remark-math');
const katex = require('rehype-katex');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Graduate-Level Technical Textbook',
  favicon: 'img/favicon.ico',

  url: 'https://ayeshadev283-max.github.io',
  baseUrl: '/Physical-AI-Humanoid-Robotics/', // <-- MUST have trailing slash

  organizationName: 'ayeshadev283-max',
  projectName: 'Physical-AI-Humanoid-Robotics',
  deploymentBranch: 'gh-pages',
  trailingSlash: false, // optional, cleaner URLs

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      ({
        docs: {
          routeBasePath: '/', // Serve docs at site root
          sidebarPath: require.resolve('./sidebars.js'),
          remarkPlugins: [math],
          rehypePlugins: [katex],
          editUrl:
            'https://github.com/ayeshadev283-max/Physical-AI-Humanoid-Robotics/tree/001-physical-ai-robotics-book/',
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  stylesheets: [
    {
      href: 'https://cdn.jsdelivr.net/npm/katex@0.16.0/dist/katex.min.css',
      type: 'text/css',
      integrity: 'sha384-Xi8rHCm/X8argJNqPSLvfTy0o4Oas3E1c+BqccERcXwfDfHHQUzO8+gcpl8G7qKq',
      crossorigin: 'anonymous',
    },
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Book Logo',
        src: 'img/logo.svg',
        href: '/Physical-AI-Humanoid-Robotics/', // <-- use baseUrl here
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'bookSidebar',
          position: 'left',
          label: 'Book',
        },
        {
          href: 'https://github.com/ayeshadev283-max/Physical-AI-Humanoid-Robotics',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Content',
          items: [
            { label: 'Book', to: '/Physical-AI-Humanoid-Robotics/' }, // baseUrl
            { label: 'Bibliography', to: '/Physical-AI-Humanoid-Robotics/references/bibliography' },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Code Examples',
              href: 'https://github.com/ayeshadev283-max/Physical-AI-Humanoid-Robotics/tree/001-physical-ai-robotics-book/examples',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/ayeshadev283-max/Physical-AI-Humanoid-Robotics',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book Contributors. Content licensed under <a href="/Physical-AI-Humanoid-Robotics/LICENSE.content.md" target="_blank">CC BY-SA 4.0</a>. Code examples licensed under <a href="/Physical-AI-Humanoid-Robotics/LICENSE.code.md" target="_blank">Apache 2.0</a>. Built with Docusaurus.`,
    },
    prism: {
      theme: lightCodeTheme,
      darkTheme: darkCodeTheme,
      additionalLanguages: ['python', 'cpp', 'cmake', 'markup', 'yaml', 'bash', 'json'],
    },
  },

  markdown: {
    mermaid: true,
  },
  themes: ['@docusaurus/theme-mermaid'],
};

module.exports = config;
