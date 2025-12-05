import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'An in-depth guide to the future of AI and robotics',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://physical-ai-humanoid-robotics-textbook-j8dec6teg.vercel.app/',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/', // Update baseUrl for GitHub Pages deployment

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'your-github-alinayasmeen-or-org', // e.g. 'facebook'
  projectName: 'Physical-AI-Humanoid-Robotics-Textbook', // e.g. 'docusaurus'

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          id: 'default',
          path: 'docs',
          routeBasePath: 'docs',
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/alinayasmeen/Physical-AI-Humanoid-Robotics-Textbook/tree/bf98087522afeec72226e191424628be1dd4d061/my-textbook-site/',
        },
        blog: false, // Disable default blog if not needed
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],
  plugins: [
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'course',
        path: 'course',
        routeBasePath: 'course',
        sidebarPath: require.resolve('./sidebarsCourse.ts'),
      },
    ],
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'labs',
        path: 'labs',
        routeBasePath: 'labs',
        sidebarPath: require.resolve('./sidebarsLabs.ts'),
      },
    ],
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'capstone',
        path: 'capstone',
        routeBasePath: 'capstone',
        sidebarPath: require.resolve('./sidebarsCapstone.ts'),
      },
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img\robot.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Robotics Textbook',
      logo: {
        alt: 'Textbook Logo',
        src: 'img/download.png', // Light mode logo
        srcDark: 'img/logo-dark.svg', // Optional: dark mode logo
      },
      items: [
          {
          type: 'localeDropdown',
        },
        {
          type: 'doc',
          docId: 'intro', // Default docs intro
          position: 'left',
          label: 'Docs',
        },
        {
          type: 'doc',
          docId: 'intro', // This refers to intro.md in the 'course' instance
          position: 'left',
          label: 'Course',
          docsPluginId: 'course',
        },
        {
          type: 'doc',
          docId: 'intro', // This refers to intro.md in the 'labs' instance
          position: 'left',
          label: 'Labs',
          docsPluginId: 'labs',
        },
        {
          type: 'doc',
          docId: 'intro', // This refers to intro.md in the 'capstone' instance
          position: 'left',
          label: 'Capstone',
          docsPluginId: 'capstone',
        },
        {to: '/course-overview', label: 'Overview', position: 'right'},
        {
          href: 'https://github.com/alinayasmeen/Physical-AI-Humanoid-Robotics-Textbook.git', // Link to your GitHub repo
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'More',
          items: [
            { label: 'GitHub', href: 'https://github.com/alinayasmeen/Physical-AI-Humanoid-Robotics-Textbook.git' },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Book by Hafiza Alina Yasmeen, Inc. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
