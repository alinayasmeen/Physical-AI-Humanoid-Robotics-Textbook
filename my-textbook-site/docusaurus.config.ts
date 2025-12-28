import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'An in-depth guide to the future of AI and robotics',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  url: 'https://physical-ai-humanoid-robotics-textbook-j8dec6teg.vercel.app/',
  baseUrl: '/',

  organizationName: 'alinayasmeen',
  projectName: 'Physical-AI-Humanoid-Robotics-Textbook',

  onBrokenLinks: 'warn',

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
          editUrl:
            'https://github.com/alinayasmeen/Physical-AI-Humanoid-Robotics-Textbook/tree/main/my-textbook-site/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  plugins: [
    // Add plugin to inject environment variables into the client bundle
    async function myPlugin(context, options) {
      return {
        name: 'docusaurus-plugin-inject-env',
        configureWebpack(config, isServer, utils) {
          const { ProvidePlugin } = require('webpack');
          return {
            resolve: {
              alias: {
                // Define process for browser compatibility
                process: require.resolve('process/browser'),
              },
            },
            plugins: [
              // Define environment variables for the client
              new (require('webpack')).DefinePlugin({
                'process.env.REACT_APP_API_URL': JSON.stringify(process.env.REACT_APP_API_URL || 'https://physical-ai-humanoid-robotics-textbook-fcve.onrender.com'),
              }),
              // Provide process globally
              new ProvidePlugin({
                process: 'process/browser',
              }),
            ],
          };
        },
      };
    },
  ],

  themeConfig: {
    image: 'img/robot.jpg',

    colorMode: {
      respectPrefersColorScheme: true,
    },

    navbar: {
      title: 'Robotics Textbook',
      logo: {
        alt: 'Textbook Logo',
        src: 'img/download.png',
        srcDark: 'img/download.png',
      },
      items: [
        {
          type: 'localeDropdown',
        },
        {
          type: 'doc',
          docId: 'intro',
          position: 'left',
          label: 'Docs',
        },
        {
          to: '/course-overview',
          label: 'Overview',
          position: 'right',
        },
        {
          href: 'https://github.com/alinayasmeen/Physical-AI-Humanoid-Robotics-Textbook.git',
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
            {
              label: 'GitHub',
              href: 'https://github.com/alinayasmeen/Physical-AI-Humanoid-Robotics-Textbook.git',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Book by Hafiza Alina Yasmeen. Built with Docusaurus.`,
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
