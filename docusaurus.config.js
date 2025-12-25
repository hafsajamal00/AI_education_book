// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

// import {themes as prismThemes} from 'prism-react-renderer';

// /** @type {import('@docusaurus/types').Config} */
// const config = {
//   title: 'ROS2 Educational Book',
//   tagline: 'Learn ROS2 step by step',
//   favicon: 'img/favicon.ico',

//   // Set the production url of your site here
//   url: 'https://your-actual-domain.vercel.app', // Replace with your actual domain
//   // Set the /<baseUrl>/ pathname under which your site is served
//   // For GitHub pages deployment, it is often '/<projectName>/'
//   baseUrl: '/',

//   // GitHub pages deployment config.
//   // If you aren't using GitHub pages, you don't need these.
//   organizationName: 'laiba', // Usually your GitHub org/user name.
//   projectName: 'hackathon-1', // Usually your repo name.

//   onBrokenLinks: 'throw',
//   onBrokenMarkdownLinks: 'warn',

//   // Even if you don't use internationalization, you can use this field to set
//   // useful metadata like html lang. For example, if your site is Chinese, you
//   // may want to replace "en" with "zh-Hans".
//   i18n: {
//     defaultLocale: 'en',
//     locales: ['en'],
//   },

//   presets: [
//     [
//       'classic',
//       /** @type {import('@docusaurus/preset-classic').Options} */
//       ({
//         docs: {
//           sidebarPath: require.resolve('./sidebars.js'),
//           // Please change this to your repo.
//           // Remove this to remove the "edit this page" links.
//           editUrl:
//             'https://github.com/laiba/hackathon-1/tree/main/',
//         },
//         theme: {
//           customCss: require.resolve('./src/css/custom.css'),
//         },
//       }),
//     ],
//   ],

//   themeConfig:
//     /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
//     ({
//       // Replace with your project's social card
//       image: 'img/docusaurus-social-card.jpg',
//       navbar: {
//         title: 'ROS2 Educational Book',
//         logo: {
//           alt: 'ROS2 Logo',
//           src: 'img/logo.svg', // Make sure to add your logo to static/img/
//         },
//         items: [
//           {
//             type: 'docSidebar',
//             sidebarId: 'tutorialSidebar',
//             position: 'left',
//             label: 'Tutorial',
//           },
//           {
//             href: 'https://github.com/laiba/hackathon-1',
//             label: 'GitHub',
//             position: 'right',
//           },
//         ],
//       },
//       footer: {
//         style: 'dark',
//         links: [
//           {
//             title: 'Docs',
//             items: [
//               {
//                 label: 'Tutorial',
//                 to: '/docs/intro',
//               },
//             ],
//           },
//           {
//             title: 'Community',
//             items: [
//               {
//                 label: 'Stack Overflow',
//                 href: 'https://stackoverflow.com/questions/tagged/docusaurus',
//               },
//               {
//                 label: 'Discord',
//                 href: 'https://discordapp.com/invite/docusaurus',
//               },
//               {
//                 label: 'Twitter',
//                 href: 'https://twitter.com/docusaurus',
//               },
//             ],
//           },
//           {
//             title: 'More',
//             items: [
//               {
//                 label: 'GitHub',
//                 href: 'https://github.com/laiba/hackathon-1',
//               },
//             ],
//           },
//         ],
//         copyright: `Copyright © ${new Date().getFullYear()} ROS2 Educational Book. Built with Docusaurus.`,
//       },
//       prism: {
//         theme: prismThemes.github,
//         darkTheme: prismThemes.dracula,
//       },
//     }),
// };

// export default config;   







import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'ROS2 Educational Book',
  tagline: 'Learn ROS2 step by step',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://hackathon-1-git-main-laiba.vercel.app', // Updated for Vercel deployment
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/',  // ye zaroor '/'
  trailingSlash: false,

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'laiba', // Usually your GitHub org/user name.
  projectName: 'hackathon-1', // Usually your repo name.

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',
  markdown: {
    mermaid: true,
  },

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  // Additional production settings
  onBrokenAnchors: 'throw',

  // Enable trailing slash for consistent routing - REMOVED AS PER DEPLOYMENT REQUIREMENTS
  // trailingSlash: true,

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/laiba/hackathon-1/tree/main/',
          routeBasePath: '/docs', // Serve docs from /docs instead of /
        },
        blog: false, // Disable blog
        pages: {
          path: 'src/pages',
          include: ['**/*.{js,jsx,ts,tsx,md,mdx}'],
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  // Enable trailing slash for consistent routing
  trailingSlash: true,

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'ROS2 Educational Book',
        logo: {
          alt: 'ROS2 Educational Book Logo',
          src: 'img/logo.svg',
          href: '/', // Link to homepage - this was causing broken links
          target: '_self', // Ensure it's a relative link
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Tutorial',
            docsPluginId: 'default', // Reference the default docs plugin
          },
          {
            href: 'https://github.com/laiba/hackathon-1',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Tutorial',
                to: '/docs/intro',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/ros2',
                className: 'footer__icon-stackoverflow',
              },
              {
                label: 'Discord',
                href: 'https://discord.gg/ros2',
                className: 'footer__icon-discord',
              },
              {
                label: 'Twitter',
                href: 'https://twitter.com/ROStwo',
                className: 'footer__icon-twitter',
              },
            ],
          },
          {
            title: 'GitHub',
            items: [
              {
                label: 'ROS2',
                href: 'https://github.com/ros2/ros2',
                className: 'footer__icon-github',
              },
              {
                label: 'This Project',
                href: 'https://github.com/laiba/hackathon-1',
                className: 'footer__icon-github',
              },
            ],
          },
        ],
        copyright: `Copyright © 2025 Physical AI & Humanoid Robotics Learning Platform. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;