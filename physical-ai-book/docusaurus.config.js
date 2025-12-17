// @ts-check
// `@type` JSDoc annotations allow IDEs and type checkers to infer types
// @ts-ignore - js file
const {themes} = require('prism-react-renderer');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'A comprehensive textbook covering Physical AI and Humanoid Robotics from theory to full system implementation',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-docusaurus-site.example.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/physical-ai-book/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'M.R Computers', // Usually your GitHub org/user name.
  projectName: 'hackathon', // Usually your repo name. Changed to match the actual GitHub repo

  onBrokenLinks: 'throw',
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
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/MR-Computers/physical-ai-book/tree/main/packages/create-docusaurus/templates/shared/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        style: 'primary', // Use primary color for navbar
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            href: 'https://github.com/JaveriaNigar',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'light', // Use light footer for pastel theme
        links: [
          {
            title: 'Textbook',
            items: [
              {
                label: 'Module One',
                to: '/docs/week-1-2/humanoid-robots',
              },
                  {
                    label: 'Module Two',
                to: '/docs/week-3-5/humanoid-urdf-links-joints-sensors',
              },
              {
                label: 'Module Three',
                to: '/docs/week-8-10/ai-robot-brain-overview',
              },
              {
                label: 'Module Four',
                to: '/docs/week-11-12/humanoid-development-vla-systems',
              }
            ],
          },
          {
            title: 'Creator',
            items: [
              {
                label: 'Portfolio',
                href: 'https://portfolio-five-alpha-98.vercel.app/',
              },
              {
                label: 'LinkedIn',
                href: 'https://www.linkedin.com/in/javeria-nigar-252b312b5/ ',
              },
              {
                label: 'Twitter',
                href: 'https://x.com/Nigarhaide526',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/JaveriaNigar',
              },
              {
                label: 'Email',
                href: 'mailto:javerianigar40@gmail.com',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook.`,
      },
      prism: {
        theme: themes.github,
        darkTheme: themes.github, // Use same theme for both modes since we're disabling dark mode
      },
      colorMode: {
        defaultMode: 'light',
        disableSwitch: true, // Disable dark mode completely
        respectPrefersColorScheme: false, // Don't respect system preference
      },
    }),
};

module.exports = config;