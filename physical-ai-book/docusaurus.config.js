require('dotenv').config();

// @ts-check
// `@type` JSDoc annotations allow IDEs and type checkers to infer types
// @ts-ignore - js file
const {themes} = require('prism-react-renderer');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'A comprehensive textbook covering Physical AI and Humanoid Robotics from theory to full system implementation',
  favicon: 'img/favicon.ico',

  // GitHub Pages URL
  url: 'https://JaveriaNigar.github.io',
  baseUrl: '/New-Physical-AI-and-Humanoid-Robotics/',

  // GitHub deployment config
  organizationName: 'JaveriaNigar', // GitHub username
  projectName: 'New-Physical-AI-and-Humanoid-Robotics', // Repo name

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',
  trailingSlash: true,

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
          editUrl: 'https://github.com/JaveriaNigar/New-Physical-AI-and-Humanoid-Robotics',
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
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        style: 'primary',
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
        style: 'light',
        links: [
          {
            title: 'Textbook',
            items: [
              { label: 'Module One', to: '/docs/week-1-2/humanoid-robots' },
              { label: 'Module Two', to: '/docs/week-3-5/humanoid-urdf-links-joints-sensors' },
              { label: 'Module Three', to: '/docs/week-8-10/ai-robot-brain-overview' },
              { label: 'Module Four', to: '/docs/week-11-13/humanoid-development-overview' },
            ],
          },
          {
            title: 'Creator',
            items: [
              { label: 'Portfolio', href: 'https://portfolio-five-alpha-98.vercel.app/' },
              { label: 'LinkedIn', href: 'https://www.linkedin.com/in/javeria-nigar-252b312b5/' },
              { label: 'Twitter', href: 'https://x.com/Nigarhaide526' },
            ],
          },
          {
            title: 'More',
            items: [
              { label: 'GitHub', href: 'https://github.com/JaveriaNigar' },
              { label: 'Email', href: 'mailto:javerianigar40@gmail.com' },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook.`,
      },
      prism: {
        theme: themes.github,
        darkTheme: themes.github,
      },
      colorMode: {
        defaultMode: 'light',
        disableSwitch: true,
        respectPrefersColorScheme: false,
      },
    }),
  customFields: {
    apiBaseUrl: process.env.REACT_APP_API_BASE_URL,
  },
};

module.exports = config;
