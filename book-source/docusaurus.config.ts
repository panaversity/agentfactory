import { themes as prismThemes } from "prism-react-renderer";
import type { Config } from "@docusaurus/types";
import type * as Preset from "@docusaurus/preset-classic";

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: "AI Native Software Development",
  tagline:
    "Colearning Agentic AI with Python and TypeScript – The AI & Spec Driven Way",
  favicon: "img/favicon.ico",

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: "https://ai-native.panaversity.org",
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: "/",

  // Sitemap is configured via the classic preset's sitemap option below

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: "panaversity", // Usually your GitHub org/user name.
  projectName: "ai-native-software-development", // Usually your repo name.
  trailingSlash: false,

  onBrokenLinks: "warn",

  // Add Font Awesome for social media icons
  headTags: [
    {
      tagName: "link",
      attributes: {
        rel: "stylesheet",
        href: "https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.5.1/css/all.min.css",
        integrity:
          "sha512-DTOQO9RWCH3ppGqcWaEA1BIZOC6xxalwEsw9c2QQeAIftl+Vegovlnee1c9QX4TctnWMn13TZye+giMm8e2LwA==",
        crossorigin: "anonymous",
        referrerpolicy: "no-referrer",
      },
    },
    // Google Analytics 4 (GA4) - Configure with environment variable
    // See docs/ANALYTICS/ga4-setup.md for setup instructions
    ...(process.env.GA4_MEASUREMENT_ID
      ? [
          {
            tagName: "script",
            attributes: {
              async: "true",
              src: `https://www.googletagmanager.com/gtag/js?id=${process.env.GA4_MEASUREMENT_ID}`,
            },
          },
          {
            tagName: "script",
            attributes: {},
            innerHTML: `
          window.dataLayer = window.dataLayer || [];
          function gtag(){dataLayer.push(arguments);}
          gtag('js', new Date());
          gtag('config', '${process.env.GA4_MEASUREMENT_ID}', {
            'anonymize_ip': true,
            'allow_google_signals': false,
            'allow_ad_personalization_signals': false
          });
        `,
          },
        ]
      : []),
  ],

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: "en",
    locales: ["en"],
  },

  presets: [
    [
      "classic",
      {
        docs: {
          sidebarPath: "./sidebars.ts",
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            "https://github.com/panaversity/ai-native-software-development/tree/main/website/",
        },
        blog: false,
        theme: {
          customCss: "./src/css/custom.css",
        },
        // Sitemap configuration for search engines
        sitemap: {
          changefreq: "weekly",
          priority: 0.5,
          filename: "sitemap.xml",
          ignorePatterns: ["**/tags/**"],
        },
      } satisfies Preset.Options,
    ],
  ],

  plugins: [
    "./plugins/docusaurus-plugin-og-image-generator",
    "./plugins/docusaurus-plugin-structured-data",
    function (context, options) {
      return {
        name: "custom-webpack-config",
        configureWebpack(config, isServer, utils) {
          const path = require("path");
          return {
            resolve: {
              alias: {
                "@": path.resolve(__dirname, "src"),
              },
            },
          };
        },
      };
    },
  ],

  themeConfig: {
    // Replace with your project's social card
    image: "img/book-cover-page.png",

    // Open Graph metadata for social media sharing
    metadata: [
      { property: "og:title", content: "AI Native Software Development" },
      {
        property: "og:description",
        content:
          "Colearning Agentic AI with Python and TypeScript – The AI & Spec Driven Way",
      },
      { property: "og:type", content: "website" },
      {
        property: "og:image",
        content: "https://ai-native.panaversity.org/img/book-cover-page.png",
      },
      { property: "og:image:width", content: "1200" },
      { property: "og:image:height", content: "630" },
      { property: "og:url", content: "https://ai-native.panaversity.org" },
      { name: "twitter:card", content: "summary_large_image" },
      { name: "twitter:title", content: "AI Native Software Development" },
      {
        name: "twitter:description",
        content:
          "Colearning Agentic AI with Python and TypeScript – The AI & Spec Driven Way",
      },
      {
        name: "twitter:image",
        content: "https://ai-native.panaversity.org/img/book-cover-page.png",
      },
    ],

    colorMode: {
      respectPrefersColorScheme: true,
    },
    docs: {
      sidebar: {
        hideable: true,
      },
    },
    navbar: {
      title: "AI Native Development",
      // logo: {
      //   alt: 'Panaversity Logo',
      //   src: 'img/book-cover.png',
      //   width: 32,
      //   height: 32,
      // },
      hideOnScroll: false,
      items: [
        {
          type: "docSidebar",
          sidebarId: "tutorialSidebar",
          position: "left",
          label: "Book",
        },
        {
          href: "https://github.com/panaversity/ai-native-software-development",
          label: "GitHub",
          position: "right",
        },
      ],
    },
    footer: {
      style: "dark",
      links: [
        {
          title: "Learn",
          items: [
            {
              label: "Start Your Journey",
              to: "/docs/preface-agent-native",
            },
            {
              label: "Full Curriculum",
              to: "/docs/preface-agent-native",
            },
            {
              label: "Learning Path",
              to: "/docs/preface-agent-native",
            },
          ],
        },
        {
          title: "Community",
          items: [
            {
              label: "YouTube",
              href: "https://youtube.com/@panaversity",
            },
            {
              label: "LinkedIn",
              href: "https://linkedin.com/company/panaversity",
            },
            {
              label: "Instagram",
              href: "https://instagram.com/panaversity",
            },
            {
              label: "Facebook",
              href: "https://facebook.com/panaversity",
            },
          ],
        },
        {
          title: "Resources",
          items: [
            {
              label: "GitHub Repository",
              href: "https://github.com/panaversity/ai-native-software-development",
            },
            {
              label: "AI Native Specification",
              href: "https://github.com/panaversity/ai-native-software-development/tree/main/specs",
            },
            {
              label: "Example Projects",
              href: "https://github.com/panaversity",
            },
          ],
        },
        {
          title: "About",
          items: [
            {
              label: "Panaversity",
              href: "https://panaversity.org/",
            },
            {
              label: "Our Mission",
              href: "https://panaversity.org/#about",
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} <strong>Panaversity</strong> • AI Native Software Development • Free & Open Source`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
