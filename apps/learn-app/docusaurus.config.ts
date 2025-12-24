import { themes as prismThemes } from "prism-react-renderer";
import type { Config } from "@docusaurus/types";
import type * as Preset from "@docusaurus/preset-classic";
import * as dotenv from "dotenv";

// Load environment variables from .env file (for local development)
// Production uses actual environment variables set in CI/CD
dotenv.config();

// Auth server URL for login/signup redirects
const AUTH_URL = process.env.AUTH_URL || "http://localhost:3001";

// OAuth client ID - use the pre-configured trusted client (PKCE + JWKS)
// This matches the trustedClients configuration in auth-server
const OAUTH_CLIENT_ID = process.env.OAUTH_CLIENT_ID || "agent-factory-public-client";

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

// Content Source:
// - Default: Read from docs/ (Git is source of truth, authors write here)
// - With R2_HYDRATE_ENABLED=true: Read from build-source/ (hydrated from R2 by CI)
//   Use this when content is authored outside this repo
const hydrateEnabled = process.env.R2_HYDRATE_ENABLED === "true";
const docsPath = hydrateEnabled ? "../build-source" : "docs";

const config: Config = {
  title: "The AI Agent Factory",
  tagline:
    "The Spec-Driven Blueprint for Building and Monetizing Digital FTEs",
  favicon: "favicon.png",

  // Custom fields accessible via useDocusaurusContext().siteConfig.customFields
  customFields: {
    authUrl: AUTH_URL,
    oauthClientId: OAUTH_CLIENT_ID,
  },

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: "https://agentfactory.panaversity.org",
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
    // Favicon and Apple Touch Icon
    {
      tagName: "link",
      attributes: {
        rel: "icon",
        type: "image/png",
        sizes: "32x32",
        href: "/favicon.png",
      },
    },
    {
      tagName: "link",
      attributes: {
        rel: "icon",
        type: "image/png",
        sizes: "16x16",
        href: "/favicon-16.png",
      },
    },
    {
      tagName: "link",
      attributes: {
        rel: "apple-touch-icon",
        sizes: "180x180",
        href: "/apple-touch-icon.png",
      },
    },
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
    // Google Fonts: Inter (UI/Body), JetBrains Mono (Code)
    {
      tagName: "link",
      attributes: {
        rel: "preconnect",
        href: "https://fonts.googleapis.com",
      },
    },
    {
      tagName: "link",
      attributes: {
        rel: "preconnect",
        href: "https://fonts.gstatic.com",
        crossorigin: "anonymous",
      },
    },
    {
      tagName: "link",
      attributes: {
        rel: "stylesheet",
        href: "https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&family=JetBrains+Mono:wght@400;500&display=swap",
      },
    },
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
          path: docsPath, // 'docs' (local) or 'docsfs' (from MCP server)
          sidebarPath: "./sidebars.ts",
          // Exclude .summary.md files from being rendered as pages
          // They are injected into lesson frontmatter by the summary injector plugin
          exclude: ["**/*.summary.md"],
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          // beforeDefaultRemarkPlugins run BEFORE Docusaurus's internal plugins
          // This is critical for modifying frontmatter via file.data.frontMatter
          beforeDefaultRemarkPlugins: [
            // Summary injection handled by docusaurus-summaries-plugin (global data approach)
          ],
          remarkPlugins: [
            // Required for :::directive syntax (os-tabs, admonitions, etc.)
            require('remark-directive'),
            // OS-specific tabs: :::os-tabs with ::windows ::macos ::linux
            require('../../libs/docusaurus/remark-os-tabs'),
            // Auto-transform Python code blocks into interactive components
            [require('../../libs/docusaurus/remark-interactive-python'), {
              includePaths: ['/05-Python-Fundamentals/'],
              excludeMeta: ['nointeractive', 'static'],
            }],
            // Metadata-driven content enhancements (slides, etc.)
            [require('../../libs/docusaurus/remark-content-enhancements'), {
              enableSlides: true,
              slidesConfig: {
                defaultHeight: 700,
              },
            }],
          ],
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

  themes: [
    // Local search plugin - generates search index at build time
    // We use our custom SearchBar UI, so disable the plugin's auto-injected search bar
    [
      require.resolve("@easyops-cn/docusaurus-search-local"),
      {
        hashed: true,
        language: ["en"],
        indexDocs: true,
        indexBlog: false,
        indexPages: false,
        docsRouteBasePath: "/docs",
        highlightSearchTermsOnTargetPage: true,
        searchResultLimits: 8,
        searchResultContextMaxLength: 50,
        explicitSearchResultPath: true,
        // Disable the plugin's auto-injected search bar - we use custom-searchBar instead
        searchBarShortcutHint: false,
      },
    ],
  ],
  plugins: [
    "../../libs/docusaurus/plugin-og-image",
    "../../libs/docusaurus/plugin-structured-data",
    // Summaries Plugin - Makes .summary.md content available via useGlobalData()
    [
      "../../libs/docusaurus/summaries-plugin",
      {
        docsPath: docsPath, // Use same docs path as content-docs
      },
    ],
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
    // Webpack fix for Pyodide compatibility
    // This BannerPlugin adds a global __webpack_require__ stub to prevent runtime errors when Pyodide is loaded from CDN
    function (context, options) {
      return {
        name: "pyodide-webpack-fix",
        configureWebpack(config, isServer, utils) {
          if (isServer) return {};
          return {
            plugins: [
              new (require("webpack").BannerPlugin)({
                banner: `if (typeof __webpack_require__ === 'undefined') {
                  var __webpack_require__ = {};}`,
                raw: true,
                test: /\.js$/,
              }),
            ],
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
      { property: "og:title", content: "The AI Agent Factory" },
      {
        property: "og:description",
        content:
          "The Spec-Driven Blueprint for Building and Monetizing Digital FTEs",
      },
      { property: "og:type", content: "website" },
      {
        property: "og:image",
        content: "https://agentfactory.panaversity.org/img/book-cover-page.png",
      },
      { property: "og:image:width", content: "1200" },
      { property: "og:image:height", content: "630" },
      { property: "og:url", content: "https://agentfactory.panaversity.org" },
      { name: "twitter:card", content: "summary_large_image" },
      { name: "twitter:title", content: "The AI Agent Factory" },
      {
        name: "twitter:description",
        content:
          "The Spec-Driven Blueprint for Building and Monetizing Digital FTEs",
      },
      {
        name: "twitter:image",
        content: "https://agentfactory.panaversity.org/img/book-cover-page.png",
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
      title: "Agent Factory",
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
          type: "custom-searchBar",
          position: "right",
        },
        {
          type: "custom-navbarAuth",
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
      copyright: `Copyright © ${new Date().getFullYear()} <strong>Panaversity</strong> • The AI Agent Factory • Free & Open Source`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
