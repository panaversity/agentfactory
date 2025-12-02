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
// This matches the trustedClients[0].clientId in auth-server
const OAUTH_CLIENT_ID = process.env.OAUTH_CLIENT_ID || "robolearn-public-client";

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

// Base URL configuration - defaults to "/" for local development
const BASE_URL = process.env.BASE_URL || "/";

// ChatKit domain key (required for whitelabeled domains, optional for custom backends)
// Read from environment variable, trim whitespace, fallback to dummy value
const CHATKIT_DOMAIN_KEY = process.env.CHATKIT_DOMAIN_KEY?.trim() || "domain_pk_local_dev";

const config: Config = {
  title: "RoboLearn",
  tagline:
    "Physical AI & Humanoid Robotics – Bridging the Digital Brain and the Physical Body",
  favicon: "img/favicon.ico",

  // Custom fields accessible via useDocusaurusContext().siteConfig.customFields
  customFields: {
    authUrl: AUTH_URL,
    oauthClientId: OAUTH_CLIENT_ID,
    backendUrl: process.env.BACKEND_URL || "http://localhost:8000",
    chatkitDomainKey: CHATKIT_DOMAIN_KEY,
  },

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: "https://mjunaidca.github.io",
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  // For local development, use '/' (default)
  // For production/GitHub Pages, set BASE_URL=/robolearn/ in environment
  baseUrl: BASE_URL,

  // Sitemap is configured via the classic preset's sitemap option below

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: "mjunaidca", // Usually your GitHub org/user name.
  projectName: "robolearn", // Usually your repo name.
  trailingSlash: false,

  onBrokenLinks: "warn",

  // Add Font Awesome for social media icons
  headTags: [
    // Google Fonts preconnect for faster font loading
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
    // Preload Neural Command Center fonts (IBM Plex Mono + IBM Plex Sans)
    {
      tagName: "link",
      attributes: {
        rel: "preload",
        href: "https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500;600;700&family=IBM+Plex+Sans:wght@400;500;600;700&family=JetBrains+Mono:wght@400;500;600;700&display=swap",
        as: "style",
      },
    },
    {
      tagName: "link",
      attributes: {
        rel: "stylesheet",
        href: "https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500;600;700&family=IBM+Plex+Sans:wght@400;500;600;700&family=JetBrains+Mono:wght@400;500;600;700&display=swap",
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
    // ChatKit CDN - Required for ChatKit UI
    // Note: Docusaurus doesn't support 'strategy' attribute, script loads normally
    {
      tagName: "script",
      attributes: {
        src: "https://cdn.platform.openai.com/deployments/chatkit/chatkit.js",
        defer: "true",
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
    locales: ["en", "ur"],
    localeConfigs: {
      en: {
        label: "EN",
      },
      ur: {
        label: "UR",
        direction: "rtl",
      },
    },
  },
  
  presets: [
    [
      "classic",
      {
        docs: {
          path: "docs",
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
            // Auto-transform Python code blocks into interactive components
            [require('./plugins/remark-interactive-python'), {
              includePaths: ['/05-Python-Fundamentals/'],
              excludeMeta: ['nointeractive', 'static'],
            }],
            // Metadata-driven content enhancements (slides, etc.)
            [require('./plugins/remark-content-enhancements'), {
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

  plugins: [
    "./plugins/docusaurus-plugin-og-image-generator",
    "./plugins/docusaurus-plugin-structured-data",
    // Summaries Plugin - Makes .summary.md content available via useGlobalData()
    [
      "./plugins/docusaurus-summaries-plugin",
      {
        docsPath: "docs",
      },
    ],
    // Auto-Translate Plugin - Uses existing translations from git (committed i18n/ur files)
    // Strategy: Translate once locally with Gemini, commit i18n/ur files, CI uses them
    // Only translates if: 1) File doesn't exist, 2) Source file is newer than translation
    [
      "./plugins/docusaurus-plugin-auto-translate",
      {
        enabled: true, // Enabled - but uses existing committed translations first
        sourceLocale: "en",
        targetLocales: ["ur"],
        apiProvider: "gemini", // Use Gemini for new translations (if needed)
        model: "gemini-flash-lite-latest",
        apiKey: process.env.GEMINI_API_KEY, // Optional - only needed for new translations
        libreTranslateUrl: "https://libretranslate.com",
        cacheDir: ".translation-cache",
        docsPath: "docs",
        temperature: 0.1,
      },
    ],
    // Local Search Plugin - No external service needed, works offline
    // Note: We use a custom search bar component, so we disable the default navbar search
    [
      require.resolve("@easyops-cn/docusaurus-search-local"),
      {
        // Index all docs
        indexDocs: true,
        // Don't index blog (disabled)
        indexBlog: false,
        // Index pages
        indexPages: true,
        // Language support
        language: ["en"],
        // Search result highlighting
        highlightSearchTermsOnTargetPage: true,
        // Explicit search path
        docsRouteBasePath: "/docs",
        // Search result limit
        searchResultLimits: 8,
        // Search result context max length
        searchResultContextMaxLength: 50,
        // Disable default navbar search (we use custom component)
        // The plugin will still index content, but won't add default search bar
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
      { property: "og:title", content: "RoboLearn: Physical AI & Humanoid Robotics" },
      {
        property: "og:description",
        content:
          "Physical AI & Humanoid Robotics – Bridging the Digital Brain and the Physical Body",
      },
      { property: "og:type", content: "website" },
      {
        property: "og:image",
        content: "https://mjunaidca.github.io/robolearn/img/book-cover-page.png",
      },
      { property: "og:image:width", content: "1200" },
      { property: "og:image:height", content: "630" },
      { property: "og:url", content: "https://mjunaidca.github.io/robolearn/" },
      { name: "twitter:card", content: "summary_large_image" },
      { name: "twitter:title", content: "RoboLearn: Physical AI & Humanoid Robotics" },
      {
        name: "twitter:description",
        content:
          "Physical AI & Humanoid Robotics – Bridging the Digital Brain and the Physical Body",
      },
      {
        name: "twitter:image",
        content: "https://mjunaidca.github.io/robolearn/img/book-cover-page.png",
      },
    ],

    colorMode: {
      defaultMode: 'dark',
      disableSwitch: false,
      respectPrefersColorScheme: false,
    },
    docs: {
      sidebar: {
        hideable: true,
      },
    },
    navbar: {
      title: "RoboLearn",
      // logo: {
      //   alt: 'RoboLearn Logo',
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
          label: "Learn Free",
        },
        {
          to: "/labs",
          label: "Labs",
          position: "left",
        },
        {
          to: "/chat",
          label: "Personalize",
          position: "left",
        },
        {
          type: "custom-searchBar",
          position: "right",
        },
        {
          type: "custom-languageToggle",
          position: "right",
        },
        {
          type: "custom-navbarAuth",
          position: "right",
          value: `<a href="${BASE_URL}docs/preface-agent-native" class="navbar__cta-button">Start Free</a>`,
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
              label: "Course Modules",
              to: "/docs/preface-agent-native",
            },
          ],
        },
        {
          title: "Technology",
          items: [
            {
              label: "ROS 2",
              href: "https://docs.ros.org/en/humble/",
            },
            {
              label: "NVIDIA Isaac",
              href: "https://developer.nvidia.com/isaac-sim",
            },
            {
              label: "Gazebo",
              href: "https://gazebosim.org/",
            },
          ],
        },
        {
          title: "Resources",
          items: [
            {
              label: "GitHub Repository",
              href: "https://github.com/mjunaidca/robolearn",
            },
            {
              label: "Panaversity",
              href: "https://panaversity.org/",
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} <strong>RoboLearn</strong> • Physical AI & Humanoid Robotics • Free & Open Source`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
