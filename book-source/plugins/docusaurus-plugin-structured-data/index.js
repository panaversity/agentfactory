/**
 * Docusaurus Plugin: Structured Data (JSON-LD) Injection
 *
 * Injects Article schema JSON-LD into every docs page during SSR.
 * This ensures search engines see the structured data in initial HTML.
 *
 * Implementation uses postBuild hook to inject JSON-LD into built HTML files
 * since injectHtmlTags doesn't have access to page-specific metadata.
 */

const fs = require("fs").promises;
const path = require("path");
const { load: loadHtml } = require("cheerio");

module.exports = function structuredDataPlugin(context, options) {
  const { siteConfig, outDir } = context;

  return {
    name: "docusaurus-plugin-structured-data",

    async postBuild({ outDir, routesPaths = [], baseUrl, head }) {
      // Post-build: inject JSON-LD into all built docs HTML files
      const docsDir = path.join(outDir, "docs");

      try {
        await processDirectory(docsDir, siteConfig);
        console.log("✓ Structured data JSON-LD injected into docs pages");
      } catch (error) {
        console.warn("⚠ Failed to inject structured data:", error.message);
      }
    },
  };
};

async function processDirectory(dirPath, siteConfig) {
  try {
    const entries = await fs.readdir(dirPath, { withFileTypes: true });

    for (const entry of entries) {
      const fullPath = path.join(dirPath, entry.name);

      if (entry.isDirectory()) {
        await processDirectory(fullPath, siteConfig);
      } else if (entry.isFile() && entry.name.endsWith(".html")) {
        await injectJSONLD(fullPath, siteConfig);
      }
    }
  } catch (error) {
    // Directory might not exist or other error
    if (error.code !== "ENOENT") {
      console.warn("Warning processing directory:", error.message);
    }
  }
}

async function injectJSONLD(filePath, siteConfig) {
  try {
    const html = await fs.readFile(filePath, "utf-8");
    const $ = loadHtml(html);

    // Extract page metadata from existing HTML
    const title = $("title").text() || siteConfig.title || "";
    const description =
      $('meta[name="description"]').attr("content") || siteConfig.tagline || "";
    const canonical = $('link[rel="canonical"]').attr("href") || "";

    // Build Article JSON-LD
    const articleData = {
      "@context": "https://schema.org",
      "@type": "Article",
      headline: title,
      description: description,
      mainEntityOfPage: canonical,
    };

    // Create script tag
    const scriptTag = `<script type="application/ld+json" id="jsonld-article">${JSON.stringify(
      articleData
    )}</script>`;

    // Inject before closing </head> tag
    $("head").append(scriptTag);

    // Write modified HTML back
    await fs.writeFile(filePath, $.html(), "utf-8");
  } catch (error) {
    console.warn(`Warning injecting JSON-LD into ${filePath}:`, error.message);
  }
}
