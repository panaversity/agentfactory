const fs = require("fs");
const path = require("path");
const sharp = require("sharp");

// Satori is ESM-only, we'll use dynamic import
let satori;
const initSatori = async () => {
  if (!satori) {
    satori = (await import("satori")).default;
  }
  return satori;
};

/**
 * Docusaurus plugin to automatically generate Open Graph images for each page
 */
module.exports = function (context, options) {

  return {
    name: "docusaurus-plugin-og-image-generator",

    async postBuild({ siteConfig, routesPaths, outDir, head }) {
      console.log("\n🎨 Generating Open Graph images...\n");
      // Ensure OG images are generated into the FINAL BUILD output, not static/
      const ogOutDir = path.join(outDir, "img", "og");
      if (!fs.existsSync(ogOutDir)) {
        fs.mkdirSync(ogOutDir, { recursive: true });
      }

      // Generate homepage OG image
      await generateOGImage({
        title: siteConfig.title,
        description: siteConfig.tagline,
        slug: 'home',
        ogDir: ogOutDir,
        siteConfig,
      });

      // Read all docs from the docs directory
      const docsDir = path.join(context.siteDir, "docs");
      await generateImagesFromDirectory(docsDir, ogOutDir, siteConfig, docsDir);

      // Inject OG image meta tags into built HTML files
      await injectOGImagesIntoHTML(outDir, siteConfig, context.siteDir);

      console.log("\n✅ Open Graph images generated and injected successfully!\n");
    },
  };
};

/**
 * Recursively scan docs directory and generate images
 */
async function generateImagesFromDirectory(dir, ogDir, siteConfig, docsRoot) {
  const entries = fs.readdirSync(dir, { withFileTypes: true });

  for (const entry of entries) {
    const fullPath = path.join(dir, entry.name);

    if (entry.isDirectory()) {
      // Recursively process subdirectories
      await generateImagesFromDirectory(fullPath, ogDir, siteConfig, docsRoot);
    } else if (
      entry.isFile() &&
      (entry.name.endsWith(".md") || entry.name.endsWith(".mdx"))
    ) {
      // Process markdown files
      const content = fs.readFileSync(fullPath, "utf-8");
      const metadata = extractFrontMatter(content);

      if (metadata.title) {
        const relativePath = path.relative(docsRoot, fullPath);
        const slug = relativePath.replace(/\\/g, "/").replace(/\.mdx?$/, "");

        await generateOGImage({
          title: metadata.title,
          description: metadata.description || "",
          slug,
          ogDir,
          siteConfig,
        });
      }
    }
  }
}

/**
 * Extract front matter from markdown content
 */
function extractFrontMatter(content) {
  const frontMatterRegex = /^---\s*\n([\s\S]*?)\n---/;
  const match = content.match(frontMatterRegex);

  if (!match) {
    return {};
  }

  const frontMatter = {};
  const lines = match[1].split("\n");

  for (const line of lines) {
    const [key, ...valueParts] = line.split(":");
    if (key && valueParts.length > 0) {
      const value = valueParts
        .join(":")
        .trim()
        .replace(/^["']|["']$/g, "");
      frontMatter[key.trim()] = value;
    }
  }

  return frontMatter;
}

/**
 * Inject OG image meta tags into built HTML files
 */
async function injectOGImagesIntoHTML(outDir, siteConfig, siteDir) {
  console.log("\n🔧 Injecting OG images into HTML files...\n");

  const htmlFiles = [];
  
  // Recursively find all HTML files
  function findHTMLFiles(dir) {
    const entries = fs.readdirSync(dir, { withFileTypes: true });
    for (const entry of entries) {
      const fullPath = path.join(dir, entry.name);
      if (entry.isDirectory()) {
        findHTMLFiles(fullPath);
      } else if (entry.isFile() && entry.name.endsWith('.html')) {
        htmlFiles.push(fullPath);
      }
    }
  }
  
  findHTMLFiles(outDir);

  // Look for generated images inside the built output directory
  const ogImagesDir = path.join(outDir, 'img', 'og');

  // Stable version token per build, overridable from env/CI
  const buildVersion = process.env.BUILD_VERSION || String(Math.floor(Date.now() / 1000));

  for (const htmlFile of htmlFiles) {
    try {
      let html = fs.readFileSync(htmlFile, 'utf-8');
      
      // Extract the path from the HTML file location
      const relativePath = path.relative(outDir, htmlFile);
      
      // Convert HTML path to slug (similar to how we generate images)
      let slug = relativePath
        .replace(/\\/g, '/')
        .replace(/\.html$/, '')
        .replace(/\/index$/, ''); // Remove trailing /index
      
      // Handle special cases
      if (slug === 'index' || slug === '') {
        slug = 'home';
      }
      
      // For docs pages, remove the "docs/" prefix to match generated image names
      if (slug.startsWith('docs/')) {
        slug = slug.replace(/^docs\//, '');
      }
      
      // Convert to OG image filename
      const imageFilename = slug.replace(/\//g, '-') + '.png';
      const ogImagePath = path.join(ogImagesDir, imageFilename);

      // Homepage: always use the static book cover image for previews
      if (slug === 'home') {
        // Ensure a lightweight 1200x630 JPEG exists in build output for faster fetch
        try {
          const srcPng = path.join(siteDir, 'static', 'img', 'book-cover-page.png');
          const destJpg = path.join(outDir, 'img', 'book-cover-social.jpg');
          if (fs.existsSync(srcPng)) {
            await sharp(srcPng)
              .resize(1200, 630, { fit: 'cover' })
              .jpeg({ quality: 80, progressive: true, chromaSubsampling: '4:2:0' })
              .toFile(destJpg);
          }
        } catch {}

        const imageUrl = `${siteConfig.url}/img/book-cover-social.jpg?v=${buildVersion}`;
        const pageUrl = `${siteConfig.url}/`;

        // Remove existing image/url tags
        html = html.replace(/<meta[^>]*property=\"og:image\"[^>]*>/gi, '');
        html = html.replace(/<meta[^>]*name=\"twitter:image\"[^>]*>/gi, '');
        html = html.replace(/<meta[^>]*property=\"og:url\"[^>]*>/gi, '');

        const ogTags = `
  <meta property=\"og:image\" content=\"${imageUrl}\">\n  <meta property=\"og:image:width\" content=\"1200\">\n  <meta property=\"og:image:height\" content=\"630\">\n  <meta property=\"og:image:secure_url\" content=\"${imageUrl}\">\n  <meta property=\"og:site_name\" content=\"${siteConfig.title}\">\n  <meta property=\"og:url\" content=\"${pageUrl}\">\n  <meta name=\"twitter:image\" content=\"${imageUrl}\">\n</head>`;

        html = html.replace(/<\/head>/i, ogTags);
        fs.writeFileSync(htmlFile, html, 'utf-8');
        console.log(`  ✓ Injected OG image (home): book-cover-page.png`);
        continue;
      }
      
      

      // Ensure OG image exists for this page (generate on demand if missing)
      if (!fs.existsSync(ogImagePath)) {
        // Try to derive title/description from existing meta tags
        const titleMatch = html.match(/<meta[^>]*property="og:title"[^>]*content="([^"]+)"[^>]*>/i) ||
                           html.match(/<title[^>]*>([^<]+)<\/title>/i);
        const descMatch = html.match(/<meta[^>]*name="description"[^>]*content="([^"]+)"[^>]*>/i) ||
                          html.match(/<meta[^>]*property="og:description"[^>]*content="([^"]+)"[^>]*>/i);

        const title = titleMatch ? (titleMatch[1] || '').trim() : '';
        const description = descMatch ? (descMatch[1] || '').trim() : '';

        try {
          await generateOGImage({
            title: title || siteConfig.title,
            description: description || siteConfig.tagline || '',
            slug,
            ogDir: ogImagesDir,
            siteConfig,
          });
        } catch {}
      }

      if (fs.existsSync(ogImagePath)) {
        const imageUrl = `${siteConfig.url}/img/og/${imageFilename}?v=${buildVersion}`;
        // Build canonical page URL for better social parsing
        let pagePath = '';
        if (slug === 'home') {
          pagePath = '/';
        } else if (relativePath.startsWith('docs/')) {
          pagePath = `/docs/${slug}`;
        } else {
          pagePath = `/${slug}`;
        }
        const pageUrl = `${siteConfig.url}${pagePath}`;
        
        // Replace or add OG image meta tags
        // Remove existing og:image, twitter:image, and og:url tags
        html = html.replace(/<meta[^>]*property="og:image"[^>]*>/gi, '');
        html = html.replace(/<meta[^>]*name="twitter:image"[^>]*>/gi, '');
        html = html.replace(/<meta[^>]*property="og:url"[^>]*>/gi, '');
        
        // Add new OG image tags before </head>
        const ogTags = `
  <meta property="og:image" content="${imageUrl}">
  <meta property="og:image:width" content="1200">
  <meta property="og:image:height" content="630">
  <meta property="og:image:secure_url" content="${imageUrl}">
  <meta property="og:site_name" content="${siteConfig.title}">
  <meta property="og:url" content="${pageUrl}">
  <meta name="twitter:image" content="${imageUrl}">
</head>`;
        
        html = html.replace(/<\/head>/i, ogTags);
        
        // Write back
        fs.writeFileSync(htmlFile, html, 'utf-8');
        console.log(`  ✓ Injected OG image: ${imageFilename}`);
      }
    } catch (error) {
      console.log(`  ⊘ Error processing ${path.basename(htmlFile)}: ${error.message}`);
    }
  }
}

/**
 * Generate an OG image for a specific page
 */
async function generateOGImage({
  title,
  description,
  slug,
  ogDir,
  siteConfig,
}) {
  try {
    const satoriRenderer = await initSatori();
    const width = 1200;
    const height = 630;

    // Truncate title if too long
    const maxTitleLength = 60;
    const displayTitle =
      title.length > maxTitleLength
        ? title.substring(0, maxTitleLength) + "..."
        : title;

    // Truncate description
    const maxDescLength = 120;
    const displayDesc =
      description && description.length > maxDescLength
        ? description.substring(0, maxDescLength) + "..."
        : description || "";

    // Create SVG using React-like JSX syntax
    // Resolve system fonts cross-platform (macOS, Linux, Windows)
    const candidates = [
      {
        name: "Sans",
        weight: 400,
        paths: [
          "/Library/Fonts/Arial.ttf",
          "/System/Library/Fonts/Supplemental/Arial.ttf",
          "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
          "C:/Windows/Fonts/arial.ttf",
        ],
      },
      {
        name: "Sans",
        weight: 700,
        paths: [
          "/Library/Fonts/Arial Bold.ttf",
          "/System/Library/Fonts/Supplemental/Arial Bold.ttf",
          "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
          "C:/Windows/Fonts/arialbd.ttf",
        ],
      },
    ];

    const resolvedFonts = candidates
      .map((c) => {
        const found = c.paths.find((p) => {
          try {
            return fs.existsSync(p);
          } catch {
            return false;
          }
        });
        if (!found) return null;
        return {
          name: c.name,
          data: fs.readFileSync(found),
          weight: c.weight,
          style: "normal",
        };
      })
      .filter(Boolean);

    if (!resolvedFonts.length) {
      throw new Error(
        "No suitable system fonts found for Satori (tried Arial/DejaVu/Windows Arial)."
      );
    }

    const svg = await satoriRenderer(
      {
        type: "div",
        props: {
          style: {
            height: "100%",
            width: "100%",
            display: "flex",
            flexDirection: "column",
            alignItems: "flex-start",
            justifyContent: "space-between",
            backgroundColor: "#1a1a2e",
            backgroundImage:
              "linear-gradient(135deg, #1a1a2e 0%, #16213e 50%, #0f3460 100%)",
            padding: 60,
            fontFamily: "Sans",
            position: "relative",
          },
          children: [
            // Decorative circles
            {
              type: "div",
              props: {
                style: {
                  position: "absolute",
                  top: -150,
                  right: -150,
                  width: 400,
                  height: 400,
                  borderRadius: "50%",
                  backgroundColor: "rgba(255, 255, 255, 0.03)",
                },
              },
            },
            {
              type: "div",
              props: {
                style: {
                  position: "absolute",
                  bottom: -100,
                  left: -100,
                  width: 300,
                  height: 300,
                  borderRadius: "50%",
                  backgroundColor: "rgba(255, 255, 255, 0.03)",
                },
              },
            },
            // Content
            {
              type: "div",
              props: {
                style: {
                  display: "flex",
                  flexDirection: "column",
                  gap: 40,
                },
                children: [
                  // Brand
                  {
                    type: "div",
                    props: {
                      style: {
                        fontSize: 24,
                        fontWeight: "bold",
                        color: "#5ee0e4",
                      },
                      children: "Panaversity • AI Native Development",
                    },
                  },
                  // Title
                  {
                    type: "div",
                    props: {
                      style: {
                        fontSize: 56,
                        fontWeight: "bold",
                        color: "#ffffff",
                        lineHeight: 1.2,
                        maxWidth: 1000,
                      },
                      children: displayTitle,
                    },
                  },
                  // Description
                  displayDesc && {
                    type: "div",
                    props: {
                      style: {
                        fontSize: 28,
                        color: "rgba(255, 255, 255, 0.7)",
                        lineHeight: 1.4,
                        maxWidth: 1000,
                      },
                      children: displayDesc,
                    },
                  },
                ].filter(Boolean),
              },
            },
            // Footer
            {
              type: "div",
              props: {
                style: {
                  display: "flex",
                  alignItems: "center",
                  justifyContent: "space-between",
                  width: "100%",
                  paddingTop: 30,
                  borderTop: "4px solid #5ee0e4",
                },
                children: [
                  {
                    type: "div",
                    props: {
                      style: {
                        fontSize: 28,
                        fontWeight: "bold",
                        color: "#5ee0e4",
                      },
                      children: "ai-native.panaversity.org",
                    },
                  },
                ],
              },
            },
          ],
        },
      },
      {
        width,
        height,
        fonts: resolvedFonts,
      }
    );

    // Convert SVG to PNG using Sharp
    const pngBuffer = await sharp(Buffer.from(svg)).png().toBuffer();

    // Save the image
    const filename = slug.replace(/\//g, "-") + ".png";
    const filepath = path.join(ogDir, filename);
    fs.writeFileSync(filepath, pngBuffer);

    console.log(`  ✓ Generated: ${filename}`);
    return filename;
  } catch (error) {
    console.error(
      `  ✗ Failed to generate image for "${title}":`,
      error.message
    );
    return null;
  }
}
