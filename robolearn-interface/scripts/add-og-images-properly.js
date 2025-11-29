#!/usr/bin/env node

/**
 * Script to add OG image paths to markdown frontmatter
 * Run this once after generating OG images to update all markdown files
 * Uses gray-matter for proper YAML parsing
 */

const fs = require("fs");
const path = require("path");
const matter = require("gray-matter");

const docsDir = path.join(__dirname, "..", "docs");

/**
 * Recursively process markdown files
 */
function processDirectory(dir) {
  const entries = fs.readdirSync(dir, { withFileTypes: true });

  for (const entry of entries) {
    const fullPath = path.join(dir, entry.name);

    if (entry.isDirectory()) {
      processDirectory(fullPath);
    } else if (
      entry.isFile() &&
      (entry.name.endsWith(".md") || entry.name.endsWith(".mdx"))
    ) {
      processMarkdownFile(fullPath);
    }
  }
}

/**
 * Add image frontmatter to a markdown file
 */
function processMarkdownFile(filePath) {
  const content = fs.readFileSync(filePath, "utf-8");

  // Parse frontmatter using gray-matter
  let parsed;
  try {
    parsed = matter(content);
  } catch (error) {
    console.log(`  ⊘ Skipped (invalid frontmatter): ${path.basename(filePath)}`);
    return;
  }

  if (!parsed.data || Object.keys(parsed.data).length === 0) {
    console.log(`  ⊘ Skipped (no frontmatter): ${path.basename(filePath)}`);
    return;
  }

  // Check if image field already exists
  if (parsed.data.image) {
    console.log(`  ⊘ Skipped (already has image): ${path.basename(filePath)}`);
    return;
  }

  // Generate slug from file path
  const relativePath = path.relative(docsDir, filePath);
  const slug = relativePath
    .replace(/\\/g, "/")
    .replace(/\.mdx?$/, "");

  const imageFilename = slug.replace(/\//g, "-") + ".png";
  const imagePath = `/img/og/${imageFilename}`;

  // Check if the OG image actually exists
  const ogImagePath = path.join(__dirname, "..", "static", "img", "og", imageFilename);
  if (!fs.existsSync(ogImagePath)) {
    console.log(`  ⊘ Skipped (OG image not found): ${path.basename(filePath)}`);
    return;
  }

  // Add image field to frontmatter
  parsed.data.image = imagePath;

  // Stringify back to markdown with updated frontmatter
  const newContent = matter.stringify(parsed.content, parsed.data);

  // Write back to file
  fs.writeFileSync(filePath, newContent, "utf-8");
  console.log(`  ✓ Added image to: ${path.basename(filePath)}`);
}

// Main execution
console.log("\n🖼️  Adding OG image paths to markdown frontmatter...\n");
processDirectory(docsDir);
console.log("\n✅ Done!\n");
