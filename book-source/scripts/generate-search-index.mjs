import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';
import matter from 'gray-matter';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

const DOCS_DIR = path.join(__dirname, '../docs');
const OUTPUT_FILE = path.join(__dirname, '../static/search-index.json');

// Extract all headings from markdown content
function extractHeadings(content) {
  const headings = [];
  const lines = content.split('\n');

  for (const line of lines) {
    // Match markdown headings (## Heading, ### Heading, etc.)
    const match = line.match(/^(#{2,6})\s+(.+)$/);
    if (match) {
      const level = match[1].length; // Number of # characters
      const text = match[2].trim();

      // Clean up heading text (remove markdown formatting)
      const cleanText = text
        .replace(/\[([^\]]+)\]\([^)]+\)/g, '$1') // Remove links but keep text
        .replace(/[*_]{1,2}([^*_]+)[*_]{1,2}/g, '$1') // Remove bold/italic
        .replace(/`([^`]+)`/g, '$1') // Remove inline code formatting
        .trim();

      headings.push({
        level,
        text: cleanText,
        // Generate anchor ID (similar to how Docusaurus does it)
        id: cleanText
          .toLowerCase()
          .replace(/[^a-z0-9\s-]/g, '')
          .replace(/\s+/g, '-')
      });
    }
  }

  return headings;
}

// Generate breadcrumb from file path
function generateBreadcrumb(filePath) {
  const relativePath = path.relative(DOCS_DIR, filePath);
  const parts = relativePath.split(path.sep);

  // Remove the filename and .md/.mdx extension
  parts.pop();

  return parts
    .map(part => {
      // Clean up directory names
      return part
        .replace(/^\d+-/, '') // Remove leading numbers
        .replace(/-/g, ' ')   // Replace hyphens with spaces
        .split(' ')
        .map(word => word.charAt(0).toUpperCase() + word.slice(1)) // Capitalize
        .join(' ');
    })
    .filter(Boolean)
    .join(' › ');
}

// Generate URL from file path
function generateUrl(filePath) {
  const relativePath = path.relative(DOCS_DIR, filePath);

  // Remove file extension
  let urlPath = relativePath.replace(/\.(md|mdx)$/, '');

  // Handle index files
  if (urlPath.endsWith('/index') || urlPath.endsWith('\\index')) {
    urlPath = urlPath.replace(/[\/\\]index$/, '');
  }

  // Handle README files
  if (urlPath.endsWith('/README') || urlPath.endsWith('\\README') || urlPath.endsWith('/readme') || urlPath.endsWith('\\readme')) {
    urlPath = urlPath.replace(/[\/\\](README|readme)$/, '');
  }

  // Convert Windows paths to URL paths
  urlPath = urlPath.replace(/\\/g, '/');

  // Remove numbered prefixes from each path segment (e.g., 04-Python-Fundamentals -> Python-Fundamentals)
  urlPath = urlPath
    .split('/')
    .map(segment => segment.replace(/^\d+-/, ''))
    .join('/');

  // Return URL
  return `/docs/${urlPath}`;
}

// Recursively find all .md and .mdx files
function findMarkdownFiles(dir) {
  const files = [];

  const items = fs.readdirSync(dir);

  for (const item of items) {
    const fullPath = path.join(dir, item);
    const stat = fs.statSync(fullPath);

    if (stat.isDirectory()) {
      // Skip node_modules and hidden directories
      if (!item.startsWith('.') && item !== 'node_modules') {
        files.push(...findMarkdownFiles(fullPath));
      }
    } else if (stat.isFile()) {
      // Only include .md and .mdx files
      if (item.endsWith('.md') || item.endsWith('.mdx')) {
        files.push(fullPath);
      }
    }
  }

  return files;
}

// Parse a markdown file and extract searchable content (page + headings)
function parseMarkdownFile(filePath) {
  try {
    const content = fs.readFileSync(filePath, 'utf-8');
    const { data: frontmatter, content: markdownContent } = matter(content);

    // Extract title from frontmatter or first heading
    let title = frontmatter.title || frontmatter.sidebar_label || '';

    if (!title) {
      const firstHeading = markdownContent.match(/^#\s+(.+)$/m);
      if (firstHeading) {
        title = firstHeading[1];
      }
    }

    // If still no title, use filename
    if (!title) {
      title = path.basename(filePath, path.extname(filePath))
        .replace(/^\d+-/, '')
        .replace(/-/g, ' ')
        .split(' ')
        .map(word => word.charAt(0).toUpperCase() + word.slice(1))
        .join(' ');
    }

    const breadcrumb = generateBreadcrumb(filePath);
    const url = generateUrl(filePath);

    // Extract all headings from the page (for TOC)
    const headings = extractHeadings(markdownContent);

    const results = [];

    // 1. Add the page itself as a searchable item
    results.push({
      type: 'page',
      title,
      breadcrumb,
      description: breadcrumb, // Use breadcrumb as description for pages
      url,
      _filePath: path.relative(DOCS_DIR, filePath),
    });

    // 2. Add each heading as a separate searchable item
    for (const heading of headings) {
      results.push({
        type: 'heading',
        title: heading.text,
        breadcrumb: `${breadcrumb} › ${title}`, // Show page context
        description: `${title}`, // Show which page this heading is on
        url: `${url}#${heading.id}`, // Link to specific heading anchor
        level: heading.level,
        _filePath: path.relative(DOCS_DIR, filePath),
      });
    }

    return results;
  } catch (error) {
    console.error(`Error parsing ${filePath}:`, error.message);
    return null;
  }
}

// Main function
function generateSearchIndex() {
  console.log('Generating search index...');
  console.log(`Scanning directory: ${DOCS_DIR}`);

  // Find all markdown files
  const markdownFiles = findMarkdownFiles(DOCS_DIR);
  console.log(`Found ${markdownFiles.length} markdown files`);

  // Parse all files
  const searchIndex = [];
  let pageCount = 0;
  let headingCount = 0;

  for (const filePath of markdownFiles) {
    const parsed = parseMarkdownFile(filePath);
    if (parsed && Array.isArray(parsed)) {
      // parsed is now an array of items (page + headings)
      for (const item of parsed) {
        searchIndex.push(item);
        if (item.type === 'page') {
          pageCount++;
        } else if (item.type === 'heading') {
          headingCount++;
        }
      }
    }
  }

  console.log(`Successfully parsed ${pageCount} pages and ${headingCount} headings`);
  console.log(`Total searchable items: ${searchIndex.length}`);

  // Ensure output directory exists
  const outputDir = path.dirname(OUTPUT_FILE);
  if (!fs.existsSync(outputDir)) {
    fs.mkdirSync(outputDir, { recursive: true });
  }

  // Write search index
  fs.writeFileSync(OUTPUT_FILE, JSON.stringify(searchIndex, null, 2), 'utf-8');
  console.log(`Search index written to: ${OUTPUT_FILE}`);
}

// Run the script
try {
  generateSearchIndex();
  console.log('✅ Search index generation complete!');
} catch (error) {
  console.error('❌ Error generating search index:', error);
  process.exit(1);
}
