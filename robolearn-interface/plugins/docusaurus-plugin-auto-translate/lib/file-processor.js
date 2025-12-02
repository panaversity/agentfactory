/**
 * File Processor Module
 * 
 * Handles file I/O, frontmatter parsing, and content extraction.
 */

const fs = require('fs');
const path = require('path');
const matter = require('gray-matter');

/**
 * Read markdown file and parse frontmatter
 */
function readMarkdownFile(filePath) {
  const content = fs.readFileSync(filePath, 'utf8');
  const parsed = matter(content);
  
  return {
    frontmatter: parsed.data,
    content: parsed.content,
    original: content, // Full original content for hashing
  };
}

/**
 * Write translated markdown file with preserved frontmatter
 */
function writeTranslatedFile(targetPath, frontmatter, translatedContent) {
  // Ensure directory exists
  const dir = path.dirname(targetPath);
  if (!fs.existsSync(dir)) {
    fs.mkdirSync(dir, { recursive: true });
  }

  // Reconstruct file with frontmatter + translated content
  const frontmatterString = matter.stringify('', frontmatter);
  const fullContent = frontmatterString + '\n' + translatedContent;
  
  fs.writeFileSync(targetPath, fullContent, 'utf8');
}

/**
 * Extract translatable content (excluding code blocks)
 * This is a simplified version - full implementation would parse markdown AST
 */
function extractTranslatableContent(content) {
  // For now, return content as-is
  // In full implementation, would extract text nodes, preserve code blocks
  return content;
}

/**
 * Reconstruct content with translated text and preserved code blocks
 * This is a simplified version - full implementation would merge translations back
 */
function reconstructContent(originalContent, translatedText) {
  // For now, return translated text
  // In full implementation, would merge translated text with preserved code blocks
  return translatedText;
}

module.exports = {
  readMarkdownFile,
  writeTranslatedFile,
  extractTranslatableContent,
  reconstructContent,
};

