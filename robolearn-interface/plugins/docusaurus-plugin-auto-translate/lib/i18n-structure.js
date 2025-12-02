/**
 * i18n Structure Helper Module
 * 
 * Manages Docusaurus i18n folder structure and path mapping.
 */

const path = require('path');

/**
 * Normalize file path to match Docusaurus doc ID format.
 * Docusaurus strips numeric prefixes like "01-", "02-" from directory and file names.
 * 
 * Example:
 * - Input: "module-1/chapter-4/01-lesson.md"
 * - Output: "module-1/chapter-4/lesson"
 */
function normalizeToDocId(filePath) {
  return filePath
    .replace(/\.md$/, '') // Remove .md extension
    .split('/')
    .map(segment => segment.replace(/^\d+-/, '')) // Strip numeric prefixes
    .join('/');
}

/**
 * Map source file path to i18n target path
 * 
 * IMPORTANT: Docusaurus expects i18n files to match the ORIGINAL file structure
 * (with numeric prefixes), not the normalized doc ID structure.
 * 
 * @param {string} sourcePath - Relative path from docs/ directory (e.g., "module-1/chapter-4/01-lesson.md")
 * @param {string} targetLocale - Target locale (e.g., "ur")
 * @returns {string} Target path in i18n structure (e.g., "i18n/ur/docusaurus-plugin-content-docs/current/module-1/chapter-4/01-lesson.md")
 */
function getI18nTargetPath(sourcePath, targetLocale) {
  // Keep original path structure (don't normalize - Docusaurus matches by original file path)
  // Just ensure .md extension is present
  const docPath = sourcePath.endsWith('.md') ? sourcePath : sourcePath + '.md';
  
  // Build i18n path with original structure
  return path.join(
    'i18n',
    targetLocale,
    'docusaurus-plugin-content-docs',
    'current',
    docPath
  );
}

/**
 * Get relative path from docs directory
 */
function getRelativePathFromDocs(fullPath, docsDir) {
  return path.relative(docsDir, fullPath);
}

/**
 * Ensure i18n directory structure exists
 */
function ensureI18nStructure(siteDir, targetLocale) {
  const i18nBase = path.join(siteDir, 'i18n', targetLocale);
  const dirs = [
    i18nBase,
    path.join(i18nBase, 'docusaurus-plugin-content-docs', 'current'),
    path.join(i18nBase, 'docusaurus-theme-classic'),
  ];

  const fs = require('fs');
  dirs.forEach(dir => {
    if (!fs.existsSync(dir)) {
      fs.mkdirSync(dir, { recursive: true });
    }
  });
}

module.exports = {
  normalizeToDocId,
  getI18nTargetPath,
  getRelativePathFromDocs,
  ensureI18nStructure,
};

