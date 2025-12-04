/**
 * Docusaurus Summaries Plugin
 *
 * Reads all .summary.md files at build time and makes them available
 * to React components via global data (useGlobalData hook).
 *
 * This enables the DocItem/Content wrapper to show AI Summary tabs
 * for lessons that have corresponding .summary.md files.
 *
 * Usage:
 * - Add to plugins array in docusaurus.config.ts
 * - Access in components via: useGlobalData()['docusaurus-summaries-plugin']
 */

const fs = require('fs');
const path = require('path');
const glob = require('glob');

/**
 * Normalize a file path to match Docusaurus doc ID format.
 * Docusaurus strips numeric prefixes like "01-", "02-" from directory and file names.
 *
 * Example:
 * - Input: "01-Introducing-AI-Driven-Development/01-ai-development-revolution/08-traditional-cs-education-gaps"
 * - Output: "Introducing-AI-Driven-Development/ai-development-revolution/traditional-cs-education-gaps"
 */
function normalizeToDocId(filePath) {
  return filePath
    .split('/')
    .map(segment => segment.replace(/^\d+-/, ''))
    .join('/');
}

module.exports = function summariesPlugin(context, options) {
  const {
    docsPath = 'docs', // Relative to siteDir
  } = options;

  return {
    name: 'docusaurus-summaries-plugin',

    async loadContent() {
      const docsDir = path.join(context.siteDir, docsPath);
      const summaries = {};

      // Find all .summary.md files
      const summaryFiles = glob.sync('**/*.summary.md', {
        cwd: docsDir,
        absolute: true,
      });

      console.log(`[Summaries Plugin] Found ${summaryFiles.length} summary files`);

      for (const summaryPath of summaryFiles) {
        try {
          // Read summary content
          let content = fs.readFileSync(summaryPath, 'utf-8');

          // Strip frontmatter from summary if present
          content = content.replace(/^---[\s\S]*?---\n*/, '').trim();

          if (content) {
            // Get the doc path relative to docsDir (without .summary.md)
            const relativePath = path.relative(docsDir, summaryPath);
            // Convert lesson.summary.md -> lesson (the doc ID pattern)
            const rawDocPath = relativePath.replace(/\.summary\.md$/, '');
            // Normalize to match Docusaurus doc ID (strips numeric prefixes)
            const docId = normalizeToDocId(rawDocPath);

            summaries[docId] = content;
            console.log(`[Summaries Plugin] Loaded summary for: ${docId} (from ${rawDocPath})`);
          }
        } catch (err) {
          console.warn(`[Summaries Plugin] Failed to read ${summaryPath}:`, err.message);
        }
      }

      return summaries;
    },

    async contentLoaded({ content, actions }) {
      const { setGlobalData } = actions;

      // Store summaries in global data for components to access
      setGlobalData({
        summaries: content || {},
      });

      console.log(`[Summaries Plugin] Set global data with ${Object.keys(content || {}).length} summaries`);
    },
  };
};
