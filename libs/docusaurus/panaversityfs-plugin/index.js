/**
 * Docusaurus PanaversityFS Plugin
 *
 * Fetches educational content from PanaversityFS MCP server via HTTP
 * and writes it to the docsfs/ folder before Docusaurus processes it.
 *
 * Features:
 * - Fetch all book content at build time using read_content (scope=book)
 * - Write content to docsfs/ folder for Docusaurus to process
 * - Preserves directory structure from server (content/ -> docsfs/)
 * - Cleans docsfs/ folder before writing (when enabled)
 * - Filters out .summary.md files (stored in R2 but not rendered in book)
 * - Keeps docs/ separate for local sample content (no server needed)
 *
 * @param {Object} context - Docusaurus context
 * @param {Object} options - Plugin options
 */
const fs = require('fs');
const path = require('path');
const MCPHttpClient = require('./mcp-http-client');

module.exports = function panaversityFSPlugin(context, options) {
  const {
    bookId = 'ai-native-dev',
    enabled = false, // Disabled by default
    serverUrl = process.env.PANAVERSITY_SERVER_URL || 'http://localhost:8000/mcp',
    apiKey = process.env.PANAVERSITY_API_KEY || null, // API key for authenticated requests
    timeoutMs = 120000, // 2 minutes default (book fetch can be slow with large content)
    docsDir = 'docsfs', // Output directory relative to siteDir (separate from docs/)
    cleanDocsDir = true, // Clean docsfs/ before writing
    // Files matching these patterns are stored in R2 but NOT written to docsfs/
    // They remain accessible via MCP server for other purposes (AI summaries, etc.)
    excludePatterns = [
      /\.summary\.md$/,  // AI-generated summaries (e.g., lesson.summary.md)
    ],
  } = options;

  /**
   * Check if a file path should be excluded from docs/ output
   * @param {string} filePath - File path to check
   * @returns {boolean} True if file should be excluded
   */
  function shouldExclude(filePath) {
    return excludePatterns.some(pattern => pattern.test(filePath));
  }

  const siteDir = context.siteDir;
  const docsPath = path.join(siteDir, docsDir);

  // Create docsfs/ directory immediately if it doesn't exist
  // This is required because Docusaurus validates docs folder existence
  // BEFORE plugins can run loadContent()
  if (enabled && !fs.existsSync(docsPath)) {
    console.log('[PanaversityFS] Creating docsfs/ directory...');
    fs.mkdirSync(docsPath, { recursive: true });
  }

  return {
    name: 'docusaurus-panaversityfs-plugin',

    async loadContent() {
      console.log('[PanaversityFS] Plugin starting...');
      console.log(`[PanaversityFS] Book ID: ${bookId}`);
      console.log(`[PanaversityFS] Enabled: ${enabled}`);
      console.log(`[PanaversityFS] Server URL: ${serverUrl}`);
      console.log(`[PanaversityFS] Auth: ${apiKey ? 'API Key configured' : 'No auth (dev mode)'}`);
      console.log(`[PanaversityFS] Docs Path: ${docsPath}`);

      if (!enabled) {
        console.log('[PanaversityFS] Plugin disabled, using existing docs/ folder');
        return null;
      }

      // Check if docsfs/ was already hydrated by hydrate-book.py script
      // This happens in CI when the deploy workflow runs hydration before build
      const hasHydratedContent = fs.existsSync(docsPath) &&
        fs.readdirSync(docsPath).some(entry => {
          const entryPath = path.join(docsPath, entry);
          return fs.statSync(entryPath).isFile() ||
            (fs.statSync(entryPath).isDirectory() && fs.readdirSync(entryPath).length > 0);
        });

      if (hasHydratedContent && !cleanDocsDir) {
        console.log('[PanaversityFS] docsfs/ already populated (hydrated by CI), skipping fetch');
        const fileCount = fs.readdirSync(docsPath, { recursive: true })
          .filter(f => f.endsWith('.md')).length;
        return {
          totalFiles: fileCount,
          writtenFiles: fileCount,
          excludedFiles: 0,
          source: 'pre-hydrated',
          timestamp: new Date().toISOString(),
        };
      }

      // Connect to PanaversityFS MCP server via HTTP
      try {
        const client = new MCPHttpClient({ serverUrl, bookId, apiKey, timeoutMs });

        // Check server availability
        console.log('[PanaversityFS] Checking server availability...');
        const available = await client.ping();
        if (!available) {
          throw new Error(`Server not available at ${serverUrl}`);
        }
        console.log('[PanaversityFS] Server is available');

        // Fetch all content using scope=book (single request, all files)
        console.log('[PanaversityFS] Fetching all book content...');
        const allContent = await client.readBookContent(bookId);
        console.log(`[PanaversityFS] Received ${allContent.length} files from server`);

        // Clean docs/ directory if enabled
        if (cleanDocsDir && fs.existsSync(docsPath)) {
          console.log('[PanaversityFS] Cleaning docs/ directory...');
          fs.rmSync(docsPath, { recursive: true, force: true });
        }

        // Create docs/ directory
        fs.mkdirSync(docsPath, { recursive: true });

        // Write each file to docs/
        let writtenCount = 0;
        let excludedCount = 0;
        for (const file of allContent) {
          // Skip non-markdown files
          if (!file.path?.endsWith('.md')) {
            continue;
          }

          // Skip files matching exclude patterns (e.g., .summary.md)
          // These files remain in R2 storage but are not rendered in the book
          if (shouldExclude(file.path)) {
            excludedCount++;
            continue;
          }

          // Transform path: content/01-Part/01-Chapter/lesson.md -> docsfs/01-Part/01-Chapter/lesson.md
          const relativePath = file.path.replace(/^content\//, '');
          const outputPath = path.join(docsPath, relativePath);
          const outputDir = path.dirname(outputPath);

          // Create directory structure
          fs.mkdirSync(outputDir, { recursive: true });

          // Write the file
          fs.writeFileSync(outputPath, file.content || '', 'utf-8');
          writtenCount++;
        }

        console.log(`[PanaversityFS] Written ${writtenCount} files to ${docsPath}`);
        if (excludedCount > 0) {
          console.log(`[PanaversityFS] Excluded ${excludedCount} files (summary files, etc.)`);
        }

        // Return summary for contentLoaded hook
        return {
          totalFiles: allContent.length,
          writtenFiles: writtenCount,
          excludedFiles: excludedCount,
          source: 'panaversityfs-http',
          serverUrl,
          bookId,
          timestamp: new Date().toISOString(),
        };
      } catch (error) {
        console.error('[PanaversityFS] Error fetching content:', error.message);

        // Check if docsfs/ has any content (not just exists - it may be empty)
        // The directory is pre-created empty at plugin init, so we need to check for actual files
        const hasContent = fs.existsSync(docsPath) &&
          fs.readdirSync(docsPath).some(entry => {
            const entryPath = path.join(docsPath, entry);
            return fs.statSync(entryPath).isFile() ||
              (fs.statSync(entryPath).isDirectory() && fs.readdirSync(entryPath).length > 0);
          });

        if (!hasContent) {
          // docsfs/ is empty or doesn't exist - this is a fatal error
          // The build cannot proceed without content
          throw new Error(
            `PanaversityFS failed to fetch content and docsfs/ folder is empty. ` +
              `Server error: ${error.message}. ` +
              `Either disable the plugin (PANAVERSITY_PLUGIN_ENABLED=false) or ensure the server is running at ${serverUrl}`
          );
        }

        // docsfs/ has content from a previous build, warn and continue
        console.warn('[PanaversityFS] Using existing docsfs/ folder as fallback');
        return {
          totalFiles: 0,
          writtenFiles: 0,
          source: 'fallback-existing',
          error: error.message,
          timestamp: new Date().toISOString(),
        };
      }
    },

    async contentLoaded({ content, actions }) {
      if (!content || !enabled) {
        return;
      }

      const { createData } = actions;

      // Store build metadata
      await createData('panaversityfs-build.json', JSON.stringify(content, null, 2));

      console.log('[PanaversityFS] Build metadata saved');
      console.log(`[PanaversityFS] Source: ${content.source}`);
      console.log(`[PanaversityFS] Files written: ${content.writtenFiles}`);
    },
  };
};
