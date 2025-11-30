/**
 * Docusaurus Auto-Translate Plugin
 * 
 * Automatically translates English content to Urdu during build process using Gemini API.
 * Integrates with Docusaurus i18n infrastructure.
 */

const fs = require('fs');
const path = require('path');
const glob = require('glob');

const cache = require('./lib/cache');
const fileProcessor = require('./lib/file-processor');
const i18nStructure = require('./lib/i18n-structure');
const translator = require('./lib/translator');

module.exports = function autoTranslatePlugin(context, options) {
  const {
    enabled = true,
    sourceLocale = 'en',
    targetLocales = ['ur'],
    apiProvider = 'libretranslate', // Default to free LibreTranslate
    model = 'gemini-flash-lite-latest',
    apiKey = process.env.GEMINI_API_KEY,
    libreTranslateUrl = 'https://libretranslate.com', // Free public instance
    cacheDir = '.translation-cache',
    docsPath = 'docs',
    temperature = 0.1, // Low temperature for deterministic translations (preserves code better)
  } = options;

  return {
    name: 'docusaurus-plugin-auto-translate',

    async loadContent() {
      if (!enabled) {
        console.log('[Auto-Translate] Plugin disabled');
        return;
      }

      // Check if we need API - only if translating new files
      // If all translations exist in git, we don't need API at all
      console.log('[Auto-Translate] Starting translation process...');
      console.log('[Auto-Translate] Will use existing translations from git if available');
      
      // Only require API key if using Gemini provider (for new translations)
      if (apiProvider === 'gemini' && !apiKey) {
        console.warn('[Auto-Translate] GEMINI_API_KEY not set. Will only use existing translations from git.');
        console.warn('[Auto-Translate] New/changed files will not be translated without API key.');
      }

      // LibreTranslate doesn't need API key, so we can proceed
      if (apiProvider === 'libretranslate' || (apiProvider === 'gemini' && !apiKey)) {
        console.log('[Auto-Translate] Will use LibreTranslate for new translations (free, no API key needed)');
      }

      const { siteDir } = context;
      const docsDir = path.join(siteDir, docsPath);
      const cacheDirPath = path.join(siteDir, cacheDir);

      // Ensure i18n structure exists
      targetLocales.forEach(locale => {
        i18nStructure.ensureI18nStructure(siteDir, locale);
      });

      // Find all markdown files in docs directory
      const mdFiles = glob.sync('**/*.md', {
        cwd: docsDir,
        absolute: true,
        ignore: ['**/*.summary.md', '**/node_modules/**'],
      });

      console.log(`[Auto-Translate] Found ${mdFiles.length} markdown files`);

      // Initialize translation provider
      let geminiModel = null;
      if (apiProvider === 'gemini') {
        if (!apiKey) {
          console.warn('[Auto-Translate] GEMINI_API_KEY not set, but apiProvider is "gemini". Switching to libreTranslate (free).');
          // Fallback to LibreTranslate if no API key
        } else {
          try {
            geminiModel = translator.createGeminiClient(apiKey, model, temperature);
          } catch (error) {
            console.error(`[Auto-Translate] Failed to initialize Gemini client: ${error.message}`);
            console.warn('[Auto-Translate] Falling back to LibreTranslate (free)');
          }
        }
      }
      
      if (apiProvider === 'libretranslate' || (apiProvider === 'gemini' && !geminiModel)) {
        console.log('[Auto-Translate] Using LibreTranslate (free, open-source)');
      }

      let translatedCount = 0;
      let cachedCount = 0;
      let errorCount = 0;

      // Process each target locale
      for (const targetLocale of targetLocales) {
        console.log(`[Auto-Translate] Translating to ${targetLocale}...`);

        for (const sourceFile of mdFiles) {
          try {
            const relativePath = i18nStructure.getRelativePathFromDocs(sourceFile, docsDir);
            const { frontmatter, content, original } = fileProcessor.readMarkdownFile(sourceFile);

            // Check if translated file already exists (committed to git)
            const targetPath = path.join(
              siteDir,
              i18nStructure.getI18nTargetPath(relativePath, targetLocale)
            );
            
            // If translated file exists and is newer than source, skip translation
            if (fs.existsSync(targetPath)) {
              const sourceStats = fs.statSync(sourceFile);
              const targetStats = fs.statSync(targetPath);
              
              // If translated file exists and is up-to-date, use it
              if (targetStats.mtime >= sourceStats.mtime) {
                cachedCount++;
                console.log(`[Auto-Translate] Using existing translation: ${relativePath}`);
                continue;
              }
            }
            
            // Check cache
            if (cache.isCacheValid(cacheDirPath, relativePath, original, targetLocale)) {
              // Validate cached translation for common corruption patterns
              const cachedPath = cache.getCachedTranslationPath(cacheDirPath, relativePath, targetLocale);
              if (cachedPath && fs.existsSync(cachedPath)) {
                const cachedContent = fs.readFileSync(cachedPath, 'utf8');
                // Check for common MDX/JSX corruption patterns (errors that would break build)
                const corruptionPatterns = [
                  /ReferenceError:\s*\w+\s+is not defined/i,
                  /undefined variable/i,
                  /undefined identifier/i,
                  /\bundefined\s+is not a function/i,
                ];
                const hasCorruption = corruptionPatterns.some(pattern => cachedContent.match(pattern));
                
                if (hasCorruption) {
                  console.warn(`[Auto-Translate] Detected corruption in cached translation: ${relativePath}, invalidating cache and deleting corrupted file`);
                  cache.invalidateCache(cacheDirPath, relativePath, targetLocale);
                  // Delete corrupted translation file
                  try {
                    fs.unlinkSync(cachedPath);
                  } catch (err) {
                    // Ignore if file doesn't exist
                  }
                  // Continue to re-translate below
                } else {
                  cachedCount++;
                  console.log(`[Auto-Translate] Cache hit: ${relativePath}`);
                  continue;
                }
              } else {
                // Cache says valid but file doesn't exist, invalidate cache
                cache.invalidateCache(cacheDirPath, relativePath, targetLocale);
              }
            }

            // Translate content
            console.log(`[Auto-Translate] Translating: ${relativePath}`);
            const effectiveProvider = (apiProvider === 'gemini' && !geminiModel) ? 'libretranslate' : apiProvider;
            
            // Add delay between files for LibreTranslate to avoid rate limits
            // Public API limit is ~5 requests/minute, so wait 15 seconds between files
            if (effectiveProvider === 'libretranslate' && translatedCount > 0) {
              console.log(`[Auto-Translate] Rate limiting: waiting 15 seconds before next translation...`);
              await new Promise(resolve => setTimeout(resolve, 15000));
            }
            
            const translatedContent = await translator.translateContent(
              geminiModel,
              content,
              sourceLocale,
              targetLocale,
              effectiveProvider,
              options.libreTranslateUrl || 'https://libretranslate.com'
            );

            // Write translated file (targetPath already defined above)
            fileProcessor.writeTranslatedFile(targetPath, frontmatter, translatedContent);

            // Update cache
            cache.storeCache(cacheDirPath, relativePath, original, targetLocale, targetPath);

            translatedCount++;
          } catch (error) {
            errorCount++;
            console.error(`[Auto-Translate] Error translating ${sourceFile}: ${error.message}`);
            // Continue with next file (non-blocking)
          }
        }
      }

      console.log(`[Auto-Translate] Complete: ${translatedCount} translated, ${cachedCount} cached, ${errorCount} errors`);
    },
  };
};

