/**
 * Remark Content Enhancements Plugin
 *
 * Composable content enhancement orchestrator that applies multiple
 * transformers based on frontmatter metadata.
 *
 * This plugin implements a 3-layer separation architecture:
 * - Layer 1: Content (frontmatter metadata - platform-agnostic)
 * - Layer 2: Build-time transformation (this plugin - portable)
 * - Layer 3: Runtime components (React - platform-specific)
 *
 * Usage in docusaurus.config.ts:
 * ```js
 * remarkPlugins: [
 *   [require('./plugins/remark-content-enhancements'), {
 *     enableSlides: true,
 *     enableInteractiveCode: true,
 *     slidesConfig: { defaultHeight: 700 },
 *     codeConfig: { excludeMeta: ['nointeractive', 'static'] }
 *   }]
 * ]
 * ```
 *
 * Supported frontmatter:
 * ```yaml
 * ---
 * slides: "slides/chapter-14.pdf"
 * # OR
 * slides:
 *   source: "https://cdn.example.com/slide.pdf"
 *   placement: "before-what-you-learn"
 *   height: 700
 *   title: "Chapter Slides"
 *
 * interactive_code: true
 * ---
 * ```
 */

const slidesTransformer = require('./transformers/slides');

function remarkContentEnhancements(options = {}) {
  const {
    enableSlides = true,
    enableInteractiveCode = false,  // Disabled by default (use existing plugin for now)
    slidesConfig = {},
    codeConfig = {},
  } = options;

  return (tree, file) => {
    // Parse frontmatter (available via file.data.frontMatter in Docusaurus - note capital M)
    // See: https://github.com/facebook/docusaurus/discussions/8759
    const frontmatter = file.data?.frontMatter || {};

    // Get slides from frontmatter
    const slides = frontmatter.slides;

    // Slides transformation (synchronous)
    if (enableSlides && slides) {
      try {
        slidesTransformer.transform(tree, file, slides, slidesConfig);
      } catch (error) {
        console.error('[Content Enhancements] Error during slides transformation:', error);
      }
    }

    // Interactive code transformation
    // Note: For now, interactive code is handled by existing remark-interactive-python plugin
    // This placeholder allows future migration to unified architecture
    if (enableInteractiveCode && frontmatter.interactive_code) {
      // TODO: Implement or integrate interactive code transformer
      console.log('[Content Enhancements] Interactive code transformation enabled (future implementation)');
    }
  };
}

module.exports = remarkContentEnhancements;
