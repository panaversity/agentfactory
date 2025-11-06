/**
 * Content Extractor Utility
 * MDX DOM traversal to extract page content for AI context
 */

/**
 * Extract page content from MDX-rendered DOM
 * Uses .markdown selector to target Docusaurus content area
 * 
 * @returns Object with title, headings, and main content
 */
export function extractPageContent(): {
  title: string;
  headings: Array<{ level: number; text: string; id?: string }>;
  content: string;
  wordCount: number;
} {
  // Extract title from h1 or document title
  const titleElement = document.querySelector('article h1');
  const title = titleElement?.textContent?.trim() || document.title;

  // Extract headings (h2-h6) with IDs for section navigation
  const headingElements = document.querySelectorAll('article h2, article h3, article h4, article h5, article h6');
  const headings = Array.from(headingElements).map(el => ({
    level: parseInt(el.tagName[1]),
    text: el.textContent?.trim() || '',
    id: el.id || undefined,
  }));

  // Extract main content from article or .markdown selector
  const contentElement = document.querySelector('article.markdown') || 
                         document.querySelector('article') ||
                         document.querySelector('.markdown') ||
                         document.querySelector('main');

  let content = '';
  
  if (contentElement) {
    // Clone the element to avoid modifying DOM
    const clone = contentElement.cloneNode(true) as HTMLElement;

    // Remove only navigation and UI elements (keep code blocks for context!)
    clone.querySelectorAll('nav, .table-of-contents, button, .pagination-nav, .theme-doc-footer').forEach(el => el.remove());

    // Get text content, preserving structure
    content = clone.textContent?.trim() || '';
    
    // If content is empty or too short, try alternative selectors
    if (content.length < 100) {
      const docContent = document.querySelector('.theme-doc-markdown') || 
                        document.querySelector('[class*="docItemContainer"]');
      if (docContent) {
        content = docContent.textContent?.trim() || '';
      }
    }
  }

  // Clean up whitespace (multiple newlines, tabs, etc.)
  content = content.replace(/\n{3,}/g, '\n\n').replace(/\t/g, ' ').trim();

  // Debug logging
  console.log('[ContentExtractor] 📝 Extracted content:', {
    title,
    headingsCount: headings.length,
    contentLength: content.length,
    contentPreview: content.substring(0, 200) + '...',
    url: window.location.pathname,
  });

  // Calculate word count (approximate)
  const wordCount = content.split(/\s+/).filter(word => word.length > 0).length;

  return {
    title,
    headings,
    content,
    wordCount,
  };
}

/**
 * Extract specific section content by heading ID
 * Useful for targeted explanations
 * 
 * @param sectionId The ID of the heading element
 * @returns Section content between this heading and next heading
 */
export function extractSectionContent(sectionId: string): string | null {
  const heading = document.getElementById(sectionId);
  if (!heading) return null;

  let content = '';
  let currentElement = heading.nextElementSibling;

  // Collect content until next heading of same or higher level
  const headingLevel = parseInt(heading.tagName[1]);

  while (currentElement) {
    // Stop at next heading of same or higher level
    if (currentElement.tagName.match(/^H[1-6]$/)) {
      const nextLevel = parseInt(currentElement.tagName[1]);
      if (nextLevel <= headingLevel) break;
    }

    content += currentElement.textContent?.trim() + '\n\n';
    currentElement = currentElement.nextElementSibling;
  }

  return content.trim();
}

/**
 * Extract surrounding context for a text selection
 * Useful for the "Explain" feature to provide context
 * 
 * @param selectedText The highlighted text
 * @param contextWords Number of words before/after to include (default: 50)
 * @returns Surrounding context
 */
export function extractSurroundingContext(
  selectedText: string,
  contextWords: number = 50
): string {
  console.log('[contentExtractor] Extracting context for:', selectedText.substring(0, 50) + '...');
  
  const pageContent = extractPageContent();
  const content = pageContent.content;

  if (!content || content.length === 0) {
    console.warn('[contentExtractor] Empty page content');
    return selectedText; // Return just the selected text if no content
  }

  // Normalize whitespace for better matching
  const normalizedContent = content.replace(/\s+/g, ' ').trim();
  const normalizedSelection = selectedText.replace(/\s+/g, ' ').trim();
  
  const index = normalizedContent.indexOf(normalizedSelection);

  if (index === -1) {
    console.warn('[contentExtractor] Selection not found in page content, returning selection only');
    return selectedText; // Selection not found in extracted content
  }

  // Find start position (contextWords before)
  const before = normalizedContent.substring(0, index);
  const beforeWords = before.split(/\s+/).filter(w => w.length > 0).slice(-contextWords);
  
  // Find end position (contextWords after)
  const after = normalizedContent.substring(index + normalizedSelection.length);
  const afterWords = after.split(/\s+/).filter(w => w.length > 0).slice(0, contextWords);

  const context = [
    ...beforeWords,
    normalizedSelection,
    ...afterWords,
  ].join(' ');

  console.log('[contentExtractor] Context extracted:', {
    beforeWords: beforeWords.length,
    afterWords: afterWords.length,
    totalLength: context.length,
  });

  return context;
}

/**
 * Get current page metadata
 * @returns Page URL, title, and canonical URL if available
 */
export function getPageMetadata(): {
  url: string;
  title: string;
  canonicalUrl?: string;
} {
  const url = window.location.pathname;
  const title = document.title;
  const canonicalLink = document.querySelector<HTMLLinkElement>('link[rel="canonical"]');
  const canonicalUrl = canonicalLink?.href;

  return {
    url,
    title,
    canonicalUrl,
  };
}

/**
 * Check if current page is a documentation/chapter page
 * (Not homepage, not search, not 404, etc.)
 * @returns true if documentation page
 */
export function isDocumentationPage(): boolean {
  const url = window.location.pathname;
  
  // Exclude special pages
  if (url === '/' || url === '/search' || url.includes('/404')) {
    return false;
  }

  // Check if article content exists
  return document.querySelector('article') !== null;
}
