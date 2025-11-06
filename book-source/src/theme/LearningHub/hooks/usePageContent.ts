/**
 * usePageContent Hook
 * Extract current page URL, title, MDX content, section metadata
 */

import { useState, useEffect } from 'react';
import { extractPageContent, getPageMetadata, isDocumentationPage } from '../utils/contentExtractor';

interface PageContent {
  url: string;
  title: string;
  content: string;
  headings: Array<{ level: number; text: string; id?: string }>;
  wordCount: number;
  isValidPage: boolean;
}

export function usePageContent(): PageContent {
  const [pageContent, setPageContent] = useState<PageContent>(() => {
    // Initial extraction
    const metadata = getPageMetadata();
    const content = extractPageContent();
    const isValid = isDocumentationPage();

    console.log('[usePageContent] 🚀 Initial state:', {
      url: metadata.url,
      title: metadata.title,
      contentLength: content.content.length,
    });

    return {
      url: metadata.url,
      title: metadata.title,
      content: content.content,
      headings: content.headings,
      wordCount: content.wordCount,
      isValidPage: isValid,
    };
  });

  useEffect(() => {
    let debounceTimer: NodeJS.Timeout;

    // Re-extract content when page changes
    const handleRouteChange = () => {
      // Clear previous timer
      clearTimeout(debounceTimer);
      
      // Debounce to avoid multiple extractions and wait for DOM to update
      debounceTimer = setTimeout(() => {
        const metadata = getPageMetadata();
        const content = extractPageContent();
        const isValid = isDocumentationPage();

        // CRITICAL: Validate that we have meaningful content before updating
        // This prevents updating with stale/empty content during page transitions
        if (content.content.length < 100) {
          console.warn('[usePageContent] ⚠️ Content too short, waiting for full load...', {
            url: metadata.url,
            contentLength: content.content.length,
          });
          // Retry after additional delay
          setTimeout(handleRouteChange, 200);
          return;
        }

        console.log('[usePageContent] 📄 Page content updated:', {
          url: metadata.url,
          title: metadata.title,
          contentLength: content.content.length,
          wordCount: content.wordCount,
          contentPreview: content.content.substring(0, 150) + '...',
        });

        setPageContent({
          url: metadata.url,
          title: metadata.title,
          content: content.content,
          headings: content.headings,
          wordCount: content.wordCount,
          isValidPage: isValid,
        });
      }, 500); // Increased to 500ms for more reliable content extraction
    };

    // CRITICAL: Listen for Docusaurus-specific route changes
    // Docusaurus uses client-side routing, so we need to watch the URL
    let lastUrl = window.location.pathname;
    
    const checkUrlChange = () => {
      const currentUrl = window.location.pathname;
      if (currentUrl !== lastUrl) {
        console.log('[usePageContent] 🔄 URL changed detected:', {
          from: lastUrl,
          to: currentUrl,
        });
        lastUrl = currentUrl;
        handleRouteChange();
      }
    };

    // Check URL every 100ms (Docusaurus doesn't fire events we can catch reliably)
    const urlCheckInterval = setInterval(checkUrlChange, 100);
    
    // Listen for browser back/forward buttons
    window.addEventListener('popstate', handleRouteChange);
    
    // Also listen for DOM mutations (content loaded asynchronously)
    const observer = new MutationObserver((mutations) => {
      // React to article content changes
      const hasContentChange = mutations.some(m => 
        Array.from(m.addedNodes).some(node => 
          node.nodeType === 1 && (node as Element).tagName === 'ARTICLE'
        )
      );
      
      if (hasContentChange) {
        console.log('[usePageContent] 📰 Article content changed in DOM');
        handleRouteChange();
      }
    });

    // Observe the main content area
    const mainElement = document.querySelector('main') || document.body;
    observer.observe(mainElement, {
      childList: true,
      subtree: true,
    });

    // Trigger initial extraction after a short delay (for SSR content)
    setTimeout(handleRouteChange, 500);

    return () => {
      clearTimeout(debounceTimer);
      clearInterval(urlCheckInterval);
      window.removeEventListener('popstate', handleRouteChange);
      observer.disconnect();
    };
  }, []);

  return pageContent;
}
