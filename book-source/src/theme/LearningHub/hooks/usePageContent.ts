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
      
      // Debounce to avoid multiple extractions
      debounceTimer = setTimeout(() => {
        const metadata = getPageMetadata();
        const content = extractPageContent();
        const isValid = isDocumentationPage();

        console.log('[usePageContent] Page changed:', {
          url: metadata.url,
          title: metadata.title,
          contentLength: content.content.length,
          wordCount: content.wordCount,
        });

        setPageContent({
          url: metadata.url,
          title: metadata.title,
          content: content.content,
          headings: content.headings,
          wordCount: content.wordCount,
          isValidPage: isValid,
        });
      }, 300); // Wait 300ms for content to fully load
    };

    // Listen for Docusaurus route changes
    window.addEventListener('popstate', handleRouteChange);
    
    // Also listen for DOM mutations (content loaded asynchronously)
    const observer = new MutationObserver((mutations) => {
      // Only react to significant changes (new article content)
      const hasContentChange = mutations.some(m => 
        Array.from(m.addedNodes).some(node => 
          node.nodeType === 1 && (node as Element).tagName === 'ARTICLE'
        )
      );
      
      if (hasContentChange) {
        handleRouteChange();
      }
    });

    // Observe the main content area
    const mainElement = document.querySelector('main') || document.body;
    observer.observe(mainElement, {
      childList: true,
      subtree: true,
    });

    // Also trigger initial extraction after a short delay (for SSR content)
    setTimeout(handleRouteChange, 500);

    return () => {
      clearTimeout(debounceTimer);
      window.removeEventListener('popstate', handleRouteChange);
      observer.disconnect();
    };
  }, []);

  return pageContent;
}
