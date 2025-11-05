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
    // Re-extract content when page changes
    const handleRouteChange = () => {
      const metadata = getPageMetadata();
      const content = extractPageContent();
      const isValid = isDocumentationPage();

      setPageContent({
        url: metadata.url,
        title: metadata.title,
        content: content.content,
        headings: content.headings,
        wordCount: content.wordCount,
        isValidPage: isValid,
      });
    };

    // Listen for Docusaurus route changes
    window.addEventListener('popstate', handleRouteChange);
    
    // Also listen for DOM mutations (content loaded asynchronously)
    const observer = new MutationObserver(() => {
      // Debounce content extraction
      setTimeout(handleRouteChange, 100);
    });

    const articleElement = document.querySelector('article');
    if (articleElement) {
      observer.observe(articleElement, {
        childList: true,
        subtree: true,
      });
    }

    return () => {
      window.removeEventListener('popstate', handleRouteChange);
      observer.disconnect();
    };
  }, []);

  return pageContent;
}
