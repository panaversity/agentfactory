import { useDoc } from '@docusaurus/plugin-content-docs/client';
import { useEffect, useState } from 'react';

interface PageContentResult {
  content: string;
  isChapterPage: boolean;
  pagePath: string;
}

export function usePageContent(): PageContentResult {
  const { metadata } = useDoc();
  const [content, setContent] = useState<string>('');
  const [isChapterPage, setIsChapterPage] = useState<boolean>(false);

  useEffect(() => {
    const fetchContent = async () => {
      try {
        // Get the source file path from metadata
        const sourcePath = metadata.source;

        // Check if this is a chapter page (under numbered directories)
        // Pattern: docs/01-Introducing-AI-Driven-Development/...
        const isChapter = /^@site\/docs\/\d{2}-[^/]+\//.test(sourcePath);
        setIsChapterPage(isChapter);

        if (!isChapter) {
          return;
        }

        // Convert @site/docs/path to actual path
        const relativePath = sourcePath.replace('@site/', '');

        // Fetch the raw markdown content
        const response = await fetch(`/${relativePath}`);
        if (response.ok) {
          const text = await response.text();
          console.log('text', text);
          setContent(text);
        }
      } catch (error) {
        console.error('Error fetching page content:', error);
      }
    };

    fetchContent();
  }, [metadata.source]);

  // Get clean page path for summary storage
  const pagePath = metadata.source.replace('@site/', '').replace(/\\/g, '/');

  return {
    content,
    isChapterPage,
    pagePath,
  };
}
