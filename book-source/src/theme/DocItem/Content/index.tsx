import React, {type ReactNode} from 'react';
import clsx from 'clsx';
import {ThemeClassNames} from '@docusaurus/theme-common';
import {useDoc} from '@docusaurus/plugin-content-docs/client';
import Heading from '@theme/Heading';
import MDXContent from '@theme/MDXContent';
import type {Props} from '@theme/DocItem/Content';
import ContentTabs from '@site/src/components/ContentTabs';

/**
 Title can be declared inside md content or declared through
 front matter and added manually. To make both cases consistent,
 the added title is added under the same div.markdown block
 See https://github.com/facebook/docusaurus/pull/4882#issuecomment-853021120

 We render a "synthetic title" if:
 - user doesn't ask to hide it with front matter
 - the markdown content does not already contain a top-level h1 heading
*/
function useSyntheticTitle(): string | null {
  const {metadata, frontMatter, contentTitle} = useDoc();
  const shouldRender =
    !frontMatter.hide_title && typeof contentTitle === 'undefined';
  if (!shouldRender) {
    return null;
  }
  return metadata.title;
}

export default function DocItemContent({children}: Props): ReactNode {
  const syntheticTitle = useSyntheticTitle();
  const {metadata} = useDoc();
  
  // Generate pageId from the document's permalink or id
  const pageId = metadata.id || metadata.permalink;
  
  // Determine if this is a lesson page (show tabs ONLY on lesson-level pages)
  // Structure: Part/Chapter/Lesson (3 segments)
  // Example: "01-Introducing-AI-Driven-Development/01-ai-development-revolution/01-moment_that_changed_everything"
  const isLessonPage = (() => {
    const path = metadata.id || '';
    
    // Split path and filter empty segments
    const segments = path.split('/').filter(s => s.length > 0);
    
    // Lesson pages have exactly 3 segments: part/chapter/lesson
    // Exclude README.md or index files (they're part/chapter overviews)
    const isReadmeOrIndex = path.toLowerCase().includes('readme') || 
                           path.toLowerCase().includes('index') ||
                           path.toLowerCase().endsWith('/');
    
    return segments.length === 3 && !isReadmeOrIndex;
  })();
  
  return (
    <div className={clsx(ThemeClassNames.docs.docMarkdown, 'markdown')}>
      {syntheticTitle && (
        <header>
          <Heading as="h1">{syntheticTitle}</Heading>
        </header>
      )}
      {isLessonPage ? (
        <ContentTabs pageId={pageId}>
          <MDXContent>{children}</MDXContent>
        </ContentTabs>
      ) : (
        <MDXContent>{children}</MDXContent>
      )}
    </div>
  );
}
