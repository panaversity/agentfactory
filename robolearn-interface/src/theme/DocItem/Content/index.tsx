/**
 * DocItem/Content Theme Swizzle (Wrap)
 *
 * Wraps the original DocItem/Content component with LessonContent
 * to provide tabbed interface for Full Lesson and AI Summary views.
 *
 * The summary is read from global data (populated by docusaurus-summaries-plugin)
 * which scans for .summary.md files at build time.
 */

import React from 'react';
import Content from '@theme-original/DocItem/Content';
import type ContentType from '@theme/DocItem/Content';
import type { WrapperProps } from '@docusaurus/types';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import { usePluginData } from '@docusaurus/useGlobalData';
import LessonContent from '../../../components/LessonContent';
import ReactMarkdown from 'react-markdown';

type Props = WrapperProps<typeof ContentType>;

interface SummariesPluginData {
  summaries: Record<string, string>;
}

export default function ContentWrapper(props: Props): React.ReactElement {
  const doc = useDoc();

  // Get summaries from global data (populated by docusaurus-summaries-plugin)
  let summaries: Record<string, string> = {};
  try {
    const pluginData = usePluginData('docusaurus-summaries-plugin') as SummariesPluginData | undefined;
    summaries = pluginData?.summaries || {};
  } catch {
    // Plugin might not be loaded yet or doesn't exist
    summaries = {};
  }

  // Get the doc's source path to look up its summary
  // The sourceDirName is like "01-Introducing-AI-Driven-Development/01-ai-development-revolution"
  // The slug is the doc ID
  // The summary key is stored as relative path without .summary.md
  // e.g., "01-Introducing-AI-Driven-Development/01-ai-development-revolution/08-traditional-cs-education-gaps"

  // Build the lookup key from doc metadata
  const metadata = doc.metadata;
  const sourceDirName = metadata.sourceDirName || '';
  const slug = metadata.slug || '';

  // The source path in doc metadata points to the markdown file
  // We need to construct the summary lookup key
  // Doc ID format example: "01-Introducing-AI-Driven-Development/01-ai-development-revolution/08-traditional-cs-education-gaps"
  const docId = metadata.id;

  // Debug log in development
  if (typeof window !== 'undefined' && process.env.NODE_ENV === 'development') {
    console.log('[DocItem/Content] Doc ID:', docId);
    console.log('[DocItem/Content] Source dir:', sourceDirName);
    console.log('[DocItem/Content] Slug:', slug);
    console.log('[DocItem/Content] Available summaries:', Object.keys(summaries));
  }

  // Look up summary by doc ID (the key format matches how plugin stores them)
  const summary = summaries[docId];

  // If no summary, just render original content
  if (!summary) {
    return <Content {...props} />;
  }

  // Render summary as markdown
  const summaryElement = <ReactMarkdown>{summary}</ReactMarkdown>;

  return (
    <LessonContent summaryElement={summaryElement}>
      <Content {...props} />
    </LessonContent>
  );
}
