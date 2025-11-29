import React from 'react';

interface PDFViewerProps {
  /** Path to PDF file relative to /static directory (e.g., "slides/chapter-01-slides.pdf") */
  src: string;
  /** Optional title for accessibility and download filename */
  title?: string;
  /** Height of viewer in pixels (default: 600) */
  height?: number;
  /** Whether to show download link (default: true) */
  showDownload?: boolean;
}

/**
 * PDFViewer component for embedding PDF slides in Docusaurus pages.
 *
 * Uses Mozilla's PDF.js for cross-browser compatibility.
 *
 * @example
 * ```tsx
 * import PDFViewer from '@site/src/components/PDFViewer';
 *
 * <PDFViewer
 *   src="slides/chapter-01-slides.pdf"
 *   title="Chapter 1: AI Development Revolution"
 *   height={700}
 * />
 * ```
 */
export default function PDFViewer({
  src,
  title = 'PDF Document',
  height = 600,
  showDownload = true,
}: PDFViewerProps): React.ReactElement {
  // Construct full URL - PDFs should be in /static directory
  const pdfUrl = src.startsWith('/') ? src : `/${src}`;

  return (
    <div className="pdf-viewer-container" style={{
      marginBottom: '1.5rem',
      backgroundColor: 'var(--ifm-background-surface-color)',
      padding: '1rem',
      borderRadius: '8px',
      border: '1px solid var(--ifm-color-emphasis-200)'
    }}>
      <div style={{
        marginBottom: '0.75rem',
        display: 'flex',
        gap: '0.5rem',
        flexWrap: 'wrap'
      }}>
        <a
          href={pdfUrl}
          target="_blank"
          rel="noopener noreferrer"
          className="button button--secondary button--sm"
        >
          🖥️ Fullscreen
        </a>
      </div>

      <iframe
        src={pdfUrl}
        width="100%"
        height={height}
        style={{
          border: '1px solid var(--ifm-color-emphasis-300)',
          borderRadius: '4px',
          backgroundColor: '#fff'
        }}
        title={title}
      />
    </div>
  );
}
