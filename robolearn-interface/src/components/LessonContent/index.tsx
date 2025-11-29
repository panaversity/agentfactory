/**
 * LessonContent Component
 *
 * A refined tabbed interface for lesson content with Full Lesson and Summary views.
 * Implements "Scholarly Precision" aesthetic with Polar Night theme integration.
 *
 * The Summary tab only appears when summaryElement is provided.
 * Automatically wrapped around doc content via the DocItem/Content theme swizzle.
 */

import React, { useState, useRef, useCallback } from 'react';
import styles from './styles.module.css';

interface LessonContentProps {
  children: React.ReactNode;
  summaryElement?: React.ReactNode;
}

/**
 * Document Icon - Represents full lesson content
 */
const DocumentIcon: React.FC<{ className?: string }> = ({ className }) => (
  <svg
    className={className}
    width="18"
    height="18"
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2"
    strokeLinecap="round"
    strokeLinejoin="round"
    aria-hidden="true"
  >
    <path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z" />
    <polyline points="14,2 14,8 20,8" />
    <line x1="16" y1="13" x2="8" y2="13" />
    <line x1="16" y1="17" x2="8" y2="17" />
    <line x1="10" y1="9" x2="8" y2="9" />
  </svg>
);

/**
 * Summary Icon - Represents condensed summary content
 */
const SummaryIcon: React.FC<{ className?: string }> = ({ className }) => (
  <svg
    className={className}
    width="18"
    height="18"
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    strokeWidth="2"
    strokeLinecap="round"
    strokeLinejoin="round"
    aria-hidden="true"
  >
    <line x1="21" y1="10" x2="3" y2="10" />
    <line x1="21" y1="6" x2="3" y2="6" />
    <line x1="21" y1="14" x2="3" y2="14" />
    <line x1="21" y1="18" x2="3" y2="18" />
  </svg>
);

export const LessonContent: React.FC<LessonContentProps> = ({
  children,
  summaryElement,
}) => {
  const [activeTab, setActiveTab] = useState<'lesson' | 'summary'>('lesson');
  const contentRef = useRef<HTMLDivElement>(null);

  // If no summary available, just render children without tabs
  if (!summaryElement) {
    return <>{children}</>;
  }

  const handleTabChange = useCallback((tab: 'lesson' | 'summary') => {
    setActiveTab(tab);
    // Smooth scroll to top of content when switching tabs
    contentRef.current?.scrollIntoView({ behavior: 'smooth', block: 'start' });
  }, []);

  return (
    <div className={styles.lessonContent} ref={contentRef}>
      {/* Tab Navigation */}
      <nav className={styles.tabNav} role="tablist" aria-label="Content view">
        <button
          role="tab"
          aria-selected={activeTab === 'lesson'}
          aria-controls="panel-lesson"
          id="tab-lesson"
          tabIndex={activeTab === 'lesson' ? 0 : -1}
          className={`${styles.tab} ${activeTab === 'lesson' ? styles.tabActive : ''}`}
          onClick={() => handleTabChange('lesson')}
          onKeyDown={(e) => {
            if (e.key === 'ArrowRight') {
              handleTabChange('summary');
              document.getElementById('tab-summary')?.focus();
            }
          }}
        >
          <span className={styles.tabIcon}>
            <DocumentIcon />
          </span>
          <span className={styles.tabLabel}>Full Lesson</span>
        </button>

        <button
          role="tab"
          aria-selected={activeTab === 'summary'}
          aria-controls="panel-summary"
          id="tab-summary"
          tabIndex={activeTab === 'summary' ? 0 : -1}
          className={`${styles.tab} ${activeTab === 'summary' ? styles.tabActive : ''}`}
          onClick={() => handleTabChange('summary')}
          onKeyDown={(e) => {
            if (e.key === 'ArrowLeft') {
              handleTabChange('lesson');
              document.getElementById('tab-lesson')?.focus();
            }
          }}
        >
          <span className={styles.tabIcon}>
            <SummaryIcon />
          </span>
          <span className={styles.tabLabel}>Summary</span>
        </button>
      </nav>

      {/* Content Panels */}
      <div className={styles.panelContainer}>
        {/* Full Lesson Panel */}
        <div
          role="tabpanel"
          id="panel-lesson"
          aria-labelledby="tab-lesson"
          className={`${styles.panel} ${activeTab === 'lesson' ? styles.panelActive : ''}`}
          hidden={activeTab !== 'lesson'}
        >
          {children}
        </div>

        {/* Summary Panel */}
        <div
          role="tabpanel"
          id="panel-summary"
          aria-labelledby="tab-summary"
          className={`${styles.panel} ${activeTab === 'summary' ? styles.panelActive : ''}`}
          hidden={activeTab !== 'summary'}
        >
          <div className={styles.summaryContent}>
            <div className={styles.summaryBody}>
              {summaryElement}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default LessonContent;
