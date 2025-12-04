import React, { ReactNode } from 'react';
import ContentGate from '@/components/ContentGate';
import styles from './GatedSummary.module.css';

interface GatedSummaryProps {
  children: ReactNode;
  /** Optional title for the summary section */
  title?: string;
  /** Override the default gate title */
  gateTitle?: string;
  /** Override the default gate description */
  gateDescription?: string;
}

/**
 * GatedSummary - A summary section wrapped with authentication gate
 *
 * Users must be signed in to access the summary content.
 * When not authenticated, shows a preview with sign-in prompt.
 *
 * Usage in MDX:
 * ```mdx
 * <GatedSummary title="Key Takeaways">
 *   - Point 1
 *   - Point 2
 *   - Point 3
 * </GatedSummary>
 * ```
 */
export function GatedSummary({
  children,
  title = "Lesson Summary",
  gateTitle,
  gateDescription,
}: GatedSummaryProps) {
  return (
    <ContentGate
      type="summary"
      title={gateTitle}
      description={gateDescription}
    >
      <div className={styles.summaryContainer}>
        <div className={styles.summaryHeader}>
          <svg className={styles.summaryIcon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path strokeLinecap="round" strokeLinejoin="round" d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z" />
          </svg>
          <h3 className={styles.summaryTitle}>{title}</h3>
        </div>
        <div className={styles.summaryContent}>
          {children}
        </div>
      </div>
    </ContentGate>
  );
}

export default GatedSummary;
