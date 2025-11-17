import React, {useEffect, useRef} from 'react';
import clsx from 'clsx';
import TOCItems from '@theme/TOCItems';
import type { TOCItemsProps } from '@theme/TOCItems';
import styles from './styles.module.css';
import { useBookmarks } from '@/contexts/BookmarkContext';
import type { TocMode } from '@/contexts/BookmarkContext';

export type TOCProps = TOCItemsProps & {
  className?: string;
};

export default function TOC({ className, ...props }: TOCProps): JSX.Element | null {
  const { toc } = props;
  const { hideTOC, tocMode, setTocMode } = useBookmarks();
  const tocBodyRef = useRef<HTMLDivElement | null>(null);
  const lastActiveRef = useRef<HTMLElement | null>(null);

  if (!toc || toc.length === 0) {
    return null;
  }

  // When the right drawer explicitly hides the TOC, we respect that
  // and keep the TOC panel hidden.
  const effectiveMode: TocMode = hideTOC ? 'hidden' : tocMode;
  const isVisible = effectiveMode !== 'hidden';

  // Keep the active TOC item in view as the user scrolls the main content.
  // We use requestAnimationFrame polling instead of a scroll event so we
  // always react after Docusaurus updates the active link classes.
  useEffect(() => {
    if (!isVisible) {
      return;
    }

    let frameId: number;

    const tick = () => {
      const container = tocBodyRef.current;
      if (container) {
        const activeLink = container.querySelector<HTMLElement>(
          '.table-of-contents__link--active',
        );

        if (activeLink && activeLink !== lastActiveRef.current) {
          lastActiveRef.current = activeLink;

          const containerRect = container.getBoundingClientRect();
          const activeRect = activeLink.getBoundingClientRect();

          const padding = 16; // keep a small buffer around the active item
          const isAbove = activeRect.top < containerRect.top + padding;
          const isBelow = activeRect.bottom > containerRect.bottom - padding;

          if (isAbove || isBelow) {
            activeLink.scrollIntoView({block: 'center'});
          }
        }
      }

      frameId = window.requestAnimationFrame(tick);
    };

    frameId = window.requestAnimationFrame(tick);

    return () => {
      window.cancelAnimationFrame(frameId);
      lastActiveRef.current = null;
    };
  }, [isVisible]);

  if (!isVisible) {
    // When TOC is hidden, return null - the toggle is now in the navbar
    return null;
  }

  // Visible state: full TOC panel with headings and scroll tracking,
  // as provided by Docusaurus TOCItems.
  return (
    <div
      className={clsx(
        styles.tocContainer,
        styles.tocVisible,
        'table-of-contents',
        className,
      )}
    >
      <div className={styles.tocHeader}>
        <span className={styles.tocTitle}>Table of Contents</span>
      </div>

      <div className={styles.tocBody} ref={tocBodyRef}>
        <TOCItems {...props} />
      </div>
    </div>
  );
}

