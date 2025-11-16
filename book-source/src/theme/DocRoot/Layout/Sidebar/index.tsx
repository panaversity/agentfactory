/**
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */
import React, {useState, useCallback, useEffect, type ReactNode} from 'react';
import clsx from 'clsx';
import {prefersReducedMotion, ThemeClassNames} from '@docusaurus/theme-common';
import {useDocsSidebar} from '@docusaurus/plugin-content-docs/client';
import {useLocation} from '@docusaurus/router';
import DocSidebar from '@theme/DocSidebar';
import ExpandButton from '@theme/DocRoot/Layout/Sidebar/ExpandButton';
import styles from './styles.module.css';

// Reset sidebar state when sidebar changes
// Use React key to unmount/remount the children
// See https://github.com/facebook/docusaurus/issues/3414
function ResetOnSidebarChange({children}: {children: ReactNode}) {
  const sidebar = useDocsSidebar();
  return (
    <React.Fragment key={sidebar?.name ?? 'noSidebar'}>
      {children}
    </React.Fragment>
  );
}

export default function DocRootLayoutSidebar({
  sidebar,
  hiddenSidebarContainer,
  setHiddenSidebarContainer,
}: {
  sidebar: any;
  hiddenSidebarContainer: boolean;
  setHiddenSidebarContainer: (value: boolean | ((prev: boolean) => boolean)) => void;
}) {
  const {pathname} = useLocation();
  const [hiddenSidebar, setHiddenSidebar] = useState(false);

  const toggleSidebar = useCallback(() => {
    if (hiddenSidebar) {
      setHiddenSidebar(false);
    }
    // onTransitionEnd won't fire when sidebar animation is disabled
    // fixes https://github.com/facebook/docusaurus/issues/8918
    if (!hiddenSidebar && prefersReducedMotion()) {
      setHiddenSidebar(true);
    }
    setHiddenSidebarContainer((value) => !value);
  }, [setHiddenSidebarContainer, hiddenSidebar]);

  // Listen for custom collapse/expand events
  useEffect(() => {
    const handleCollapse = () => {
      console.log('🟢 Sidebar received collapseSidebar event, hiddenSidebarContainer:', hiddenSidebarContainer);
      // Only collapse if sidebar is currently visible
      if (!hiddenSidebarContainer) {
        console.log('✅ Calling toggleSidebar to collapse');
        toggleSidebar();
      } else {
        console.log('⚠️ Sidebar already hidden, skipping');
      }
    };

    const handleExpand = () => {
      console.log('🟢 Sidebar received expandSidebar event, hiddenSidebarContainer:', hiddenSidebarContainer);
      // Only expand if sidebar is currently hidden
      if (hiddenSidebarContainer) {
        console.log('✅ Calling toggleSidebar to expand');
        toggleSidebar();
      } else {
        console.log('⚠️ Sidebar already visible, skipping');
      }
    };

    console.log('🔵 Setting up sidebar event listeners');
    window.addEventListener('collapseSidebar', handleCollapse);
    window.addEventListener('expandSidebar', handleExpand);

    return () => {
      console.log('🔴 Cleaning up sidebar event listeners');
      window.removeEventListener('collapseSidebar', handleCollapse);
      window.removeEventListener('expandSidebar', handleExpand);
    };
  }, [hiddenSidebarContainer, toggleSidebar]);

  return (
    <aside
      className={clsx(
        ThemeClassNames.docs.docSidebarContainer,
        styles.docSidebarContainer,
        hiddenSidebarContainer && styles.docSidebarContainerHidden,
      )}
      onTransitionEnd={(e) => {
        if (!e.currentTarget.classList.contains(styles.docSidebarContainer)) {
          return;
        }
        if (hiddenSidebarContainer) {
          setHiddenSidebar(true);
        }
      }}>
      <ResetOnSidebarChange>
        <div
          className={clsx(
            styles.sidebarViewport,
            hiddenSidebar && styles.sidebarViewportHidden,
          )}>
          <DocSidebar
            sidebar={sidebar}
            path={pathname}
            onCollapse={toggleSidebar}
            isHidden={hiddenSidebar}
          />
          {hiddenSidebar && <ExpandButton toggleSidebar={toggleSidebar} />}
        </div>
      </ResetOnSidebarChange>
    </aside>
  );
}
