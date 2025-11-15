/**
 * ContentTabs Component - main compound component with tab state management
 * 
 * Provides three content views: Original, Summary, and Personalized.
 * Manages active tab state with 'original' as default.
 */
import React, { useState, ReactNode } from 'react';
import { TabType } from '../../types/contentTabs';
import TabBar from './TabBar';
import OriginalTab from './OriginalTab';
import SummaryTab from './SummaryTab';
import PersonalizedTab from './PersonalizedTab';
import ErrorBoundary from './ErrorBoundary';
import styles from './styles.module.css';

interface ContentTabsProps {
  children: ReactNode;
  pageId?: string;
}

export default function ContentTabs({ children, pageId = '' }: ContentTabsProps): React.ReactElement {
  const [activeTab, setActiveTab] = useState<TabType>('original');

  const handleTabChange = (tab: TabType) => {
    setActiveTab(tab);
  };

  // Extract content as string for summary generation
  const getContentAsString = (): string => {
    // Get text content from the document
    // This extracts all text from the main article content area
    const article = document.querySelector('article');
    if (article) {
      // Get all text content, remove extra whitespace
      const text = article.innerText || article.textContent || '';
      return text.replace(/\s+/g, ' ').trim();
    }
    
    // Fallback: try to extract from children if article not found
    if (typeof children === 'string') {
      return children;
    }
    
    return '';
  };

  return (
    <ErrorBoundary>
      <div className={styles.tabsContainer}>
        <TabBar activeTab={activeTab} onTabClick={handleTabChange} />
        
        <div className={styles.tabContent}>
          {/* Conditional rendering based on active tab */}
          {activeTab === 'original' && (
            <OriginalTab>{children}</OriginalTab>
          )}
          
          {activeTab === 'summary' && (
            <SummaryTab pageId={pageId} content={getContentAsString()} />
          )}
          
          {activeTab === 'personalized' && (
            <PersonalizedTab />
          )}
        </div>
      </div>
    </ErrorBoundary>
  );
}
