/**
 * Learning Hub Main Component
 * Lazy loading, toggle state, tab navigation
 */

import React, { Suspense, lazy } from 'react';
import { useLearningHub } from './context/LearningHubContext';
import { LearningHubToggle } from './LearningHubToggle';
import { usePageContent } from './hooks/usePageContent';
import styles from './styles/LearningHub.module.css';

// Lazy load tab components
const ChatInterface = lazy(() => import('./components/AIChat/ChatInterface').then(m => ({ default: m.ChatInterface })));

export function LearningHub() {
  const { state, dispatch } = useLearningHub();
  const pageContent = usePageContent();

  // Don't render on non-documentation pages
  if (!pageContent.isValidPage) {
    return null;
  }

  const handleToggle = () => {
    dispatch({ type: 'TOGGLE_SIDEBAR' });
  };

  const handleTabChange = (tab: typeof state.activeTab) => {
    dispatch({ type: 'SET_ACTIVE_TAB', payload: tab });
  };

  return (
    <>
      <LearningHubToggle isOpen={state.isOpen} onClick={handleToggle} />

      <aside
        className={`${styles.learningHubSidebar} ${state.isOpen ? styles.isOpen : ''}`}
        aria-label="Learning Hub Sidebar"
        aria-hidden={!state.isOpen}
      >
        {/* Header */}
        <header className={styles.sidebarHeader}>
          <h2 className={styles.sidebarTitle}>Learning Hub</h2>
          <button
            className={styles.closeButton}
            onClick={handleToggle}
            aria-label="Close sidebar"
          >
            ×
          </button>
        </header>

        {/* Tab Navigation */}
        <nav className={styles.tabNavigation} role="tablist">
          <button
            className={`${styles.tabButton} ${state.activeTab === 'chat' ? styles.active : ''}`}
            onClick={() => handleTabChange('chat')}
            role="tab"
            aria-selected={state.activeTab === 'chat'}
            aria-controls="chat-panel"
          >
            💬 AI Chat
          </button>
          <button
            className={`${styles.tabButton} ${state.activeTab === 'highlights' ? styles.active : ''}`}
            onClick={() => handleTabChange('highlights')}
            role="tab"
            aria-selected={state.activeTab === 'highlights'}
            aria-controls="highlights-panel"
            disabled
          >
            ✨ Highlights
          </button>
          <button
            className={`${styles.tabButton} ${state.activeTab === 'quiz' ? styles.active : ''}`}
            onClick={() => handleTabChange('quiz')}
            role="tab"
            aria-selected={state.activeTab === 'quiz'}
            aria-controls="quiz-panel"
            disabled
          >
            📝 Quiz
          </button>
        </nav>

        {/* Content Area */}
        <div className={styles.sidebarContent} role="tabpanel" id={`${state.activeTab}-panel`}>
          <Suspense fallback={<div className={styles.loadingSpinner}>Loading...</div>}>
            {state.activeTab === 'chat' && (
              <div>Chat interface will be integrated here</div>
            )}
            {state.activeTab === 'highlights' && (
              <div>Highlights feature coming soon</div>
            )}
            {state.activeTab === 'quiz' && (
              <div>Quiz feature coming soon</div>
            )}
          </Suspense>
        </div>
      </aside>
    </>
  );
}
