/**
 * Learning Hub Main Component
 * Lazy loading, toggle state, tab navigation
 */

import React, { Suspense, lazy } from 'react';
import { useLearningHub } from './context/LearningHubContext';
import { LearningHubToggle } from './LearningHubToggle';
import { usePageContent } from './hooks/usePageContent';
import { useGeminiChat } from './hooks/useGeminiChat';
import styles from './styles/LearningHub.module.css';

// Lazy load tab components
const ChatInterface = lazy(() => import('./components/AIChat/ChatInterface').then(m => ({ default: m.ChatInterface })));

// Wrapper component to use the chat hook
// Using key={pageUrl} on parent ensures this remounts on page change
function ChatInterfaceWrapper({ pageContent }: { pageContent: any }) {
  // CRITICAL: Verify page content matches current URL
  const currentUrl = typeof window !== 'undefined' ? window.location.pathname : '';
  const urlMismatch = currentUrl !== pageContent.url;
  
  // Log when this component renders
  console.log('[ChatInterfaceWrapper] 🎨 Rendering:', {
    providedUrl: pageContent.url,
    currentBrowserUrl: currentUrl,
    urlMismatch: urlMismatch,
    title: pageContent.title,
    contentLength: pageContent.content?.length || 0,
    contentPreview: pageContent.content?.substring(0, 100) || 'EMPTY',
  });

  // WARNING: If there's a URL mismatch, show loading state
  if (urlMismatch) {
    console.warn('[ChatInterfaceWrapper] ⚠️ URL MISMATCH - Content may be stale!', {
      expected: currentUrl,
      received: pageContent.url,
    });
    return (
      <div style={{ padding: '20px', textAlign: 'center' }}>
        <div>Loading page content...</div>
        <small style={{ color: '#666' }}>Waiting for {currentUrl}</small>
      </div>
    );
  }

  const { messages, isLoading, error, sendMessage, retryLastMessage } = useGeminiChat(
    pageContent.url,
    pageContent.title,
    pageContent.content
  );

  // Debug: Log messages for this page
  console.log('[ChatInterfaceWrapper] 💬 Messages state:', {
    url: pageContent.url,
    messageCount: messages.length,
    messagePages: [...new Set(messages.map(m => m.pageUrl))],
  });

  return (
    <ChatInterface
      messages={messages}
      onSendMessage={sendMessage}
      isLoading={isLoading}
      error={error}
      onRetry={retryLastMessage}
    />
  );
}

export function LearningHub() {
  const { state, dispatch } = useLearningHub();
  const pageContent = usePageContent();

  // Log page content whenever it changes
  console.log('[LearningHub] 📖 Page content state:', {
    url: pageContent.url,
    title: pageContent.title,
    isValidPage: pageContent.isValidPage,
    contentLength: pageContent.content?.length || 0,
    currentBrowserUrl: typeof window !== 'undefined' ? window.location.pathname : '',
  });

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
              <ChatInterfaceWrapper 
                key={`${pageContent.url}-${pageContent.content?.length || 0}`} 
                pageContent={pageContent} 
              />
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
