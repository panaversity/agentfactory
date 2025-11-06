/**
 * Learning Hub Main Component
 * Lazy loading, toggle state, tab navigation
 */

import React, { Suspense, lazy } from 'react';
import { useLearningHub } from './context/LearningHubContext';
import { LearningHubToggle } from './LearningHubToggle';
import { usePageContent } from './hooks/usePageContent';
import { useGeminiChat } from './hooks/useGeminiChat';
import { useHighlights } from './hooks/useHighlights';
import { useKeyConcepts } from './hooks/useKeyConcepts';
import { useProgress } from './hooks/useProgress';
import { HighlightManager } from './components/SmartHighlights/HighlightManager';
import { ProgressDashboard } from './components/ProgressTracker/ProgressDashboard';
// import { HighlightRenderer } from './components/SmartHighlights/HighlightRenderer'; // Disabled - highlights only in sidebar
import styles from './styles/LearningHub.module.css';

// Lazy load tab components
const ChatInterface = lazy(() => import('./components/AIChat/ChatInterface').then(m => ({ default: m.ChatInterface })));
const HighlightsList = lazy(() => import('./components/SmartHighlights/HighlightsList').then(m => ({ default: m.HighlightsList })));
const QuizInterface = lazy(() => import('./components/QuickQuiz/QuizInterface'));
const ConceptsList = lazy(() => import('./components/KeyConcepts/ConceptsList'));

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

  // Listen for custom sendMessage events from HighlightManager
  React.useEffect(() => {
    const handleSendMessage = (event: CustomEvent) => {
      const { message } = event.detail;
      console.log('[ChatInterfaceWrapper] Received sendMessage event:', message);
      sendMessage(message);
    };

    window.addEventListener('learningHub:sendMessage', handleSendMessage as EventListener);
    
    return () => {
      window.removeEventListener('learningHub:sendMessage', handleSendMessage as EventListener);
    };
  }, [sendMessage]);

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
  const { highlights, deleteHighlight } = useHighlights(pageContent.url);
  const { concepts, isLoading: conceptsLoading, error: conceptsError, extractConcepts } = useKeyConcepts(pageContent.url);
  const { progress, elapsedTime, isCurrentPageVisited, clearProgress, updateHighlightCount, getRecentRecords } = useProgress(
    pageContent.url,
    pageContent.title,
    50 // TODO: Get actual total chapters from sidebar config
  );

  // Auto-extract concepts when concepts tab is opened
  React.useEffect(() => {
    if (state.activeTab === 'concepts' && concepts.length === 0 && !conceptsLoading && !conceptsError && pageContent.isValidPage) {
      const wordCount = pageContent.content.split(/\s+/).length;
      if (wordCount >= 200) {
        extractConcepts({
          url: pageContent.url,
          title: pageContent.title,
          content: pageContent.content,
        });
      }
    }
  }, [state.activeTab, concepts.length, conceptsLoading, conceptsError, pageContent, extractConcepts]);

  // Update progress tracker with highlight count
  React.useEffect(() => {
    updateHighlightCount(highlights.length);
  }, [highlights.length, updateHighlightCount]);

  // Log page content whenever it changes
  console.log('[LearningHub] 📖 Page content state:', {
    url: pageContent.url,
    title: pageContent.title,
    isValidPage: pageContent.isValidPage,
    contentLength: pageContent.content?.length || 0,
    highlightsCount: highlights.length,
    currentBrowserUrl: typeof window !== 'undefined' ? window.location.pathname : '',
  });

  // Don't render on non-documentation pages (moved AFTER all hooks)
  if (!pageContent.isValidPage) {
    return null;
  }

  const handleToggle = () => {
    dispatch({ type: 'TOGGLE_SIDEBAR' });
  };

  const handleTabChange = (tab: typeof state.activeTab) => {
    dispatch({ type: 'SET_ACTIVE_TAB', payload: tab });
  };

  const handleHighlightClick = (highlight: any) => {
    // Scroll to the highlighted text on the page
    const highlightElement = document.querySelector(`mark[data-highlight-id="${highlight.id}"]`) as HTMLElement;
    if (highlightElement) {
      highlightElement.scrollIntoView({ behavior: 'smooth', block: 'center' });
      
      // Flash animation
      if (highlightElement.style) {
        highlightElement.style.backgroundPosition = '0 100%';
        setTimeout(() => {
          highlightElement.style.backgroundPosition = '0 0';
        }, 500);
      }
    }
  };

  const handleConceptClick = (concept: any) => {
    // Try to scroll to section if sectionId provided
    if (concept.sectionId) {
      const element = document.getElementById(concept.sectionId);
      if (element) {
        element.scrollIntoView({ behavior: 'smooth', block: 'start' });
        return;
      }
    }

    // Fallback: search for heading with matching text
    const headings = document.querySelectorAll('h1, h2, h3, h4, h5, h6');
    for (const heading of Array.from(headings)) {
      if (heading.textContent?.toLowerCase().includes(concept.title.toLowerCase())) {
        heading.scrollIntoView({ behavior: 'smooth', block: 'start' });
        break;
      }
    }
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
          >
            ✨ Highlights {highlights.length > 0 && `(${highlights.length})`}
          </button>
          <button
            className={`${styles.tabButton} ${state.activeTab === 'quiz' ? styles.active : ''}`}
            onClick={() => handleTabChange('quiz')}
            role="tab"
            aria-selected={state.activeTab === 'quiz'}
            aria-controls="quiz-panel"
          >
            📝 Quiz
          </button>
          <button
            className={`${styles.tabButton} ${state.activeTab === 'concepts' ? styles.active : ''}`}
            onClick={() => handleTabChange('concepts')}
            role="tab"
            aria-selected={state.activeTab === 'concepts'}
            aria-controls="concepts-panel"
          >
            💡 Concepts
          </button>
          <button
            className={`${styles.tabButton} ${state.activeTab === 'progress' ? styles.active : ''}`}
            onClick={() => handleTabChange('progress')}
            role="tab"
            aria-selected={state.activeTab === 'progress'}
            aria-controls="progress-panel"
          >
            📊 Progress
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
              <HighlightsList
                highlights={highlights}
                onDelete={deleteHighlight}
                onHighlightClick={handleHighlightClick}
              />
            )}
            {state.activeTab === 'quiz' && (
              <QuizInterface />
            )}
            {state.activeTab === 'concepts' && (
              <ConceptsList
                concepts={concepts}
                isLoading={conceptsLoading}
                error={conceptsError}
                onConceptClick={handleConceptClick}
                onRetry={() => extractConcepts({
                  url: pageContent.url,
                  title: pageContent.title,
                  content: pageContent.content,
                })}
              />
            )}
            {state.activeTab === 'progress' && (
              <ProgressDashboard
                progress={progress}
                elapsedTime={elapsedTime}
                isCurrentPageVisited={isCurrentPageVisited}
                onClearProgress={clearProgress}
                recentRecords={getRecentRecords(10)}
              />
            )}
          </Suspense>
        </div>
      </aside>

      {/* Highlight Management Components (invisible) */}
      <HighlightManager pageUrl={pageContent.url} pageTitle={pageContent.title} />
      {/* HighlightRenderer disabled - highlights only shown in sidebar, not on page */}
      {/* <HighlightRenderer pageUrl={pageContent.url} /> */}
    </>
  );
}
