/**
 * Learning Hub Main Component
 * Lazy loading, toggle state, tab navigation
 */

import React, { Suspense, lazy, useRef, useCallback } from 'react';
import { useLearningHub } from './context/LearningHubContext';
import { LearningHubToggle } from './LearningHubToggle';
import { usePageContent } from './hooks/usePageContent';
import { useGeminiChat } from './hooks/useGeminiChat';
import { useHighlights } from './hooks/useHighlights';
import { useKeyConcepts } from './hooks/useKeyConcepts';
import { useProgress } from './hooks/useProgress';
import { HighlightManager } from './components/SmartHighlights/HighlightManager';
import { ProgressDashboard } from './components/ProgressTracker/ProgressDashboard';
import { ErrorBoundary } from './components/ErrorBoundary';
import { LoadingSkeleton } from './components/LoadingSkeleton/LoadingSkeleton';
import { ConnectionStatus } from './components/ConnectionStatus/ConnectionStatus';
import { APIKeyStatus } from './components/APIKeyStatus/APIKeyStatus';
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

  // Tab switching debounce (300ms) to cancel in-flight AI requests
  const tabSwitchTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const pendingTabRef = useRef<typeof state.activeTab | null>(null);

  // Sidebar resize state
  const [sidebarWidth, setSidebarWidth] = React.useState(() => {
    if (typeof localStorage !== 'undefined') {
      const stored = localStorage.getItem('learningHub_sidebarWidth');
      return stored ? parseInt(stored, 10) : 400;
    }
    return 400;
  });
  const [isResizing, setIsResizing] = React.useState(false);
  const sidebarRef = useRef<HTMLElement>(null);

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

  // Keyboard shortcuts: Ctrl+Shift+L to toggle, Escape to close
  React.useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      // Ctrl+Shift+L (or Cmd+Shift+L on Mac) to toggle
      if ((event.ctrlKey || event.metaKey) && event.shiftKey && event.key === 'L') {
        event.preventDefault();
        dispatch({ type: 'TOGGLE_SIDEBAR' });
      }
      // Escape to close
      else if (event.key === 'Escape' && state.isOpen) {
        event.preventDefault();
        dispatch({ type: 'TOGGLE_SIDEBAR' });
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [state.isOpen, dispatch]);

  // Cleanup timeout on unmount
  React.useEffect(() => {
    return () => {
      if (tabSwitchTimeoutRef.current) {
        clearTimeout(tabSwitchTimeoutRef.current);
      }
    };
  }, []);

  // Handle sidebar resize
  const handleMouseDown = useCallback((e: React.MouseEvent) => {
    e.preventDefault();
    setIsResizing(true);
  }, []);

  React.useEffect(() => {
    if (!isResizing) return;

    const handleMouseMove = (e: MouseEvent) => {
      if (!sidebarRef.current) return;
      
      // Calculate new width based on distance from right edge
      const newWidth = window.innerWidth - e.clientX;
      
      // Clamp between min and max
      const clampedWidth = Math.max(300, Math.min(800, newWidth));
      
      setSidebarWidth(clampedWidth);
      
      // Update CSS variable
      if (sidebarRef.current) {
        sidebarRef.current.style.setProperty('--sidebar-width', `${clampedWidth}px`);
      }
    };

    const handleMouseUp = () => {
      setIsResizing(false);
      
      // Save to localStorage
      if (typeof localStorage !== 'undefined') {
        localStorage.setItem('learningHub_sidebarWidth', sidebarWidth.toString());
      }
    };

    document.addEventListener('mousemove', handleMouseMove);
    document.addEventListener('mouseup', handleMouseUp);

    // Prevent text selection while resizing
    document.body.style.userSelect = 'none';
    document.body.style.cursor = 'ew-resize';

    return () => {
      document.removeEventListener('mousemove', handleMouseMove);
      document.removeEventListener('mouseup', handleMouseUp);
      document.body.style.userSelect = '';
      document.body.style.cursor = '';
    };
  }, [isResizing, sidebarWidth]);

  // Log page content whenever it changes
  console.log('[LearningHub] 📖 Page content state:', {
    url: pageContent.url,
    title: pageContent.title,
    isValidPage: pageContent.isValidPage,
    contentLength: pageContent.content?.length || 0,
    highlightsCount: highlights.length,
    currentBrowserUrl: typeof window !== 'undefined' ? window.location.pathname : '',
  });

  // Define handlers (must be after all hooks)
  const handleToggle = useCallback(() => {
    dispatch({ type: 'TOGGLE_SIDEBAR' });
  }, [dispatch]);

  const handleTabChange = useCallback((tab: typeof state.activeTab) => {
    // Clear any pending tab switches
    if (tabSwitchTimeoutRef.current) {
      clearTimeout(tabSwitchTimeoutRef.current);
    }

    // Store pending tab
    pendingTabRef.current = tab;

    // Debounce tab switch by 300ms to allow cancelling in-flight AI requests
    tabSwitchTimeoutRef.current = setTimeout(() => {
      console.log('[LearningHub] Tab switch debounce complete, switching to:', tab);
      dispatch({ type: 'SET_ACTIVE_TAB', payload: tab });
      pendingTabRef.current = null;
    }, 300);
  }, [dispatch]);

  const handleHighlightClick = useCallback((highlight: any) => {
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
  }, []);

  const handleConceptClick = useCallback((concept: any) => {
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
  }, []);

  // Don't render on non-documentation pages (MUST be after ALL hooks)
  if (!pageContent.isValidPage) {
    return null;
  }

  return (
    <>
      <LearningHubToggle isOpen={state.isOpen} onClick={handleToggle} />

      <aside
        ref={sidebarRef}
        className={`${styles.learningHubSidebar} ${state.isOpen ? styles.isOpen : ''} ${isResizing ? styles.resizing : ''}`}
        role="complementary"
        aria-label="Learning Hub - AI-powered learning assistant"
        aria-hidden={!state.isOpen}
        style={{ '--sidebar-width': `${sidebarWidth}px` } as React.CSSProperties}
      >
        {/* Resize Handle */}
        <div 
          className={styles.resizeHandle}
          onMouseDown={handleMouseDown}
          aria-label="Resize sidebar"
        />
        {/* Header */}
        <header className={styles.sidebarHeader}>
          <h2 className={styles.sidebarTitle} id="learning-hub-title">Learning Hub</h2>
          <button
            className={styles.closeButton}
            onClick={handleToggle}
            aria-label="Close Learning Hub sidebar (or press Escape)"
            title="Close sidebar (Esc)"
          >
            ×
          </button>
        </header>

        {/* Connection & API Status */}
        <div className={styles.statusBanners}>
          <ConnectionStatus />
          <APIKeyStatus />
        </div>

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
          <ErrorBoundary>
            <Suspense fallback={
              <LoadingSkeleton 
                type={state.activeTab === 'chat' ? 'chat' : 
                      state.activeTab === 'highlights' ? 'highlights' : 
                      state.activeTab === 'quiz' ? 'quiz' : 
                      state.activeTab === 'concepts' ? 'concepts' : 'progress'} 
                count={state.activeTab === 'quiz' ? 1 : 3} 
              />
            }>
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
          </ErrorBoundary>
        </div>
      </aside>

      {/* Highlight Management Components (invisible) */}
      <HighlightManager pageUrl={pageContent.url} pageTitle={pageContent.title} />
      {/* HighlightRenderer disabled - highlights only shown in sidebar, not on page */}
      {/* <HighlightRenderer pageUrl={pageContent.url} /> */}
    </>
  );
}
