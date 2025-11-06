/**
 * useGeminiChat Hook
 * Message state, streaming handling, error recovery
 */

import { useState, useCallback, useEffect, useRef } from 'react';
import type { ChatMessage } from '../types/entities';
import { useLearningHub } from '../context/LearningHubContext';
import { sendChatMessage } from '../services/geminiChatService';
import { errorLogger } from '../services/errorLogger';

interface UseGeminiChatReturn {
  messages: ChatMessage[];
  isLoading: boolean;
  error: string | null;
  sendMessage: (content: string) => Promise<void>;
  clearMessages: () => void;
  retryLastMessage: () => Promise<void>;
}

export function useGeminiChat(pageUrl: string, pageTitle: string, pageContent: string): UseGeminiChatReturn {
  const { state, dispatch } = useLearningHub();
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [lastUserMessage, setLastUserMessage] = useState<string | null>(null);
  const [currentPageUrl, setCurrentPageUrl] = useState(pageUrl);
  
  // Use refs to always get the latest values (avoid stale closures)
  const pageUrlRef = useRef(pageUrl);
  const pageTitleRef = useRef(pageTitle);
  const pageContentRef = useRef(pageContent);

  // CRITICAL: Update refs IMMEDIATELY when props change (before any effects run)
  // This ensures refs always have the latest values, even during remount
  pageUrlRef.current = pageUrl;
  pageTitleRef.current = pageTitle;
  pageContentRef.current = pageContent;

  // Log whenever props change
  useEffect(() => {
    console.log('[useGeminiChat] 🔄 Props changed:', {
      pageUrl,
      pageTitle,
      contentLength: pageContent?.length || 0,
      contentPreview: pageContent?.substring(0, 150) || 'EMPTY',
      timestamp: new Date().toISOString(),
    });
  }, [pageUrl, pageTitle, pageContent]);

  // Filter messages for current page only
  const messages = state.chatHistory.filter(msg => msg.pageUrl === pageUrl);

  // Clear chat when page changes - use CLEAR_PAGE_CHAT for proper per-page clearing
  useEffect(() => {
    if (currentPageUrl && currentPageUrl !== pageUrl) {
      console.log('[useGeminiChat] ⚠️ PAGE CHANGED - CLEARING PAGE CHAT', {
        from: currentPageUrl,
        to: pageUrl,
        newContentLength: pageContent?.length || 0,
      });
      // Clear messages for the OLD page (keep messages from other pages)
      dispatch({ type: 'CLEAR_PAGE_CHAT', payload: currentPageUrl });
      // Also clear messages for the NEW page to start fresh
      dispatch({ type: 'CLEAR_PAGE_CHAT', payload: pageUrl });
      setCurrentPageUrl(pageUrl);
      setError(null);
      setLastUserMessage(null);
    } else if (!currentPageUrl) {
      // Initialize on first render
      setCurrentPageUrl(pageUrl);
    }
  }, [pageUrl, currentPageUrl, pageContent, dispatch]);

  const sendMessage = useCallback(async (content: string) => {
    setError(null);
    setIsLoading(true);
    setLastUserMessage(content);

    // CRITICAL: ALWAYS use refs to get the CURRENT values at send time
    const currentUrl = pageUrlRef.current;
    const currentTitle = pageTitleRef.current;
    const currentContent = pageContentRef.current;

    // CRITICAL VALIDATION: Ensure we're using current page data
    console.log('[useGeminiChat] 📤 SENDING MESSAGE - VALIDATION', {
      pageUrl: currentUrl,
      pageTitle: currentTitle,
      contentLength: currentContent?.length || 0,
      contentPreview: currentContent?.substring(0, 150) || 'EMPTY',
      propsPageUrl: pageUrl, // Compare with props
      propsPageTitle: pageTitle,
      propsContentLength: pageContent?.length || 0,
      MATCH: currentUrl === pageUrl && currentContent === pageContent,
    });

    try {
      // Add user message immediately
      const userMessage: ChatMessage = {
        id: crypto.randomUUID(),
        role: 'user',
        content,
        timestamp: Date.now(),
        pageUrl: currentUrl,
      };

      dispatch({ type: 'ADD_CHAT_MESSAGE', payload: userMessage });

      // Get only messages from current page for conversation history
      const currentPageMessages = state.chatHistory.filter(msg => msg.pageUrl === currentUrl);

      // CRITICAL: Validate we're sending current page content
      if (!currentContent || currentContent.length < 100) {
        console.warn('[useGeminiChat] ⚠️ WARNING: Page content seems empty or too short!', {
          url: currentUrl,
          contentLength: currentContent?.length || 0,
        });
      }

      // Send to API with streaming
      let accumulatedResponse = '';
      const assistantMessageId = crypto.randomUUID();
      let isFirstChunk = true;
      
      console.log('[useGeminiChat] 🚀 CALLING API with:', {
        url: currentUrl,
        title: currentTitle,
        contentLength: currentContent?.length || 0,
        contentPreviewFirst100: currentContent?.substring(0, 100) || 'EMPTY',
        historyCount: currentPageMessages.length,
      });

      // CRITICAL: Final validation before API call
      const apiRequest = {
        userMessage: content,
        pageUrl: currentUrl,
        pageTitle: currentTitle,
        pageContent: currentContent || '',
        conversationHistory: currentPageMessages.slice(-10), // Last 10 messages from THIS page
      };

      console.log('[useGeminiChat] 🎯 FINAL API REQUEST:', {
        pageUrl: apiRequest.pageUrl,
        pageTitle: apiRequest.pageTitle,
        contentLength: apiRequest.pageContent.length,
        contentHash: apiRequest.pageContent.substring(0, 50) + '...' + apiRequest.pageContent.substring(apiRequest.pageContent.length - 50),
        userMessage: apiRequest.userMessage,
      });

      for await (const chunk of sendChatMessage(apiRequest)) {
        accumulatedResponse += chunk;

        // Create/update assistant message in real-time
        const assistantMessage: ChatMessage = {
          id: assistantMessageId,
          role: 'assistant',
          content: accumulatedResponse,
          timestamp: Date.now(),
          pageUrl: currentUrl,
        };

        // Add on first chunk, update on subsequent chunks
        if (isFirstChunk) {
          dispatch({ type: 'ADD_CHAT_MESSAGE', payload: assistantMessage });
          isFirstChunk = false;
        } else {
          dispatch({ type: 'UPDATE_CHAT_MESSAGE', payload: assistantMessage });
        }
      }

      setIsLoading(false);
    } catch (err: any) {
      setIsLoading(false);
      const errorMessage = err.message || 'Failed to get response from AI';
      setError(errorMessage);
      
      errorLogger.logError(err, {
        context: 'useGeminiChat.sendMessage',
        userMessage: content,
        pageUrl,
      });

      // Add error message to chat
      const errorChatMessage: ChatMessage = {
        id: crypto.randomUUID(),
        role: 'assistant',
        content: `❌ ${errorMessage}`,
        timestamp: Date.now(),
        pageUrl: pageUrlRef.current,
      };

      dispatch({ type: 'ADD_CHAT_MESSAGE', payload: errorChatMessage });
    }
  }, [state.chatHistory, dispatch]); // Only depend on state and dispatch, refs handle the rest

  const clearMessages = useCallback(() => {
    // Clear only the current page's chat
    dispatch({ type: 'CLEAR_PAGE_CHAT', payload: pageUrlRef.current });
    setError(null);
  }, [dispatch]);

  const retryLastMessage = useCallback(async () => {
    if (lastUserMessage) {
      await sendMessage(lastUserMessage);
    }
  }, [lastUserMessage, sendMessage]);

  return {
    messages,
    isLoading,
    error,
    sendMessage,
    clearMessages,
    retryLastMessage,
  };
}
