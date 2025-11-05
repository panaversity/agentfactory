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

  // Update refs whenever props change
  useEffect(() => {
    pageUrlRef.current = pageUrl;
    pageTitleRef.current = pageTitle;
    pageContentRef.current = pageContent;
    
    console.log('[useGeminiChat] Refs updated:', {
      pageUrl,
      pageTitle,
      contentLength: pageContent?.length || 0,
    });
  }, [pageUrl, pageTitle, pageContent]);

  // Filter messages for current page only
  const messages = state.chatHistory.filter(msg => msg.pageUrl === pageUrl);

  // Clear chat when page changes
  useEffect(() => {
    if (currentPageUrl !== pageUrl) {
      console.log('[useGeminiChat] ⚠️ PAGE CHANGED - CLEARING ALL CHAT', {
        from: currentPageUrl,
        to: pageUrl,
        newContentLength: pageContent?.length || 0,
      });
      // Clear ALL messages to start completely fresh
      dispatch({ type: 'CLEAR_CHAT_HISTORY' });
      setCurrentPageUrl(pageUrl);
      setError(null);
      setLastUserMessage(null);
    }
  }, [pageUrl, currentPageUrl, pageContent, dispatch]);

  const sendMessage = useCallback(async (content: string) => {
    setError(null);
    setIsLoading(true);
    setLastUserMessage(content);

    // ALWAYS use refs to get the latest values (no stale closures!)
    const currentUrl = pageUrlRef.current;
    const currentTitle = pageTitleRef.current;
    const currentContent = pageContentRef.current;

    console.log('[useGeminiChat] 📤 SENDING MESSAGE', {
      pageUrl: currentUrl,
      pageTitle: currentTitle,
      contentLength: currentContent?.length || 0,
      contentPreview: currentContent?.substring(0, 100) || '',
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

      // Send to API with streaming
      let accumulatedResponse = '';
      const assistantMessageId = crypto.randomUUID();
      let isFirstChunk = true;
      
      console.log('[useGeminiChat] 🚀 CALLING API with:', {
        url: currentUrl,
        title: currentTitle,
        contentLength: currentContent?.length || 0,
        historyCount: currentPageMessages.length,
      });

      for await (const chunk of sendChatMessage({
        userMessage: content,
        pageUrl: currentUrl,
        pageTitle: currentTitle,
        pageContent: currentContent || '',
        conversationHistory: currentPageMessages.slice(-10), // Last 10 messages from THIS page
      })) {
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
    dispatch({ type: 'CLEAR_CHAT_HISTORY' });
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
