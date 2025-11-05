/**
 * useGeminiChat Hook
 * Message state, streaming handling, error recovery
 */

import { useState, useCallback } from 'react';
import type { ChatMessage } from '../types';
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

  const messages = state.chatHistory;

  const sendMessage = useCallback(async (content: string) => {
    setError(null);
    setIsLoading(true);
    setLastUserMessage(content);

    try {
      // Add user message immediately
      const userMessage: ChatMessage = {
        id: crypto.randomUUID(),
        role: 'user',
        content,
        timestamp: Date.now(),
        pageUrl,
      };

      dispatch({ type: 'ADD_CHAT_MESSAGE', payload: userMessage });

      // Send to API with streaming
      let accumulatedResponse = '';
      const assistantMessageId = crypto.randomUUID();

      for await (const chunk of sendChatMessage({
        userMessage: content,
        pageUrl,
        pageTitle,
        pageContent,
        conversationHistory: messages.slice(-10), // Last 10 messages
      })) {
        accumulatedResponse += chunk;

        // Update assistant message in real-time
        const assistantMessage: ChatMessage = {
          id: assistantMessageId,
          role: 'assistant',
          content: accumulatedResponse,
          timestamp: Date.now(),
          pageUrl,
        };

        // Replace or add assistant message
        dispatch({ type: 'ADD_CHAT_MESSAGE', payload: assistantMessage });
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
        pageUrl,
      };

      dispatch({ type: 'ADD_CHAT_MESSAGE', payload: errorChatMessage });
    }
  }, [pageUrl, pageTitle, pageContent, messages, dispatch]);

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
