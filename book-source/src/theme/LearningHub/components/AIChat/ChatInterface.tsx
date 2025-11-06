/**
 * Chat Interface Component
 * Message history, input field, loading state
 */

import React, { useEffect, useRef } from 'react';
import type { ChatMessage as ChatMessageType } from '../../types';
import { ChatMessage } from './ChatMessage';
import { ChatInput } from './ChatInput';
import styles from './ChatInterface.module.css';

interface ChatInterfaceProps {
  messages: ChatMessageType[];
  onSendMessage: (message: string) => void;
  isLoading: boolean;
  error?: string | null;
  onRetry?: () => void;
}

export function ChatInterface({ 
  messages, 
  onSendMessage, 
  isLoading, 
  error,
  onRetry 
}: ChatInterfaceProps) {
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const messagesContainerRef = useRef<HTMLDivElement>(null);
  const previousMessageCountRef = useRef(messages.length);
  const userHasScrolledRef = useRef(false);

  // Track if user has manually scrolled up
  useEffect(() => {
    const container = messagesContainerRef.current;
    if (!container) return;

    const handleScroll = () => {
      const { scrollTop, scrollHeight, clientHeight } = container;
      const isAtBottom = scrollHeight - scrollTop - clientHeight < 50; // 50px threshold
      
      // If user scrolls to bottom, enable auto-scroll again
      if (isAtBottom) {
        userHasScrolledRef.current = false;
      } else {
        // User scrolled up
        userHasScrolledRef.current = true;
      }
    };

    container.addEventListener('scroll', handleScroll);
    return () => container.removeEventListener('scroll', handleScroll);
  }, []);

  // Auto-scroll to bottom only when new messages arrive and user hasn't scrolled up
  useEffect(() => {
    const hasNewMessages = messages.length > previousMessageCountRef.current;
    previousMessageCountRef.current = messages.length;

    // Only auto-scroll if:
    // 1. There are new messages
    // 2. User hasn't manually scrolled up
    if (hasNewMessages && !userHasScrolledRef.current) {
      messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages]);

  return (
    <div className={styles.chatInterface}>
      <div className={styles.messagesContainer} ref={messagesContainerRef}>
        {messages.length === 0 && !isLoading && !error && (
          <div className={styles.emptyState}>
            <div className={styles.emptyIcon}>🤖</div>
            <h3>AI Learning Assistant</h3>
            <p>Ask questions, get explanations, or discuss concepts from this chapter. I'm here to help you learn!</p>
          </div>
        )}

        {messages.map(message => (
          <ChatMessage key={message.id} message={message} />
        ))}

        {isLoading && (
          <div className={styles.loadingIndicator}>
            <div className={styles.typingDots}>
              <span></span>
              <span></span>
              <span></span>
            </div>
            <span className={styles.loadingText}>AI is thinking...</span>
          </div>
        )}

        {error && (
          <div className={styles.errorMessage}>
            <div className={styles.errorIcon}>⚠️</div>
            <div className={styles.errorContent}>
              <p className={styles.errorTitle}>AI assistant temporarily unavailable</p>
              <p className={styles.errorDescription}>{error}</p>
              {onRetry && (
                <button className={styles.retryButton} onClick={onRetry}>
                  Try Again
                </button>
              )}
            </div>
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      <ChatInput 
        onSend={onSendMessage} 
        disabled={isLoading}
        placeholder="Ask a question about this chapter..."
      />
    </div>
  );
}
