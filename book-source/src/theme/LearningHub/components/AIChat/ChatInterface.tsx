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

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  return (
    <div className={styles.chatInterface}>
      <div className={styles.messagesContainer}>
        {messages.length === 0 && !isLoading && (
          <div className={styles.emptyState}>
            <div className={styles.emptyIcon}>💬</div>
            <h3>Start a conversation</h3>
            <p>Ask me anything about the content you're reading!</p>
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
