/**
 * Chat Message Component
 * Role-based styling for user vs assistant messages
 */

import React from 'react';
import type { ChatMessage as ChatMessageType } from '../../types';
import styles from './ChatMessage.module.css';

interface ChatMessageProps {
  message: ChatMessageType;
}

export function ChatMessage({ message }: ChatMessageProps) {
  const { role, content, timestamp } = message;
  const isUser = role === 'user';

  const formattedTime = new Date(timestamp).toLocaleTimeString('en-US', {
    hour: 'numeric',
    minute: '2-digit',
  });

  return (
    <div className={`${styles.message} ${isUser ? styles.userMessage : styles.assistantMessage}`}>
      <div className={styles.messageHeader}>
        <span className={styles.role}>{isUser ? 'You' : 'AI Assistant'}</span>
        <span className={styles.timestamp}>{formattedTime}</span>
      </div>
      <div className={styles.content}>
        {content}
      </div>
    </div>
  );
}
