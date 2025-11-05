/**
 * Chat Input Component
 * Enter/Shift+Enter handling, character limit (10,000), disabled state
 */

import React, { useState, useRef, type KeyboardEvent } from 'react';
import { LIMITS } from '../../types';
import styles from './ChatInput.module.css';

interface ChatInputProps {
  onSend: (message: string) => void;
  disabled?: boolean;
  placeholder?: string;
}

export function ChatInput({ onSend, disabled = false, placeholder = 'Ask a question...' }: ChatInputProps) {
  const [message, setMessage] = useState('');
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  const handleKeyDown = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    // Enter sends, Shift+Enter adds new line
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const handleSend = () => {
    const trimmed = message.trim();
    if (trimmed && !disabled) {
      onSend(trimmed);
      setMessage('');
      
      // Reset textarea height
      if (textareaRef.current) {
        textareaRef.current.style.height = 'auto';
      }
    }
  };

  const handleChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    const value = e.target.value;
    
    // Enforce character limit
    if (value.length <= LIMITS.MAX_CHAT_MESSAGE_LENGTH) {
      setMessage(value);
      
      // Auto-resize textarea
      if (textareaRef.current) {
        textareaRef.current.style.height = 'auto';
        textareaRef.current.style.height = `${textareaRef.current.scrollHeight}px`;
      }
    }
  };

  const remainingChars = LIMITS.MAX_CHAT_MESSAGE_LENGTH - message.length;
  const isNearLimit = remainingChars < 100;

  return (
    <div className={styles.inputContainer}>
      <textarea
        ref={textareaRef}
        className={styles.textarea}
        value={message}
        onChange={handleChange}
        onKeyDown={handleKeyDown}
        placeholder={placeholder}
        disabled={disabled}
        rows={1}
        aria-label="Chat message input"
      />
      
      <div className={styles.inputFooter}>
        <div className={styles.charCounter}>
          {isNearLimit && (
            <span className={styles.charCountWarning}>
              {remainingChars} characters remaining
            </span>
          )}
        </div>
        
        <button
          className={styles.sendButton}
          onClick={handleSend}
          disabled={disabled || !message.trim()}
          aria-label="Send message"
        >
          Send
        </button>
      </div>
      
      <div className={styles.hint}>
        Press <kbd>Enter</kbd> to send, <kbd>Shift+Enter</kbd> for new line
      </div>
    </div>
  );
}
