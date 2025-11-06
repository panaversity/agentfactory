/**
 * HighlightPopup Component
 * Small popup with "Explain", "Save Highlight", and custom question input
 */

import React, { useState } from 'react';
import styles from './HighlightPopup.module.css';

interface HighlightPopupProps {
  position: { top: number; left: number };
  onExplain: () => void;
  onSave: () => void;
  onCustomQuestion: (question: string) => void;
  onClose: () => void;
  isLoading?: boolean;
}

export function HighlightPopup({
  position,
  onExplain,
  onSave,
  onCustomQuestion,
  onClose,
  isLoading = false,
}: HighlightPopupProps) {
  const [customQuestion, setCustomQuestion] = useState('');
  const [showInput, setShowInput] = useState(false);

  console.log('[HighlightPopup] Rendering at position:', position);

  const handleButtonClick = (callback: () => void) => (e: React.MouseEvent) => {
    e.preventDefault();
    console.log('[HighlightPopup] Button clicked');
    callback();
  };

  const handleCustomSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    e.stopPropagation();
    console.log('[HighlightPopup] Form submitted', customQuestion);
    if (customQuestion.trim()) {
      onCustomQuestion(customQuestion.trim());
      setCustomQuestion('');
      setShowInput(false);
    }
  };

  const handleSubmitClick = (e: React.MouseEvent) => {
    e.preventDefault();
    e.stopPropagation();
    console.log('[HighlightPopup] Submit button clicked', customQuestion);
    if (customQuestion.trim()) {
      onCustomQuestion(customQuestion.trim());
      setCustomQuestion('');
      setShowInput(false);
    }
  };

  const handleAskClick = (e: React.MouseEvent) => {
    e.preventDefault();
    console.log('[HighlightPopup] Ask button clicked, toggling input');
    setShowInput(!showInput);
  };

  return (
    <div
      className={`${styles.highlightPopup} highlight-popup-container`}
      style={{
        top: `${position.top}px`,
        left: `${position.left}px`,
      }}
    >
      <div className={styles.buttonRow}>
        <button
          className={`${styles.popupButton} ${styles.explainButton}`}
          onClick={handleButtonClick(onExplain)}
          disabled={isLoading}
          title="Ask AI to explain this text"
        >
          {isLoading ? '⏳' : '💡'} Explain
        </button>
        
        <button
          className={`${styles.popupButton} ${styles.askButton}`}
          onClick={handleAskClick}
          disabled={isLoading}
          title="Ask a custom question about this text"
        >
          💬 Ask
        </button>

        <button
          className={`${styles.popupButton} ${styles.saveButton}`}
          onClick={handleButtonClick(onSave)}
          disabled={isLoading}
          title="Save this highlight"
        >
          ⭐ Save
        </button>

        <button
          className={`${styles.popupButton} ${styles.closeButton}`}
          onClick={handleButtonClick(onClose)}
          title="Close"
        >
          ✕
        </button>
      </div>

      {showInput && (
        <form 
          className={styles.customQuestionForm} 
          onSubmit={handleCustomSubmit}
        >
          <input
            type="text"
            className={styles.customQuestionInput}
            placeholder="Ask anything about this text..."
            value={customQuestion}
            onChange={(e) => setCustomQuestion(e.target.value)}
            autoFocus
            disabled={isLoading}
          />
          <button
            type="button"
            className={styles.submitButton}
            onClick={handleSubmitClick}
            disabled={isLoading || !customQuestion.trim()}
          >
            Send
          </button>
        </form>
      )}
    </div>
  );
}
