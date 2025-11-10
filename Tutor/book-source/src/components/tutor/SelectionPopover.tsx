/**
 * SelectionPopover Component
 *
 * Appears when user highlights text in the book content
 * Shows action buttons: Summary, Explain, Main Points, Example, Ask Tutor
 */

import React, { useState, useEffect, useRef, useCallback } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import { agentApi, useMockResponses, mockAgentResponse, type AgentAction, type ChatMessage } from '@/utils/agentApi';
import InlinePreview from './InlinePreview';

interface SelectionPopoverProps {
  onOpenChat?: (prefillText: string, autoSend?: boolean) => void;
  onNewMessage?: (message: ChatMessage) => void;
}

interface PopoverPosition {
  top: number;
  left: number;
}

interface PopoverState {
  visible: boolean;
  position: PopoverPosition;
  selectedText: string;
  cursorContext: string;
  range: Range | null;
}

const SelectionPopover: React.FC<SelectionPopoverProps> = ({ onOpenChat, onNewMessage }) => {
  const [popoverState, setPopoverState] = useState<PopoverState>({
    visible: false,
    position: { top: 0, left: 0 },
    selectedText: '',
    cursorContext: '',
    range: null,
  });

  const [inlinePreview, setInlinePreview] = useState<{
    visible: boolean;
    content: string;
    position: PopoverPosition;
  } | null>(null);

  const [isLoading, setIsLoading] = useState(false);
  const popoverRef = useRef<HTMLDivElement>(null);
  const selectionTimeoutRef = useRef<NodeJS.Timeout | null>(null);

  const actions: Array<{
    id: AgentAction['action'];
    label: string;
    icon: string;
    color: string;
  }> = [
    { id: 'summary', label: 'Summary', icon: '📝', color: '#3b82f6' },
    { id: 'explain', label: 'Explain', icon: '💡', color: '#8b5cf6' },
    { id: 'main_points', label: 'Main Points', icon: '🎯', color: '#10b981' },
    { id: 'example', label: 'Example', icon: '🔍', color: '#f59e0b' },
    { id: 'ask', label: 'Ask Tutor', icon: '💬', color: '#ec4899' },
  ];

  // Get surrounding context (paragraph before and after)
  const getCursorContext = (range: Range): string => {
    try {
      const container = range.commonAncestorContainer;
      const element =
        container.nodeType === Node.TEXT_NODE
          ? container.parentElement
          : (container as Element);

      if (!element) return '';

      // Find the parent paragraph or article
      const paragraph = element.closest('p, article, div[class*="markdown"]');
      if (paragraph) {
        return paragraph.textContent?.substring(0, 500) || '';
      }

      return '';
    } catch (error) {
      console.error('Error getting cursor context:', error);
      return '';
    }
  };

  // Calculate popover position based on selection
  const calculatePosition = (range: Range): PopoverPosition => {
    const rect = range.getBoundingClientRect();
    const scrollTop = window.pageYOffset || document.documentElement.scrollTop;
    const scrollLeft = window.pageXOffset || document.documentElement.scrollLeft;

    return {
      top: rect.top + scrollTop - 60, // Position above selection
      left: rect.left + scrollLeft + rect.width / 2, // Center on selection
    };
  };

  // Handle text selection
  const handleSelection = useCallback(() => {
    if (selectionTimeoutRef.current) {
      clearTimeout(selectionTimeoutRef.current);
    }

    selectionTimeoutRef.current = setTimeout(() => {
      const selection = window.getSelection();
      const selectedText = selection?.toString().trim() || '';

      if (selectedText.length > 0 && selectedText.length < 5000) {
        const range = selection?.getRangeAt(0);
        if (range) {
          const position = calculatePosition(range);
          const context = getCursorContext(range);

          setPopoverState({
            visible: true,
            position,
            selectedText,
            cursorContext: context,
            range,
          });
        }
      } else {
        setPopoverState((prev) => ({ ...prev, visible: false }));
      }
    }, 200); // Small delay to prevent flickering during selection
  }, []);

  // Handle clicking outside popover
  const handleClickOutside = useCallback((event: MouseEvent) => {
    if (popoverRef.current && !popoverRef.current.contains(event.target as Node)) {
      // Check if clicking on inline preview
      const inlinePreviewElement = document.querySelector('.inline-preview');
      if (inlinePreviewElement && inlinePreviewElement.contains(event.target as Node)) {
        return;
      }

      setPopoverState((prev) => ({ ...prev, visible: false }));
      setInlinePreview(null);
    }
  }, []);

  // Set up event listeners
  useEffect(() => {
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);
    document.addEventListener('mousedown', handleClickOutside);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
      document.removeEventListener('mousedown', handleClickOutside);
      if (selectionTimeoutRef.current) {
        clearTimeout(selectionTimeoutRef.current);
      }
    };
  }, [handleSelection, handleClickOutside]);

  // Handle action click
  const handleActionClick = async (action: AgentAction['action']) => {
    if (isLoading) return;

    setIsLoading(true);

    try {
      let promptText = '';

      // Format the prompt based on action
      switch (action) {
        case 'summary':
          promptText = `Please provide a summary of the following text:\n\n"${popoverState.selectedText}"`;
          break;
        case 'explain':
          promptText = `Please explain the following text in simple terms:\n\n"${popoverState.selectedText}"`;
          break;
        case 'main_points':
          promptText = `Please list the main points from the following text:\n\n"${popoverState.selectedText}"`;
          break;
        case 'example':
          promptText = `Please provide a practical example for the following concept:\n\n"${popoverState.selectedText}"`;
          break;
        case 'ask':
          // For "Ask Tutor", just prefill the text without auto-sending
          promptText = `Student highlighted: "${popoverState.selectedText}"\n\nQuestion: `;
          // Open chat without auto-send for Ask Tutor
          onOpenChat?.(promptText, false);
          setPopoverState((prev) => ({ ...prev, visible: false }));
          setIsLoading(false);
          return;
      }

      // For all other actions, open TutorChat with the prompt and auto-send
      // Dispatch event directly to TutorChat
      const event = new CustomEvent('openTutorChat', {
        detail: {
          text: promptText,
          autoSend: true  // Auto-send for Summary, Explain, Main Points, Example
        }
      });
      window.dispatchEvent(event);

      // Hide popover
      setPopoverState((prev) => ({ ...prev, visible: false }));
    } catch (error) {
      console.error('Error handling action:', error);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <>
      {/* Selection Popover */}
      <AnimatePresence>
        {popoverState.visible && (
          <motion.div
            ref={popoverRef}
            initial={{ opacity: 0, scale: 0.9, y: 10 }}
            animate={{ opacity: 1, scale: 1, y: 0 }}
            exit={{ opacity: 0, scale: 0.9, y: 10 }}
            transition={{ duration: 0.15 }}
            className="selection-popover"
            style={{
              position: 'absolute',
              top: `${popoverState.position.top}px`,
              left: `${popoverState.position.left}px`,
              transform: 'translateX(-50%)',
              zIndex: 9999,
            }}
            role="menu"
            aria-label="Text selection actions"
          >
            <div className="popover-arrow"></div>
            <div className="popover-actions">
              {actions.map((action) => (
                <motion.button
                  key={action.id}
                  whileHover={{ scale: 1.05, y: -2 }}
                  whileTap={{ scale: 0.95 }}
                  onClick={() => handleActionClick(action.id)}
                  disabled={isLoading}
                  className="popover-action-button"
                  style={{ borderColor: action.color }}
                  title={action.label}
                  aria-label={`${action.label} selected text`}
                >
                  <span className="action-icon">{action.icon}</span>
                  <span className="action-label">{action.label}</span>
                </motion.button>
              ))}
            </div>
            {isLoading && (
              <div className="popover-loading">
                <div className="spinner"></div>
                <span>Processing...</span>
              </div>
            )}
          </motion.div>
        )}
      </AnimatePresence>

      {/* Inline Preview */}
      {inlinePreview && (
        <InlinePreview
          content={inlinePreview.content}
          position={inlinePreview.position}
          onClose={() => setInlinePreview(null)}
        />
      )}
    </>
  );
};

export default SelectionPopover;
