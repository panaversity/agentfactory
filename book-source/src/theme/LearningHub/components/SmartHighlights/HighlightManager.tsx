/**
 * HighlightManager Component
 * Manages text selection, popup display, and highlight operations
 */

import React, { useState, useEffect, useCallback } from 'react';
import { useTextSelection } from '../../hooks/useTextSelection';
import { useHighlights } from '../../hooks/useHighlights';
import { useLearningHub } from '../../context/LearningHubContext';
import { HighlightPopup } from './HighlightPopup';
import { geminiService } from '../../services/geminiService';
import { extractSurroundingContext } from '../../utils/contentExtractor';

interface HighlightManagerProps {
  pageUrl: string;
  pageTitle: string;
}

export function HighlightManager({ pageUrl, pageTitle }: HighlightManagerProps) {
  const { selection, clearSelection } = useTextSelection();
  const { saveHighlight } = useHighlights(pageUrl);
  const { dispatch } = useLearningHub();
  const [isExplaining, setIsExplaining] = useState(false);
  const [popupPosition, setPopupPosition] = useState<{ top: number; left: number } | null>(null);
  const [showPopup, setShowPopup] = useState(false);
  const isInteractingRef = React.useRef(false);
  const savedSelectionRef = React.useRef<typeof selection>(null);

  // Calculate popup position when selection changes and save selection to ref
  useEffect(() => {
    if (selection && selection.boundingRect) {
      const rect = selection.boundingRect;
      const scrollY = window.scrollY;
      const scrollX = window.scrollX;

      // Position popup above the selection
      const top = rect.top + scrollY - 50;
      const left = rect.left + scrollX + (rect.width / 2);

      setPopupPosition({ top, left });
      setShowPopup(true);
      // CRITICAL: Save selection to ref so it persists across re-renders
      savedSelectionRef.current = selection;
      isInteractingRef.current = false; // Reset interaction flag on new selection
      console.log('[HighlightManager] Selection saved to ref:', selection.text.substring(0, 30));
    } else if (!isInteractingRef.current && !showPopup) {
      // Only clear if we're not showing popup and not interacting
      setPopupPosition(null);
      savedSelectionRef.current = null;
    }
  }, [selection, showPopup]);

  // Handle Escape key to close popup
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && selection) {
        clearSelection();
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [selection, clearSelection]);

  // Handle click outside to close popup - DEFENSIVE approach
  useEffect(() => {
    if (!showPopup) {
      // Don't add listener if popup not showing
      return;
    }

    const handleClickOutside = (e: MouseEvent) => {
      const target = e.target as HTMLElement;
      
      // FIRST: Check if clicked inside the popup container
      const clickedPopup = target.closest('.highlight-popup-container');
      if (clickedPopup) {
        console.log('[HighlightManager] ✅ Click inside popup - keeping open');
        e.stopPropagation();
        e.preventDefault();
        return; // DO NOT CLOSE
      }
      
      // SECOND: Double-check for any popup-related elements
      const isPopupElement = 
        target.classList.contains('highlight-popup-container') ||
        target.closest('.highlight-popup-container') !== null;
      
      if (isPopupElement) {
        console.log('[HighlightManager] ✅ Popup element detected - keeping open');
        return; // DO NOT CLOSE
      }
      
      // THIRD: Only close if we're REALLY outside
      console.log('[HighlightManager] ❌ Click outside popup - closing');
      setShowPopup(false);
      setPopupPosition(null);
      savedSelectionRef.current = null;
      clearSelection();
    };

    // Add listener with a delay to let the popup render first
    console.log('[HighlightManager] ⏰ Scheduling click-outside listener (600ms delay)');
    const timeoutId = setTimeout(() => {
      console.log('[HighlightManager] ✅ Adding click-outside listener');
      document.addEventListener('click', handleClickOutside, false); // Use bubble phase, not capture
    }, 600);

    return () => {
      clearTimeout(timeoutId);
      document.removeEventListener('click', handleClickOutside, false);
      console.log('[HighlightManager] 🧹 Removed click-outside listener');
    };
  }, [showPopup, clearSelection]);

  const handleExplain = useCallback(() => {
    const currentSelection = savedSelectionRef.current;
    if (!currentSelection || currentSelection.text.length === 0) return;

    console.log('[HighlightManager] Explaining text in AI Chat');

    // Send to AI Chat with explanation prompt - Question first, then selected text
    const message = `Please explain this:\n\n${currentSelection.text}`;
    
    // Dispatch custom event that ChatInterface will listen for
    const event = new CustomEvent('learningHub:sendMessage', {
      detail: { message }
    });
    window.dispatchEvent(event);
    
    // Open sidebar and switch to AI Chat tab
    dispatch({ type: 'OPEN_SIDEBAR' });
    dispatch({ type: 'SET_ACTIVE_TAB', payload: 'chat' });

    // Clear popup and selection
    setShowPopup(false);
    setPopupPosition(null);
    savedSelectionRef.current = null;
    clearSelection();
  }, [dispatch, clearSelection]);

  const handleCustomQuestion = useCallback((question: string) => {
    const currentSelection = savedSelectionRef.current;
    if (!currentSelection || currentSelection.text.length === 0) return;

    console.log('[HighlightManager] Asking custom question about text');

    // Send to AI Chat - Question first, then selected text
    const message = `${question}\n\n${currentSelection.text}`;
    
    // Dispatch custom event that ChatInterface will listen for
    const event = new CustomEvent('learningHub:sendMessage', {
      detail: { message }
    });
    window.dispatchEvent(event);
    
    // Open sidebar and switch to AI Chat tab
    dispatch({ type: 'OPEN_SIDEBAR' });
    dispatch({ type: 'SET_ACTIVE_TAB', payload: 'chat' });

    // Clear popup and selection
    setShowPopup(false);
    setPopupPosition(null);
    savedSelectionRef.current = null;
    clearSelection();
  }, [dispatch, clearSelection]);

  const handleSave = useCallback(() => {
    const currentSelection = savedSelectionRef.current;
    if (!currentSelection || currentSelection.text.length === 0) return;

    // Validate text length (max 1000 characters)
    if (currentSelection.text.length > 1000) {
      alert('Highlight is too long. Please select up to 1000 characters.');
      return;
    }

    // Save highlight without explanation
    const highlightId = saveHighlight({
      text: currentSelection.text,
      pageUrl,
      pageTitle,
      startXPath: currentSelection.startXPath,
      endXPath: currentSelection.endXPath,
      startOffset: currentSelection.startOffset,
      endOffset: currentSelection.endOffset,
    });

    console.log('[HighlightManager] Highlight saved:', highlightId);

    // Show success feedback
    console.log('✓ Highlight saved!');

    // Clear popup and selection
    setShowPopup(false);
    setPopupPosition(null);
    savedSelectionRef.current = null;
    clearSelection();
  }, [pageUrl, pageTitle, saveHighlight, clearSelection]);

  const handleClose = useCallback(() => {
    setShowPopup(false);
    setPopupPosition(null);
    savedSelectionRef.current = null;
    clearSelection();
  }, [clearSelection]);

  // Only render popup if showPopup is true and we have a position
  if (!showPopup || !popupPosition || !savedSelectionRef.current) {
    console.log('[HighlightManager] Not rendering popup:', { 
      showPopup, 
      hasPosition: !!popupPosition,
      hasSavedSelection: !!savedSelectionRef.current
    });
    return null;
  }

  console.log('[HighlightManager] Rendering popup at:', popupPosition);

  return (
    <HighlightPopup
      position={popupPosition}
      onExplain={handleExplain}
      onSave={handleSave}
      onCustomQuestion={handleCustomQuestion}
      onClose={handleClose}
      isLoading={isExplaining}
    />
  );
}
