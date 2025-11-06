/**
 * HighlightRenderer Component
 * Restores saved highlights on page load and makes them clickable
 */

import React, { useEffect, useState, useCallback } from 'react';
import { useHighlights } from '../../hooks/useHighlights';
import { useLearningHub } from '../../context/LearningHubContext';
import type { Highlight } from '../../types';

interface HighlightRendererProps {
  pageUrl: string;
}

/**
 * Get node from XPath
 */
function getNodeFromXPath(xpath: string): Node | null {
  try {
    const result = document.evaluate(
      xpath,
      document,
      null,
      XPathResult.FIRST_ORDERED_NODE_TYPE,
      null
    );
    return result.singleNodeValue;
  } catch (error) {
    console.error('[HighlightRenderer] Failed to evaluate XPath:', error);
    return null;
  }
}

/**
 * Apply highlight marker to a range
 */
function applyHighlightToRange(highlight: Highlight): HTMLElement | null {
  try {
    const startNode = getNodeFromXPath(highlight.startXPath);
    const endNode = getNodeFromXPath(highlight.endXPath);

    if (!startNode || !endNode) {
      console.warn('[HighlightRenderer] Cannot restore highlight - nodes not found:', highlight.id);
      return null;
    }

    const range = document.createRange();
    range.setStart(startNode, highlight.startOffset);
    range.setEnd(endNode, highlight.endOffset);

    // Create highlight marker element
    const mark = document.createElement('mark');
    mark.className = 'learning-hub-highlight';
    mark.dataset.highlightId = highlight.id;
    mark.style.background = 'linear-gradient(to bottom, transparent 50%, #ffeb3b 50%)';
    mark.style.backgroundSize = '100% 200%';
    mark.style.backgroundPosition = '0 0';
    mark.style.cursor = 'pointer';
    mark.style.transition = 'background-position 0.3s ease';
    mark.style.padding = '2px 0';
    mark.style.borderRadius = '2px';
    mark.title = 'Click to view explanation';
    // CRITICAL: Explicitly inherit color from parent to preserve original text color
    mark.style.color = 'inherit';

    // Wrap the range contents
    try {
      range.surroundContents(mark);
      return mark;
    } catch (error) {
      console.error('[HighlightRenderer] Failed to wrap range:', error);
      return null;
    }
  } catch (error) {
    console.error('[HighlightRenderer] Failed to apply highlight:', error);
    return null;
  }
}

export function HighlightRenderer({ pageUrl }: HighlightRendererProps) {
  const { highlights } = useHighlights(pageUrl);
  const { dispatch } = useLearningHub();

  // Restore highlights on page load and when highlights change
  useEffect(() => {
    console.log('[HighlightRenderer] Restoring highlights:', highlights.length);

    // Clear all existing highlight markers first
    const existingMarks = document.querySelectorAll('.learning-hub-highlight');
    existingMarks.forEach((mark) => {
      if (mark.parentNode && mark.textContent) {
        const textNode = document.createTextNode(mark.textContent);
        mark.parentNode.replaceChild(textNode, mark);
      }
    });

    // Apply each highlight
    const renderedElements: HTMLElement[] = [];
    highlights.forEach((highlight) => {
      const element = applyHighlightToRange(highlight);
      if (element) {
        renderedElements.push(element);

        // Add click handler
        element.addEventListener('click', () => {
          handleHighlightClick(highlight);
        });

        // Add hover effect
        element.addEventListener('mouseenter', () => {
          element.style.backgroundPosition = '0 100%';
          element.style.boxShadow = '0 2px 4px rgba(255, 235, 59, 0.3)';
        });

        element.addEventListener('mouseleave', () => {
          element.style.backgroundPosition = '0 0';
          element.style.boxShadow = 'none';
        });
      }
    });

    // Cleanup on unmount
    return () => {
      renderedElements.forEach((element) => {
        if (element.parentNode && element.textContent) {
          const textNode = document.createTextNode(element.textContent);
          element.parentNode.replaceChild(textNode, element);
        }
      });
    };
  }, [highlights, dispatch]);

  const handleHighlightClick = useCallback((highlight: Highlight) => {
    console.log('[HighlightRenderer] Highlight clicked:', highlight.id);

    // Open sidebar if closed
    dispatch({ type: 'TOGGLE_SIDEBAR' });
    
    // Switch to highlights tab
    dispatch({ type: 'SET_ACTIVE_TAB', payload: 'highlights' });

    // Scroll the highlight into view in the sidebar
    // This will be handled by the HighlightsList component
    setTimeout(() => {
      const highlightElement = document.querySelector(`[data-highlight-id="${highlight.id}"]`);
      if (highlightElement) {
        highlightElement.scrollIntoView({ behavior: 'smooth', block: 'center' });
      }
    }, 300); // Wait for sidebar animation
  }, [dispatch]);

  // This component doesn't render anything visible
  return null;
}
