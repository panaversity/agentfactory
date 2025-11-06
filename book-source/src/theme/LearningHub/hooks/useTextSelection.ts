/**
 * useTextSelection Hook
 * Detects and manages text selection on the page with XPath serialization
 */

import { useState, useEffect, useCallback } from 'react';

interface TextSelection {
  text: string;
  range: Range | null;
  anchorNode: Node | null;
  // XPath for precise restoration
  startXPath: string;
  endXPath: string;
  startOffset: number;
  endOffset: number;
  // Position for popup placement
  boundingRect: DOMRect | null;
}

/**
 * Get XPath for a given node
 */
function getXPathForNode(node: Node): string {
  const parts: string[] = [];
  let current: Node | null = node;

  while (current && current.nodeType !== Node.DOCUMENT_NODE) {
    if (current.nodeType === Node.ELEMENT_NODE) {
      const element = current as Element;
      let index = 0;
      let sibling = element.previousSibling;

      // Count siblings with same tag name
      while (sibling) {
        if (sibling.nodeType === Node.ELEMENT_NODE && sibling.nodeName === element.nodeName) {
          index++;
        }
        sibling = sibling.previousSibling;
      }

      const tagName = element.nodeName.toLowerCase();
      const position = index > 0 ? `[${index + 1}]` : '';
      parts.unshift(`${tagName}${position}`);
    } else if (current.nodeType === Node.TEXT_NODE) {
      // For text nodes, find position among text node siblings
      let index = 0;
      let sibling = current.previousSibling;
      
      while (sibling) {
        if (sibling.nodeType === Node.TEXT_NODE) {
          index++;
        }
        sibling = sibling.previousSibling;
      }
      
      parts.unshift(`text()[${index + 1}]`);
    }

    current = current.parentNode;
  }

  return '/' + parts.join('/');
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
    console.error('[useTextSelection] Failed to evaluate XPath:', error);
    return null;
  }
}

export function useTextSelection(): {
  selection: TextSelection | null;
  clearSelection: () => void;
  restoreSelection: (selection: TextSelection) => boolean;
} {
  const [selection, setSelection] = useState<TextSelection | null>(null);

  const handleSelectionChange = useCallback(() => {
    const windowSelection = window.getSelection();
    
    if (!windowSelection || windowSelection.rangeCount === 0) {
      setSelection(null);
      return;
    }

    const text = windowSelection.toString().trim();
    
    // Only consider selections with >5 characters (reduced from 10 for better UX)
    if (text.length < 5) {
      setSelection(null);
      return;
    }

    try {
      const range = windowSelection.getRangeAt(0);
      const { startContainer, endContainer, startOffset, endOffset } = range;

      // Ensure selection is within article content (avoid selecting sidebar, nav, etc.)
      const commonAncestor = range.commonAncestorContainer;
      const ancestorElement = commonAncestor.nodeType === Node.ELEMENT_NODE 
        ? commonAncestor as Element
        : commonAncestor.parentElement;
      
      const articleElement = ancestorElement?.closest('article, main, .markdown');
      
      if (!articleElement) {
        console.log('[useTextSelection] Selection outside main content area - ignoring');
        setSelection(null);
        return;
      }

      // Note: No need to check for existing highlights since HighlightRenderer is disabled
      // Users can select the same text multiple times if they want

      // Get XPath for precise restoration
      const startXPath = getXPathForNode(startContainer);
      const endXPath = getXPathForNode(endContainer);

      // Get bounding rectangle for popup positioning
      const boundingRect = range.getBoundingClientRect();

      // Validate XPaths are not empty
      if (!startXPath || !endXPath) {
        console.warn('[useTextSelection] Failed to generate XPath for selection');
        setSelection(null);
        return;
      }

      console.log('[useTextSelection] Selection detected:', {
        text: text.substring(0, 50) + (text.length > 50 ? '...' : ''),
        length: text.length,
        startXPath,
        endXPath,
        startOffset,
        endOffset,
        inArticle: !!articleElement,
      });

      setSelection({
        text,
        range,
        anchorNode: startContainer,
        startXPath,
        endXPath,
        startOffset,
        endOffset,
        boundingRect,
      });
    } catch (error) {
      console.error('[useTextSelection] Error processing selection:', error);
      setSelection(null);
    }
  }, []);

  const clearSelection = useCallback(() => {
    window.getSelection()?.removeAllRanges();
    setSelection(null);
  }, []);

  const restoreSelection = useCallback((sel: TextSelection): boolean => {
    try {
      const startNode = getNodeFromXPath(sel.startXPath);
      const endNode = getNodeFromXPath(sel.endXPath);

      if (!startNode || !endNode) {
        console.warn('[useTextSelection] Cannot restore selection - nodes not found');
        return false;
      }

      const range = document.createRange();
      range.setStart(startNode, sel.startOffset);
      range.setEnd(endNode, sel.endOffset);

      const windowSelection = window.getSelection();
      if (windowSelection) {
        windowSelection.removeAllRanges();
        windowSelection.addRange(range);
        console.log('[useTextSelection] Selection restored');
        return true;
      }

      return false;
    } catch (error) {
      console.error('[useTextSelection] Failed to restore selection:', error);
      return false;
    }
  }, []);

  useEffect(() => {
    // Debounce selection changes to avoid excessive calls during drag
    let debounceTimer: NodeJS.Timeout | null = null;
    
    const debouncedHandler = () => {
      if (debounceTimer) {
        clearTimeout(debounceTimer);
      }
      
      debounceTimer = setTimeout(() => {
        handleSelectionChange();
      }, 200); // Wait 200ms after user stops selecting
    };

    // Listen for selection changes
    document.addEventListener('selectionchange', debouncedHandler);

    return () => {
      if (debounceTimer) {
        clearTimeout(debounceTimer);
      }
      document.removeEventListener('selectionchange', debouncedHandler);
    };
  }, [handleSelectionChange]);

  return {
    selection,
    clearSelection,
    restoreSelection,
  };
}
