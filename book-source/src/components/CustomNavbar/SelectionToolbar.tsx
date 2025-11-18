import React, { useState, useEffect, useCallback, useRef } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import './selectionToolbar.css';

interface TextAnchor {
  selectedText: string;
  prefix: string;
  suffix: string;
  startOffset: number;
  endOffset: number;
}

interface SelectionToolbarProps {
  onAction: (action: string, selectedText: string, elementId?: string, textAnchor?: TextAnchor) => void;
}

interface Position {
  top: number;
  left: number;
}

interface HighlightColor {
  name: string;
  color: string;
  label: string;
}

interface SummaryStyle {
  name: string;
  label: string;
  description: string;
}

const HIGHLIGHT_COLORS: HighlightColor[] = [
  { name: 'yellow', color: '#fef08a', label: 'Yellow' },
  { name: 'green', color: '#86efac', label: 'Green' },
  { name: 'blue', color: '#93c5fd', label: 'Blue' },
  { name: 'pink', color: '#f9a8d4', label: 'Pink' },
  { name: 'purple', color: '#d8b4fe', label: 'Purple' },
  { name: 'orange', color: '#fdba74', label: 'Orange' },
];

const SUMMARY_STYLES: SummaryStyle[] = [
  { name: 'short', label: 'Short', description: '2-3 sentence summary' },
  { name: 'medium', label: 'Medium', description: '1 paragraph summary' },
  { name: 'detailed', label: 'Detailed', description: 'Comprehensive with key points' },
];

const SelectionToolbar: React.FC<SelectionToolbarProps> = ({ onAction }) => {
  // Only render on client-side
  if (!ExecutionEnvironment.canUseDOM) {
    return null;
  }
  const [visible, setVisible] = useState(false);
  const [position, setPosition] = useState<Position>({ top: 0, left: 0 });
  const [selectedText, setSelectedText] = useState('');
  const [showColorPicker, setShowColorPicker] = useState(false);
  const [selectedColor, setSelectedColor] = useState<string>('yellow');
  const [showSummaryPicker, setShowSummaryPicker] = useState(false);
  const toolbarRef = useRef<HTMLDivElement>(null);
  const colorPickerRef = useRef<HTMLDivElement>(null);
  const summaryPickerRef = useRef<HTMLDivElement>(null);

  const calculateToolbarPosition = useCallback(() => {
    const selection = window.getSelection();
    if (!selection || selection.rangeCount === 0) return null;

    const range = selection.getRangeAt(0);
    const rect = range.getBoundingClientRect();

    // Get actual toolbar dimensions if it exists
    const toolbarWidth = toolbarRef.current?.offsetWidth || 320;
    const toolbarHeight = toolbarRef.current?.offsetHeight || 48;
    const gap = 2; // 2px gap between toolbar and selection

    // Calculate vertical position - position above the selection (viewport coordinates)
    // Use rect.top directly since position: fixed uses viewport coordinates
    let top = rect.top - toolbarHeight - gap;

    // If not enough space above, position below the selection
    if (top < 10) {
      // Not enough space above, position below
      top = rect.bottom + gap;
    }

    // Calculate horizontal position - centered on selection
    let left = rect.left + (rect.width / 2) - (toolbarWidth / 2);

    // Prevent toolbar from going off-screen horizontally
    const viewportWidth = window.innerWidth;
    const minLeft = 10;
    const maxLeft = viewportWidth - toolbarWidth - 10;

    left = Math.max(minLeft, Math.min(left, maxLeft));

    return { top, left };
  }, []);

  const handleSelection = useCallback(() => {
    const selection = window.getSelection();
    const text = selection?.toString().trim();

    if (text && text.length > 0) {
      const range = selection?.getRangeAt(0);
      const commonAncestor = range?.commonAncestorContainer;

      // Check if selection is within main content area
      // We'll check for common Docusaurus content containers
      const isInMainContent = (node: Node | null): boolean => {
        if (!node) return false;

        let element = node.nodeType === Node.ELEMENT_NODE ? node as Element : node.parentElement;

        while (element) {
          // Check for Docusaurus main content containers
          if (
            element.tagName === 'ARTICLE' ||
            element.classList?.contains('theme-doc-markdown') ||
            element.classList?.contains('markdown') ||
            element.tagName === 'MAIN' ||
            element.getAttribute('role') === 'main' ||
            element.classList?.contains('main-wrapper')
          ) {
            return true;
          }

          // Exclude navbar, sidebar, footer, and other UI elements
          if (
            element.classList?.contains('navbar') ||
            element.classList?.contains('custom-navbar') ||
            element.classList?.contains('sidebar') ||
            element.classList?.contains('menu') ||
            element.classList?.contains('footer') ||
            element.classList?.contains('table-of-contents') ||
            element.classList?.contains('pagination') ||
            element.tagName === 'NAV' ||
            element.tagName === 'ASIDE' ||
            element.tagName === 'FOOTER'
          ) {
            return false;
          }

          element = element.parentElement;
        }

        return false;
      };

      // Only show toolbar if selection is in main content
      if (!isInMainContent(commonAncestor)) {
        setVisible(false);
        setSelectedText('');
        return;
      }

      const newPosition = calculateToolbarPosition();

      if (newPosition) {
        setPosition(newPosition);
        setSelectedText(text);
        setVisible(true);
      }
    } else {
      setVisible(false);
      setSelectedText('');
    }
  }, [calculateToolbarPosition]);

  // Extract text anchor information for precise text location
  const extractTextAnchor = useCallback(() => {
    const selection = window.getSelection();
    if (!selection || selection.rangeCount === 0) return undefined;

    const range = selection.getRangeAt(0);
    const selectedText = range.toString().trim();

    if (!selectedText) return undefined;

    // Get the main content container
    const mainContent = document.querySelector('article, .theme-doc-markdown, main, [role="main"]');
    if (!mainContent) return undefined;

    // Get all text content from main content
    const fullText = mainContent.textContent || '';

    // Find the position of selected text in the full content
    const selectedIndex = fullText.indexOf(selectedText);

    if (selectedIndex === -1) return undefined;

    // Extract prefix (up to 100 characters before)
    const prefixStart = Math.max(0, selectedIndex - 100);
    const prefix = fullText.substring(prefixStart, selectedIndex);

    // Extract suffix (up to 100 characters after)
    const suffixEnd = Math.min(fullText.length, selectedIndex + selectedText.length + 100);
    const suffix = fullText.substring(selectedIndex + selectedText.length, suffixEnd);

    // Get offsets within the range
    const startOffset = range.startOffset;
    const endOffset = range.endOffset;

    return {
      selectedText,
      prefix: prefix.trim(),
      suffix: suffix.trim(),
      startOffset,
      endOffset,
    };
  }, []);

  // Generate element ID for headings (if selection is within a heading with ID)
  const getElementIdIfAvailable = useCallback((): string | undefined => {
    const selection = window.getSelection();
    if (!selection || selection.rangeCount === 0) return undefined;

    const range = selection.getRangeAt(0);
    let element = range.commonAncestorContainer;

    if (element.nodeType === Node.TEXT_NODE) {
      element = element.parentElement!;
    }

    // Traverse up to find a heading with an existing ID
    while (element && element !== document.body) {
      const tagName = (element as Element).tagName?.toLowerCase();
      if (['h1', 'h2', 'h3', 'h4', 'h5', 'h6'].includes(tagName)) {
        const existingId = (element as Element).getAttribute('id');
        if (existingId) {
          return existingId;
        }
      }
      element = (element as Element).parentElement!;
    }

    return undefined;
  }, []);

  const handleAction = useCallback((action: string) => {
    if (selectedText) {
      let elementId: string | undefined;
      let textAnchor: TextAnchor | undefined;

      // For bookmarks and notes, capture both element ID (if available) and text anchor
      if (action === 'Bookmark' || action === 'Notes') {
        elementId = getElementIdIfAvailable();
        textAnchor = extractTextAnchor();
      }

      onAction(action, selectedText, elementId, textAnchor);
      // Collapse left sidebar when opening right drawer for maximum space
      window.dispatchEvent(new CustomEvent('collapseSidebar'));
      // Clear selection after action
      window.getSelection()?.removeAllRanges();
      setVisible(false);
    }
  }, [selectedText, onAction, getElementIdIfAvailable, extractTextAnchor]);

  const buildSummaryPrompt = useCallback((text: string, style: string): string => {
    // Get page metadata
    const pageTitle = document.title;
    const pageUrl = window.location.pathname;

    // Extract chapter/section if available
    const h1 = document.querySelector('h1')?.textContent || 'Unknown Section';

    // Style-specific instructions
    const styleInstructions: Record<string, string> = {
      short: 'Provide a concise 2-3 sentence summary highlighting the key takeaway',
      medium: 'Provide a comprehensive 1-paragraph summary covering the main points',
      detailed: 'Provide a detailed summary with key points listed as bullets, covering all important concepts'
    };

    return `Context: This text is from "${pageTitle}" (${pageUrl}), section: "${h1}"

${styleInstructions[style] || styleInstructions.medium}.

Selected Text:
"""
${text}
"""

Please provide the summary now.`;
  }, []);

  const handleExplain = useCallback(() => {
    if (!selectedText) return;

    // Get page metadata
    const pageTitle = document.title;
    const pageUrl = window.location.pathname;
    const h1 = document.querySelector('h1')?.textContent || 'Unknown Section';

    // Build the explain prompt with context
    const explainPrompt = `Context: This text is from "${pageTitle}" (${pageUrl}), section: "${h1}"

Please explain the following text in detail. Break down complex concepts, provide examples if helpful, and make it easy to understand:

"""
${selectedText}
"""`;

    // Dispatch custom event to open PanaChat with the message
    window.dispatchEvent(new CustomEvent('openPanaChatWithMessage', {
      detail: { message: explainPrompt }
    }));

    // Clear selection and hide toolbar
    window.getSelection()?.removeAllRanges();
    setVisible(false);
  }, [selectedText]);

  const handleSummary = useCallback((style: string) => {
    if (!selectedText) return;

    // Build the summary prompt with context
    const summaryPrompt = buildSummaryPrompt(selectedText, style);

    // Dispatch custom event to open PanaChat with the message
    window.dispatchEvent(new CustomEvent('openPanaChatWithMessage', {
      detail: { message: summaryPrompt }
    }));

    // Clear selection and hide toolbar
    window.getSelection()?.removeAllRanges();
    setVisible(false);
    setShowSummaryPicker(false);
  }, [selectedText, buildSummaryPrompt]);

  const toggleSummaryPicker = useCallback(() => {
    setShowSummaryPicker(prev => !prev);
    // Close color picker if open
    if (showColorPicker) {
      setShowColorPicker(false);
    }
  }, [showColorPicker]);

  const createHighlightSpan = (text: string, color: string, highlightId: string) => {
    const span = document.createElement('span');
    span.className = `text-highlight text-highlight-${color}`;
    span.setAttribute('data-highlight-color', color);
    span.setAttribute('data-highlight-id', highlightId);
    span.textContent = text;

    // Add delete button to each highlight
    const deleteBtn = document.createElement('span');
    deleteBtn.className = 'text-highlight__delete';
    deleteBtn.textContent = '×';
    deleteBtn.title = 'Remove highlight';
    deleteBtn.setAttribute('aria-label', 'Remove highlight');
    deleteBtn.setAttribute('data-highlight-id', highlightId);
    span.appendChild(deleteBtn);

    return span;
  };

  const wrapTextNodesInRange = (range: Range, baseId: string, color: string) => {
    const fragment = range.extractContents();
    const container = document.createDocumentFragment();
    let highlightCounter = 0;

    // Process the fragment to wrap text nodes individually
    const processNode = (node: Node): Node | DocumentFragment => {
      if (node.nodeType === Node.TEXT_NODE) {
        const text = node.textContent?.trim();
        if (text && text.length > 0) {
          // Create a unique highlight for each text node
          const highlightId = `${baseId}-${highlightCounter++}`;
          return createHighlightSpan(node.textContent || '', color, highlightId);
        } else {
          // Preserve whitespace/empty text nodes
          return document.createTextNode(node.textContent || '');
        }
      } else if (node.nodeType === Node.ELEMENT_NODE) {
        // Clone the element and process its children
        const clone = (node as Element).cloneNode(false) as Element;

        // Process all child nodes
        Array.from(node.childNodes).forEach(child => {
          const processed = processNode(child);
          if (processed instanceof DocumentFragment) {
            clone.appendChild(processed);
          } else {
            clone.appendChild(processed);
          }
        });

        return clone;
      } else {
        // For other node types, clone as-is
        return node.cloneNode(true);
      }
    };

    // Process all nodes in the fragment
    Array.from(fragment.childNodes).forEach(node => {
      const processed = processNode(node);
      if (processed instanceof DocumentFragment) {
        container.appendChild(processed);
      } else {
        container.appendChild(processed);
      }
    });

    return container;
  };

  const handleHighlight = useCallback((color: string) => {
    const selection = window.getSelection();
    if (!selection || selection.rangeCount === 0) return;

    const range = selection.getRangeAt(0);
    const selectedContent = range.toString().trim();

    if (!selectedContent) return;

    // Generate a base ID for this highlight group
    const baseHighlightId = `highlight-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;

    // Wrap text nodes while preserving structure
    const highlightedContent = wrapTextNodesInRange(range, baseHighlightId, color);
    range.insertNode(highlightedContent);

    // Save to localStorage
    saveHighlight(baseHighlightId, color, selectedContent);

    // Clear selection
    selection.removeAllRanges();
    setVisible(false);
    setShowColorPicker(false);
  }, []);

  const saveHighlight = (id: string, color: string, text: string) => {
    const pageUrl = window.location.pathname;
    const highlights = JSON.parse(localStorage.getItem('pageHighlights') || '{}');

    if (!highlights[pageUrl]) {
      highlights[pageUrl] = [];
    }

    highlights[pageUrl].push({
      id,
      color,
      text,
      timestamp: Date.now(),
    });

    localStorage.setItem('pageHighlights', JSON.stringify(highlights));
  };

  const restoreHighlights = useCallback(() => {
    const pageUrl = window.location.pathname;
    const highlights = JSON.parse(localStorage.getItem('pageHighlights') || '{}');
    const pageHighlights = highlights[pageUrl] || [];

    if (pageHighlights.length === 0) return;

    // Find all text nodes in the main content
    const mainContent = document.querySelector('article, .theme-doc-markdown, main, [role="main"]');
    if (!mainContent) return;

    pageHighlights.forEach((highlight: any) => {
      // Try to find and highlight the text
      const walker = document.createTreeWalker(
        mainContent,
        NodeFilter.SHOW_TEXT,
        null
      );

      let node;
      while ((node = walker.nextNode())) {
        const text = node.textContent || '';
        const index = text.indexOf(highlight.text);

        if (index !== -1 && !node.parentElement?.classList.contains('text-highlight')) {
          try {
            const range = document.createRange();
            range.setStart(node, index);
            range.setEnd(node, index + highlight.text.length);

            const highlightSpan = document.createElement('span');
            highlightSpan.className = `text-highlight text-highlight-${highlight.color}`;
            highlightSpan.setAttribute('data-highlight-color', highlight.color);
            highlightSpan.setAttribute('data-highlight-text', highlight.text);
            highlightSpan.setAttribute('data-highlight-id', highlight.id);

            range.surroundContents(highlightSpan);

            // Add delete button (as a separate element)
            const deleteBtn = document.createElement('span');
            deleteBtn.className = 'text-highlight__delete';
            deleteBtn.textContent = '×';
            deleteBtn.title = 'Remove highlight';
            deleteBtn.setAttribute('aria-label', 'Remove highlight');
            highlightSpan.appendChild(deleteBtn);
          } catch (error) {
            // Skip if highlighting fails
            console.warn('Failed to restore highlight:', error);
          }
        }
      }
    });
  }, []);

  const toggleColorPicker = useCallback(() => {
    setShowColorPicker(prev => !prev);
  }, []);

  // Reposition toolbar after it's rendered to get accurate dimensions
  useEffect(() => {
    if (visible && toolbarRef.current) {
      const newPosition = calculateToolbarPosition();
      if (newPosition) {
        setPosition(newPosition);
      }
    }
  }, [visible, calculateToolbarPosition]);

  useEffect(() => {
    // Add selection listeners
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    // Listen for selection changes to hide toolbar when text is deselected
    const handleSelectionChange = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      // If no text is selected, hide the toolbar immediately
      if (!text || text.length === 0) {
        setVisible(false);
        setShowColorPicker(false);
        setShowSummaryPicker(false);
      }
    };

    document.addEventListener('selectionchange', handleSelectionChange);

    // Click outside to hide toolbar and pickers
    const handleClickOutside = (event: MouseEvent) => {
      if (toolbarRef.current && !toolbarRef.current.contains(event.target as Node)) {
        const selection = window.getSelection();
        if (!selection?.toString().trim()) {
          setVisible(false);
          setShowColorPicker(false);
          setShowSummaryPicker(false);
        }
      }

      // Close color picker if clicking outside of it
      if (colorPickerRef.current && !colorPickerRef.current.contains(event.target as Node)) {
        if (!(event.target as Element).closest('.selection-toolbar__highlighter-button')) {
          setShowColorPicker(false);
        }
      }

      // Close summary picker if clicking outside of it
      if (summaryPickerRef.current && !summaryPickerRef.current.contains(event.target as Node)) {
        if (!(event.target as Element).closest('.selection-toolbar__summary-button')) {
          setShowSummaryPicker(false);
        }
      }
    };

    // Update toolbar position on scroll when visible
    const handleScroll = () => {
      if (visible) {
        const newPosition = calculateToolbarPosition();
        if (newPosition) {
          setPosition(newPosition);
        }
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    window.addEventListener('scroll', handleScroll, true); // Use capture phase for all scroll events

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
      document.removeEventListener('selectionchange', handleSelectionChange);
      document.removeEventListener('mousedown', handleClickOutside);
      window.removeEventListener('scroll', handleScroll, true);
    };
  }, [handleSelection, visible, calculateToolbarPosition]);

  const removeHighlight = useCallback((highlightId: string, element: HTMLElement) => {
    // Remove just this specific highlight element
    const parent = element.parentNode;
    if (parent) {
      // Remove delete button
      const deleteBtn = element.querySelector('.text-highlight__delete');
      if (deleteBtn) {
        deleteBtn.remove();
      }

      // Move text content back
      const textNode = document.createTextNode(element.textContent || '');
      parent.replaceChild(textNode, element);

      // Merge adjacent text nodes
      parent.normalize();
    }

    // Check if this was the last highlight in a group
    const baseId = highlightId.split('-').slice(0, -1).join('-');
    const remainingHighlights = document.querySelectorAll(`[data-highlight-id^="${baseId}"]`);

    // If no more highlights from this group, remove from localStorage
    if (remainingHighlights.length === 0) {
      const pageUrl = window.location.pathname;
      const highlights = JSON.parse(localStorage.getItem('pageHighlights') || '{}');

      if (highlights[pageUrl]) {
        highlights[pageUrl] = highlights[pageUrl].filter((h: any) => !h.id.startsWith(baseId));

        if (highlights[pageUrl].length === 0) {
          delete highlights[pageUrl];
        }

        localStorage.setItem('pageHighlights', JSON.stringify(highlights));
      }
    }
  }, []);

  const setupHighlightListeners = useCallback(() => {
    // Add delete button functionality to all highlights
    const handleHighlightInteraction = (e: MouseEvent) => {
      const target = e.target as HTMLElement;

      // Check if clicked on delete button
      if (target.classList.contains('text-highlight__delete')) {
        const highlightSpan = target.closest('.text-highlight') as HTMLElement;
        if (highlightSpan) {
          const highlightId = highlightSpan.getAttribute('data-highlight-id');
          if (highlightId) {
            removeHighlight(highlightId, highlightSpan);
          }
        }
        e.stopPropagation();
        return;
      }
    };

    document.addEventListener('click', handleHighlightInteraction);

    return () => {
      document.removeEventListener('click', handleHighlightInteraction);
    };
  }, [removeHighlight]);

  // Restore highlights when component mounts or page changes
  useEffect(() => {
    // Wait for content to load
    const timer = setTimeout(() => {
      restoreHighlights();
      // Setup listeners after highlights are restored
      setTimeout(() => {
        addDeleteButtons();
      }, 100);
    }, 500);

    return () => clearTimeout(timer);
  }, [restoreHighlights, window.location.pathname]);

  // Listen for URL changes (Docusaurus client-side navigation)
  useEffect(() => {
    const handleRouteChange = () => {
      // Clear existing highlights from DOM before restoring
      const existingHighlights = document.querySelectorAll('.text-highlight');
      existingHighlights.forEach(highlight => {
        const parent = highlight.parentNode;
        if (parent) {
          const textContent = highlight.textContent?.replace('×', '') || '';
          const textNode = document.createTextNode(textContent);
          parent.replaceChild(textNode, highlight);
          parent.normalize();
        }
      });

      // Wait for new page content to render, then restore highlights
      setTimeout(() => {
        restoreHighlights();
        setTimeout(() => {
          addDeleteButtons();
        }, 100);
      }, 500);
    };

    // Listen for popstate (browser back/forward)
    window.addEventListener('popstate', handleRouteChange);

    // Create observer for URL changes
    let lastUrl = window.location.pathname;
    const urlObserver = setInterval(() => {
      if (window.location.pathname !== lastUrl) {
        lastUrl = window.location.pathname;
        handleRouteChange();
      }
    }, 100);

    return () => {
      window.removeEventListener('popstate', handleRouteChange);
      clearInterval(urlObserver);
    };
  }, [restoreHighlights]);

  // Setup highlight interaction listeners
  useEffect(() => {
    const cleanup = setupHighlightListeners();
    return cleanup;
  }, [setupHighlightListeners]);

  const addDeleteButtons = () => {
    // Add delete buttons to all existing highlights
    const highlights = document.querySelectorAll('.text-highlight');
    highlights.forEach((highlight) => {
      if (!highlight.querySelector('.text-highlight__delete')) {
        const deleteBtn = document.createElement('span');
        deleteBtn.className = 'text-highlight__delete';
        deleteBtn.textContent = '×';
        deleteBtn.title = 'Remove highlight';
        deleteBtn.setAttribute('aria-label', 'Remove highlight');
        highlight.appendChild(deleteBtn);
      }
    });
  };

  if (!visible) return null;

  return (
    <div
      ref={toolbarRef}
      className="selection-toolbar"
      style={{
        top: `${position.top}px`,
        left: `${position.left}px`,
      }}
    >
      <button
        className="selection-toolbar__button"
        onClick={() => handleAction('Bookmark')}
        title="Add to Bookmark"
      >
        <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
          <path
            d="M3 2C3 1.44772 3.44772 1 4 1H12C12.5523 1 13 1.44772 13 2V15L8 12L3 15V2Z"
            stroke="currentColor"
            strokeWidth="1.5"
            strokeLinecap="round"
            strokeLinejoin="round"
          />
        </svg>
        <span>Bookmark</span>
      </button>
{/* 
      <button
        className="selection-toolbar__button"
        onClick={() => handleAction('Mindmap')}
        title="Add to Mindmap"
      >
        <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
          <circle cx="8" cy="8" r="2" stroke="currentColor" strokeWidth="1.5" />
          <circle cx="3" cy="3" r="1.5" stroke="currentColor" strokeWidth="1.5" />
          <circle cx="13" cy="3" r="1.5" stroke="currentColor" strokeWidth="1.5" />
          <circle cx="3" cy="13" r="1.5" stroke="currentColor" strokeWidth="1.5" />
          <circle cx="13" cy="13" r="1.5" stroke="currentColor" strokeWidth="1.5" />
          <path d="M6.5 7L4 4.5M9.5 7L12 4.5M6.5 9L4 11.5M9.5 9L12 11.5" stroke="currentColor" strokeWidth="1.5" />
          </svg>
        <span>Mindmap</span>
      </button>
*/}

      <button
        className="selection-toolbar__button"
        onClick={() => handleAction('Notes')}
        title="Add to Notes"
      >
        <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
          <path
            d="M4 1H12C12.5523 1 13 1.44772 13 2V14C13 14.5523 12.5523 15 12 15H4C3.44772 15 3 14.5523 3 14V2C3 1.44772 3.44772 1 4 1Z"
            stroke="currentColor"
            strokeWidth="1.5"
            strokeLinecap="round"
            strokeLinejoin="round"
          />
          <path d="M6 5H10M6 8H10M6 11H8" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" />
        </svg>
        <span>Notes</span>
      </button>

      <button
        className="selection-toolbar__button"
        onClick={handleExplain}
        title="Explain selected text"
      >
        <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
          <circle
            cx="8"
            cy="8"
            r="6"
            stroke="currentColor"
            strokeWidth="1.5"
            strokeLinecap="round"
            strokeLinejoin="round"
          />
          <path
            d="M8 12V8M8 5.5V5"
            stroke="currentColor"
            strokeWidth="1.5"
            strokeLinecap="round"
            strokeLinejoin="round"
          />
        </svg>
        <span>Explain</span>
      </button>

{/*
      <button
        className="selection-toolbar__button"
        onClick={() => handleAction('Assessment')}
        title="Add to Assessment"
      >
        <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
          <path
            d="M4 1H12C12.5523 1 13 1.44772 13 2V14C13 14.5523 12.5523 15 12 15H4C3.44772 15 3 14.5523 3 14V2C3 1.44772 3.44772 1 4 1Z"
            stroke="currentColor"
            strokeWidth="1.5"
          />
          <circle cx="6" cy="5" r="0.5" fill="currentColor" />
          <circle cx="6" cy="8" r="0.5" fill="currentColor" />
          <circle cx="6" cy="11" r="0.5" fill="currentColor" />
          <path d="M8 5H10M8 8H10M8 11H10" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" />
        </svg>
        <span>Assessment</span>
        </button>
*/}

      {/* Summary with Style Picker */}
      <div className="selection-toolbar__summary">
        <button
          className="selection-toolbar__button selection-toolbar__summary-button"
          onClick={toggleSummaryPicker}
          title="Summarize selected text"
        >
          <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
            <path
              d="M2 3H14M2 8H14M2 13H10"
              stroke="currentColor"
              strokeWidth="1.5"
              strokeLinecap="round"
              strokeLinejoin="round"
            />
          </svg>
          <span>Summary</span>
          <svg
            width="12"
            height="12"
            viewBox="0 0 12 12"
            fill="none"
            className="selection-toolbar__dropdown-arrow"
          >
            <path
              d="M3 5L6 8L9 5"
              stroke="currentColor"
              strokeWidth="1.5"
              strokeLinecap="round"
              strokeLinejoin="round"
            />
          </svg>
        </button>

        {/* Summary Style Picker Dropdown */}
        {showSummaryPicker && (
          <div ref={summaryPickerRef} className="selection-toolbar__summary-picker">
            {SUMMARY_STYLES.map((styleOption) => (
              <button
                key={styleOption.name}
                className="selection-toolbar__summary-option"
                onClick={() => handleSummary(styleOption.name)}
                title={styleOption.description}
              >
                <span className="selection-toolbar__summary-option-label">{styleOption.label}</span>
                <span className="selection-toolbar__summary-option-description">{styleOption.description}</span>
              </button>
            ))}
          </div>
        )}
      </div>

      {/* Highlighter with Color Picker */}
      <div className="selection-toolbar__highlighter">
        <button
          className="selection-toolbar__button selection-toolbar__highlighter-button"
          onClick={toggleColorPicker}
          title="Highlight text"
        >
          <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
            <path
              d="M10.5 2L14 5.5L8.5 11L5 14.5L2 13L3.5 10L9 4.5L10.5 2Z"
              stroke="currentColor"
              strokeWidth="1.5"
              strokeLinecap="round"
              strokeLinejoin="round"
            />
            <path
              d="M9 4.5L11.5 7"
              stroke="currentColor"
              strokeWidth="1.5"
              strokeLinecap="round"
            />
          </svg>
          <span>Highlight</span>
          <svg
            width="12"
            height="12"
            viewBox="0 0 12 12"
            fill="none"
            className="selection-toolbar__dropdown-arrow"
          >
            <path
              d="M3 5L6 8L9 5"
              stroke="currentColor"
              strokeWidth="1.5"
              strokeLinecap="round"
              strokeLinejoin="round"
            />
          </svg>
        </button>

        {/* Color Picker Dropdown */}
        {showColorPicker && (
          <div ref={colorPickerRef} className="selection-toolbar__color-picker">
            {HIGHLIGHT_COLORS.map((colorOption) => (
              <button
                key={colorOption.name}
                className={`selection-toolbar__color-option ${selectedColor === colorOption.name ? 'selection-toolbar__color-option--selected' : ''}`}
                style={{ backgroundColor: colorOption.color }}
                onClick={() => {
                  setSelectedColor(colorOption.name);
                  handleHighlight(colorOption.name);
                }}
                title={colorOption.label}
              >
                {selectedColor === colorOption.name && (
                  <svg width="12" height="12" viewBox="0 0 12 12" fill="none">
                    <path
                      d="M2 6L5 9L10 3"
                      stroke="#000"
                      strokeWidth="2"
                      strokeLinecap="round"
                      strokeLinejoin="round"
                    />
                  </svg>
                )}
              </button>
            ))}
          </div>
        )}
      </div>
    </div>
  );
};

export default SelectionToolbar;
