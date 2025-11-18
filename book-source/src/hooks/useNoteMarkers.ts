import { useEffect, useCallback } from 'react';
import { useLocation } from '@docusaurus/router';
import ReactDOM from 'react-dom/client';
import React from 'react';
import { useNotes } from '../contexts/NotesContext';
import NoteIndicator from '../components/NoteIndicator';
import type { Note } from '../contexts/NotesContext';

interface UseNoteMarkersOptions {
  onNoteClick: (noteId: string) => void;
}

export const useNoteMarkers = ({ onNoteClick }: UseNoteMarkersOptions) => {
  const location = useLocation();
  const { getNotesByPage, notes } = useNotes();

  // Find text in the page using text anchor (prefix, selected text, suffix)
  const findTextInPage = useCallback((textAnchor: Note['textAnchor']): Range | null => {
    if (!textAnchor) return null;

    const mainContent = document.querySelector('article, .theme-doc-markdown, main, [role="main"]');
    if (!mainContent) return null;

    const fullText = mainContent.textContent || '';
    const { selectedText, prefix, suffix } = textAnchor;

    // Strategy 1: Try to find with prefix + selectedText + suffix
    let fullMatch = `${prefix}${selectedText}${suffix}`;
    let index = fullText.indexOf(fullMatch);

    if (index !== -1) {
      const startIndex = index + prefix.length;
      const endIndex = startIndex + selectedText.length;
      return createRangeFromIndices(mainContent, startIndex, endIndex);
    }

    // Strategy 2: Try selectedText with prefix only
    const prefixMatch = `${prefix}${selectedText}`;
    index = fullText.indexOf(prefixMatch);

    if (index !== -1) {
      const startIndex = index + prefix.length;
      const endIndex = startIndex + selectedText.length;
      return createRangeFromIndices(mainContent, startIndex, endIndex);
    }

    // Strategy 3: Try selectedText with suffix only
    const suffixMatch = `${selectedText}${suffix}`;
    index = fullText.indexOf(suffixMatch);

    if (index !== -1) {
      const endIndex = index + selectedText.length;
      return createRangeFromIndices(mainContent, index, endIndex);
    }

    // Strategy 4: Try just the selected text
    index = fullText.indexOf(selectedText);

    if (index !== -1) {
      const endIndex = index + selectedText.length;
      return createRangeFromIndices(mainContent, index, endIndex);
    }

    return null;
  }, []);

  // Create a Range object from character indices in the full text
  const createRangeFromIndices = useCallback((
    container: Element,
    startIndex: number,
    endIndex: number
  ): Range | null => {
    const walker = document.createTreeWalker(
      container,
      NodeFilter.SHOW_TEXT,
      null
    );

    let currentIndex = 0;
    let startNode: Node | null = null;
    let startOffset = 0;
    let endNode: Node | null = null;
    let endOffset = 0;

    let node: Node | null;
    while ((node = walker.nextNode())) {
      const textLength = node.textContent?.length || 0;
      const nextIndex = currentIndex + textLength;

      if (startNode === null && startIndex >= currentIndex && startIndex < nextIndex) {
        startNode = node;
        startOffset = startIndex - currentIndex;
      }

      if (endNode === null && endIndex >= currentIndex && endIndex <= nextIndex) {
        endNode = node;
        endOffset = endIndex - currentIndex;
      }

      if (startNode && endNode) break;

      currentIndex = nextIndex;
    }

    if (startNode && endNode) {
      const range = document.createRange();
      range.setStart(startNode, startOffset);
      range.setEnd(endNode, endOffset);
      return range;
    }

    return null;
  }, []);

  // Insert note indicator after a text range
  const insertNoteIndicator = useCallback((range: Range, note: Note) => {
    try {
      // Create a marker element to hold the React component
      const markerContainer = document.createElement('span');
      markerContainer.className = 'note-marker-container';
      markerContainer.setAttribute('data-note-marker', note.id);

      // Insert the marker after the range
      const endNode = range.endContainer;
      const endOffset = range.endOffset;

      if (endNode.nodeType === Node.TEXT_NODE) {
        const parent = endNode.parentNode;
        if (parent) {
          // Split the text node at the end offset if needed
          if (endOffset < (endNode.textContent?.length || 0)) {
            const textNode = endNode as Text;
            textNode.splitText(endOffset);
          }

          // Insert the marker after the text node
          if (endNode.nextSibling) {
            parent.insertBefore(markerContainer, endNode.nextSibling);
          } else {
            parent.appendChild(markerContainer);
          }

          // Render the React component into the marker
          const root = ReactDOM.createRoot(markerContainer);
          root.render(
            React.createElement(NoteIndicator, {
              noteId: note.id,
              noteName: note.name,
              onClick: onNoteClick,
            })
          );

          // Store the root for cleanup
          (markerContainer as any)._reactRoot = root;
        }
      }
    } catch (error) {
      console.error('Failed to insert note indicator:', error);
    }
  }, [onNoteClick]);

  // Remove all note markers from the page
  const removeAllMarkers = useCallback(() => {
    const markers = document.querySelectorAll('[data-note-marker]');
    markers.forEach((marker) => {
      try {
        // Unmount the React component
        const root = (marker as any)._reactRoot;
        if (root) {
          root.unmount();
        }
        // Remove the marker element
        marker.remove();
      } catch (error) {
        console.error('Failed to remove note marker:', error);
      }
    });
  }, []);

  // Insert markers for all notes on the current page
  const insertMarkers = useCallback(() => {
    // Wait for the DOM to be ready
    setTimeout(() => {
      const pageNotes = getNotesByPage(location.pathname);

      pageNotes.forEach((note) => {
        if (note.textAnchor) {
          const range = findTextInPage(note.textAnchor);
          if (range) {
            insertNoteIndicator(range, note);
          }
        }
      });
    }, 500); // Delay to ensure content is fully rendered
  }, [location.pathname, getNotesByPage, findTextInPage, insertNoteIndicator]);

  // Effect to manage note markers
  useEffect(() => {
    // Remove existing markers
    removeAllMarkers();

    // Insert new markers
    insertMarkers();

    // Cleanup on unmount or when dependencies change
    return () => {
      removeAllMarkers();
    };
  }, [location.pathname, notes, insertMarkers, removeAllMarkers]);

  return {
    refreshMarkers: insertMarkers,
  };
};
