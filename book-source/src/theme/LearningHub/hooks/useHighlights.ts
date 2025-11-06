/**
 * useHighlights Hook
 * Manages saved highlights with localStorage persistence and CRUD operations
 */

import { useState, useCallback, useEffect } from 'react';
import { useLearningHub } from '../context/LearningHubContext';
import type { Highlight } from '../types';

interface UseHighlightsReturn {
  highlights: Highlight[];
  saveHighlight: (highlight: Omit<Highlight, 'id' | 'timestamp'>) => string;
  deleteHighlight: (id: string) => void;
  getHighlight: (id: string) => Highlight | undefined;
  getHighlightsForPage: (pageUrl: string) => Highlight[];
}

export function useHighlights(pageUrl: string): UseHighlightsReturn {
  const { state, dispatch } = useLearningHub();
  
  // Get highlights for current page
  const highlights = state.savedHighlights[pageUrl] || [];

  const saveHighlight = useCallback((
    highlight: Omit<Highlight, 'id' | 'timestamp'>
  ): string => {
    const newHighlight: Highlight = {
      ...highlight,
      id: crypto.randomUUID(),
      timestamp: Date.now(),
    };

    console.log('[useHighlights] Saving highlight:', {
      id: newHighlight.id,
      pageUrl: highlight.pageUrl,
      textLength: highlight.text.length,
    });

    dispatch({
      type: 'ADD_HIGHLIGHT',
      payload: newHighlight,
    });

    return newHighlight.id;
  }, [dispatch]);

  const deleteHighlight = useCallback((id: string) => {
    console.log('[useHighlights] Deleting highlight:', id);
    
    dispatch({
      type: 'DELETE_HIGHLIGHT',
      payload: id,
    });
  }, [dispatch]);

  const getHighlight = useCallback((id: string): Highlight | undefined => {
    return highlights.find(h => h.id === id);
  }, [highlights]);

  const getHighlightsForPage = useCallback((url: string): Highlight[] => {
    return state.savedHighlights[url] || [];
  }, [state.savedHighlights]);

  return {
    highlights,
    saveHighlight,
    deleteHighlight,
    getHighlight,
    getHighlightsForPage,
  };
}
