import React, { useEffect, useCallback } from 'react';

interface HighlightDetectorProps {
  onHighlight: (selection: string, context?: string) => void;
  onLoading: (loading: boolean) => void;
  onError: (error: string | null) => void;
  onContent: (content: any | null) => void;
}

const API_BASE_URL = 'http://localhost:8000'; // FastAPI backend URL

const HighlightDetector: React.FC<HighlightDetectorProps> = ({ onHighlight, onLoading, onError, onContent }) => {
  const sendHighlightToBackend = useCallback(async (selectedText: string, selectionContext?: string) => {
    onLoading(true);
    onError(null);
    onContent(null);

    try {
      const response = await fetch(`${API_BASE_URL}/generate-ai-content`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          selectionType: 'text', // Assuming text for now, will be dynamic later
          selectionContent: selectedText,
          selectionContext: selectionContext,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Failed to fetch AI content');
      }

      const data = await response.json();
      onContent(data);
    } catch (err: any) {
      onError(err.message);
    } finally {
      onLoading(false);
    }
  }, [onLoading, onError, onContent]);

  const handleSelectionChange = useCallback(() => {
    const selection = window.getSelection();
    if (selection && selection.toString().length > 0) {
      const selectedText = selection.toString();
      onHighlight(selectedText);
      sendHighlightToBackend(selectedText);
    } else {
      onContent(null); // Clear content if no selection
    }
  }, [onHighlight, sendHighlightToBackend, onContent]);

  useEffect(() => {
    document.addEventListener('mouseup', handleSelectionChange);
    return () => {
      document.removeEventListener('mouseup', handleSelectionChange);
    };
  }, [handleSelectionChange]);

  return null;
};

export default HighlightDetector;
