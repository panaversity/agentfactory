import React, { useEffect, useCallback, useState } from 'react';
import HighlightConfirmation from './HighlightConfirmation';

interface HighlightDetectorProps {
  onHighlight: (selection: string, context?: string) => void;
  onLoading: (loading: boolean) => void;
  onError: (error: string | null) => void;
  onContent: (content: any | null) => void;
}

const API_BASE_URL = 'http://localhost:8000'; // FastAPI backend URL
const DEFAULT_API_KEY = 'AIzaSyCihdH5ZMgckh8Jlan7C_edZIxLNmmJ1uc'; // Default test API key

const HighlightDetector: React.FC<HighlightDetectorProps> = ({ onHighlight, onLoading, onError, onContent }) => {
  const [highlightedText, setHighlightedText] = useState('');
  const [showConfirmation, setShowConfirmation] = useState(false);
  const [confirmationPosition, setConfirmationPosition] = useState({ x: 0, y: 0 });
  const [pendingRequest, setPendingRequest] = useState<{ text: string; context?: string } | null>(null);

  // Helper function to find the heading context
  const findHeadingContext = useCallback((): { previousContext: string; nextContext: string } => {
    const selection = window.getSelection();
    if (!selection || selection.rangeCount === 0) {
      return { previousContext: '', nextContext: '' };
    }

    const range = selection.getRangeAt(0);
    const selectedElement = range.startContainer.parentElement;

    // Find the closest section or content area to search within
    let sectionElement = selectedElement?.closest('main, .container, .markdown, .docContent');
    if (!sectionElement) {
      sectionElement = document.querySelector('main, .container, .markdown, .docContent') || document.body;
    }

    // Find all heading elements
    const allHeadings = Array.from(sectionElement.querySelectorAll('h1, h2, h3, h4, h5, h6'));
    
    // Find the heading that comes before the selection
    let previousHeading: Element | null = null;
    let nextHeading: Element | null = null;
    
    for (let i = 0; i < allHeadings.length; i++) {
      const heading = allHeadings[i] as HTMLElement;
      
      if (heading.compareDocumentPosition(selectedElement) & Node.DOCUMENT_POSITION_PRECEDING) {
        // This heading comes before our selection
        previousHeading = heading;
      } else if (heading.compareDocumentPosition(selectedElement) & Node.DOCUMENT_POSITION_FOLLOWING) {
        // This is the first heading after our selection
        nextHeading = heading;
        break;
      }
    }

    // Get text content of the contexts
    const previousContext = previousHeading ? previousHeading.textContent?.trim() || '' : '';
    const nextContext = nextHeading ? nextHeading.textContent?.trim() || '' : '';

    return { previousContext, nextContext };
  }, []);

  const sendHighlightToBackend = useCallback(async (selectedText: string, selectionContext?: string) => {
    onLoading(true);
    onError(null);
    onContent(null);

    try {
      // Get the API key from localStorage (as configured in ai-config.tsx)
      let apiKey = localStorage.getItem('ai_api_key');
      
      // If no API key in localStorage, use the default test key
      if (!apiKey) {
        apiKey = DEFAULT_API_KEY;
        console.log('Using default API key for testing');
      }

      // Verify the API key appears to be valid (basic check)
      if (apiKey.length < 30 || !apiKey.startsWith('AIzaSy')) {
        throw new Error('Invalid API key format. Please check your API key in the AI Config page.');
      }

      // Get the heading context
      const { previousContext, nextContext } = findHeadingContext();

      // Create the enhanced query with context
      let queryWithContext = `Explain this content: ${selectedText}`;
      
      if (previousContext || nextContext) {
        queryWithContext = `Context: The selected text "${selectedText}" is within the following sections:\n`;
        if (previousContext) {
          queryWithContext += `- Previous topic: ${previousContext}\n`;
        }
        if (nextContext) {
          queryWithContext += `- Next topic: ${nextContext}\n`;
        }
        queryWithContext += `\nExplain the selected text and how it relates to these surrounding topics: ${selectedText}`;
      }

      // Create an AbortController for timeout handling
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 30000); // 30 second timeout

      let response;
      try {
        response = await fetch(`${API_BASE_URL}/api/ai/query`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            highlighted_text: selectedText,
            query: queryWithContext,
            previous_heading_context: previousContext,
            next_heading_context: nextContext,
            api_key: apiKey  // Include the API key in the request
          }),
          signal: controller.signal  // Add the abort signal
        });
      } catch (fetchError: any) {
        clearTimeout(timeoutId);
        if (fetchError.name === 'AbortError') {
          throw new Error('Request timed out. The AI is taking too long to respond.');
        } else {
          console.error('Network error when fetching from backend:', fetchError);
          throw new Error(`Network error: ${fetchError.message || 'Unable to connect to the AI service'}`);
        }
      }

      clearTimeout(timeoutId); // Clear the timeout if the request completes

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Failed to fetch AI content');
      }

      const data = await response.json();
      
      // Parse the structured response
      const responseText = data.response;
      
      // Extract explanation, implementation, and examples from the response using flexible matching
      let explanation = "No explanation provided.";
      let implementation = "No implementation required.";
      let examples = "No examples available.";
      
      // Try to find sections with different possible formats
      const lines = responseText.split('\n');
      let currentSection = '';
      let explanationText = '';
      let implementationText = '';
      let examplesText = '';
      
      // First try regex patterns for numbered sections
      const explanationMatch = responseText.match(/1\.\s*(?:Explanation:?)\s*([\s\S]*?)(?=2\.|2\s+Implementation|$)/i);
      const implementationMatch = responseText.match(/2\.\s*(?:Implementation:?)\s*([\s\S]*?)(?=3\.|3\s+Examples?|$)/i);
      const examplesMatch = responseText.match(/3\.\s*(?:Examples?:?)\s*([\s\S]*)/i);
      
      if (explanationMatch) {
        explanation = explanationMatch[1].trim();
      } else {
        // Look for keywords in the text
        const expMatch = responseText.match(/(?:^|\n)Explanation:\s*([\s\S]*?)(?=\n[A-Z]|\n\d\.|$)/i);
        if (expMatch) {
          explanation = expMatch[1].trim();
        } else {
          // If no clear section found, use the beginning of the response as explanation
          // But make sure it's not just the highlighted text
          const firstSection = responseText.substring(0, Math.min(responseText.indexOf('\n\n') !== -1 ? responseText.indexOf('\n\n') : responseText.length, 500)).trim();
          
          if (firstSection.toLowerCase().includes(selectedText.toLowerCase()) && firstSection.length <= selectedText.length + 50) {
            // The first section is nearly identical to the selected text, so look for a more detailed part
            // Try to find a more substantial explanation in the response
            const paragraphs = responseText.split('\n\n');
            for (const para of paragraphs) {
              if (para.trim().length > 100 && !para.toLowerCase().includes(selectedText.toLowerCase())) {
                explanation = para.trim();
                break;
              }
            }
            if (explanation === "No explanation provided.") {
              // If we still don't have a good explanation, take the first substantial paragraph
              explanation = paragraphs[0]?.trim() || responseText.substring(0, 300).trim();
            }
          } else {
            explanation = firstSection;
          }
        }
      }
      
      if (implementationMatch) {
        implementation = implementationMatch[1].trim();
      } else {
        // Look for implementation keywords
        const impMatch = responseText.match(/(?:^|\n)Implementation:\s*([\s\S]*?)(?=\n[A-Z]|\n\d\.|$)/i);
        const noImplMatch = responseText.match(/(?:no|No) (?:implementation|required)/i);
        if (impMatch) {
          implementation = impMatch[1].trim();
        } else if (noImplMatch) {
          implementation = "No implementation required.";
        }
      }
      
      if (examplesMatch) {
        examples = examplesMatch[1].trim();
      } else {
        // Look for examples keywords
        const exMatch = responseText.match(/(?:^|\n)Examples?:\s*([\s\S]*)(?=\n[A-Z]|\n\d\.|$)/i);
        const noExMatch = responseText.match(/(?:no|No) (?:examples?|available)/i);
        if (exMatch) {
          examples = exMatch[1].trim();
        } else if (noExMatch) {
          examples = "No examples available.";
        }
      }
      
      // Final check: if explanation is still just the highlighted text or very short, try to get more content
      if (explanation === "No explanation provided." || 
          explanation.toLowerCase().includes(selectedText.toLowerCase()) && explanation.length <= selectedText.length + 50 ||
          explanation.length < 30) {
        // Extract a more comprehensive part of the response
        const paragraphs = responseText.split('\n\n').filter(p => p.trim().length > 0);
        for (const para of paragraphs) {
          if (para.trim().length > selectedText.length && !para.toLowerCase().includes(selectedText.toLowerCase())) {
            explanation = para.trim();
            break;
          }
        }
        // If still not found, take the largest paragraph that's not just the selected text
        if (explanation === "No explanation provided." || 
            explanation.toLowerCase().includes(selectedText.toLowerCase()) && explanation.length <= selectedText.length + 50) {
          let bestPara = "";
          for (const para of paragraphs) {
            if (para.length > bestPara.length && !para.toLowerCase().includes(selectedText.toLowerCase())) {
              bestPara = para;
            }
          }
          explanation = bestPara || responseText.substring(0, 300).trim();
        }
      }
      
      // Format response to match the expected structure in AIDialog
      const formattedData = {
        contextOfSelection: selectedText,
        basicTheory: explanation,
        instructions: implementation,
        example: examples,
        previousContext: previousContext,
        nextContext: nextContext
      };
      
      onContent(formattedData);
    } catch (err: any) {
      console.error('Error in sendHighlightToBackend:', err);
      onError(err.message);
    } finally {
      onLoading(false);
    }
  }, [onLoading, onError, onContent, findHeadingContext]);

  // Function to proceed immediately with the request (bypassing popup)
  const proceedWithRequest = useCallback((text: string, context?: string) => {
    onHighlight(text);
    sendHighlightToBackend(text, context);
  }, [onHighlight, sendHighlightToBackend]);

  const showConfirmationPopup = useCallback((text: string, context?: string) => {
    // Get cursor position to show the popup near the selection
    const selection = window.getSelection();
    if (selection && selection.rangeCount > 0) {
      const range = selection.getRangeAt(0);
      const rect = range.getBoundingClientRect();
      setConfirmationPosition({ x: rect.left, y: rect.bottom + 10 });
    } else {
      // Fallback to mouse position if selection rect is not available
      setConfirmationPosition({ x: window.innerWidth / 2 - 100, y: 100 });
    }

    setHighlightedText(text);
    setPendingRequest({ text, context });
    setShowConfirmation(true);
  }, []);

  const handleConfirmation = useCallback(() => {
    if (pendingRequest) {
      onHighlight(pendingRequest.text);
      sendHighlightToBackend(pendingRequest.text, pendingRequest.context);
    }
    setShowConfirmation(false);
    setPendingRequest(null);
  }, [pendingRequest, onHighlight, sendHighlightToBackend]);

  const handleCancellation = useCallback(() => {
    setShowConfirmation(false);
    setPendingRequest(null);
  }, []);

  const handleSelectionChange = useCallback(() => {
    const selection = window.getSelection();
    if (selection && selection.toString().length > 0) {
      const selectedText = selection.toString().trim();
      if (selectedText.length > 0) {
        showConfirmationPopup(selectedText);
      }
    } else {
      // Clear content if no selection
      onContent(null);
    }
  }, [showConfirmationPopup, onContent]);

  // Add global keydown listener for the shortcut key (Ctrl/Cmd + Shift + E)
  useEffect(() => {
    const handleGlobalKeyDown = (e: KeyboardEvent) => {
      // Check if the confirmation popup is visible
      if (showConfirmation) {
        // Enter to confirm
        if (e.key === 'Enter') {
          e.preventDefault();
          handleConfirmation();
        }
        // Escape to cancel
        if (e.key === 'Escape') {
          e.preventDefault();
          handleCancellation();
        }
      } 
      // Shortcut to trigger AI explanation if there's selected text (bypass popup)
      else if ((e.ctrlKey || e.metaKey) && e.shiftKey && e.key.toLowerCase() === 'e') {
        const selection = window.getSelection();
        if (selection && selection.toString().length > 0) {
          const selectedText = selection.toString().trim();
          if (selectedText.length > 0) {
            // Prevent default browser behavior for this shortcut
            e.preventDefault();
            // Proceed immediately without showing popup
            proceedWithRequest(selectedText);
          }
        }
      }
    };

    window.addEventListener('keydown', handleGlobalKeyDown);
    return () => {
      window.removeEventListener('keydown', handleGlobalKeyDown);
    };
  }, [showConfirmation, handleConfirmation, handleCancellation, proceedWithRequest]);

  useEffect(() => {
    document.addEventListener('mouseup', handleSelectionChange);
    return () => {
      document.removeEventListener('mouseup', handleSelectionChange);
    };
  }, [handleSelectionChange]);

  return (
    <>
      <HighlightConfirmation
        highlightedText={highlightedText}
        onConfirm={handleConfirmation}
        onCancel={handleCancellation}
        position={confirmationPosition}
        isVisible={showConfirmation}
      />
    </>
  );
};

export default HighlightDetector;