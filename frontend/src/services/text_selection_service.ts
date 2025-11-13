/**
 * Service for handling text selection functionality in the application.
 */

export interface TextSelection {
  text: string;
  position: { 
    x: number; 
    y: number 
  };
}

class TextSelectionService {
  /**
   * Get the currently selected text along with its position.
   * @returns TextSelection object with the selected text and coordinates, or null if no text is selected
   */
  getSelectedText(): TextSelection | null {
    const selection = window.getSelection();
    if (!selection || selection.toString().trim() === '') {
      return null;
    }

    const selectedText = selection.toString().trim();
    if (!selectedText) {
      return null;
    }

    // Get the bounding rectangle of the selection to position the AI dialog
    const range = selection.getRangeAt(0);
    const rect = range.getBoundingClientRect();

    return {
      text: selectedText,
      position: {
        x: rect.left + window.scrollX,
        y: rect.top + window.scrollY + rect.height
      }
    };
  }

  /**
   * Clear the current text selection.
   */
  clearSelection(): void {
    if (window.getSelection) {
      window.getSelection()?.removeAllRanges();
    }
  }

  /**
   * Set up event listeners to detect text selection.
   * @param callback Function to call when text is selected
   */
  setupSelectionListener(callback: (selection: TextSelection | null) => void): void {
    document.addEventListener('mouseup', () => {
      const selection = this.getSelectedText();
      callback(selection);
    });

    // Also handle touch events for mobile devices
    document.addEventListener('touchend', () => {
      const selection = this.getSelectedText();
      callback(selection);
    });
  }
}

export default new TextSelectionService();