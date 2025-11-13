import React, { useState, useEffect, useRef } from 'react';
import AIService from '../services/ai_service';
import { TextSelection } from '../services/text_selection_service';

interface AIDialogProps {
  initialText?: string;
  onClose: () => void;
  position?: { x: number; y: number };
}

const AIDialog: React.FC<AIDialogProps> = ({ initialText = '', onClose, position = { x: 0, y: 0 } }) => {
  const [query, setQuery] = useState<string>(initialText);
  const [response, setResponse] = useState<string>('');
  const [loading, setLoading] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);
  const [isConfigured, setIsConfigured] = useState<boolean>(false);
  const dialogRef = useRef<HTMLDivElement>(null);

  // Check if API is configured on component mount
  useEffect(() => {
    checkConfigStatus();
    setQuery(initialText);
  }, [initialText]);

  // Close dialog when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (dialogRef.current && !dialogRef.current.contains(event.target as Node)) {
        onClose();
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [onClose]);

  const checkConfigStatus = async () => {
    try {
      const status = await AIService.getConfigStatus();
      setIsConfigured(status.is_configured && status.is_valid === true);
    } catch (err) {
      console.error('Error checking config status:', err);
      setIsConfigured(false);
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    
    if (!query.trim()) {
      setError('Please enter a query');
      return;
    }

    if (!isConfigured) {
      setError('AI is not configured. Please set up your API key first.');
      return;
    }

    try {
      setLoading(true);
      setError(null);
      setResponse('');

      const result = await AIService.queryAI({ 
        highlighted_text: initialText, 
        query 
      });
      
      setResponse(result.response);
    } catch (err) {
      setError((err as Error).message || 'Error getting AI response');
      console.error('Error querying AI:', err);
    } finally {
      setLoading(false);
    }
  };

  // Calculate position to keep dialog within viewport
  const dialogStyle: React.CSSProperties = {
    position: 'fixed',
    left: Math.min(position.x, window.innerWidth - 400), // 400px is approximate dialog width
    top: Math.min(position.y, window.innerHeight - 300), // 300px is approximate dialog height
    zIndex: 1000,
  };

  return (
    <div className="ai-dialog-overlay" style={dialogStyle} ref={dialogRef}>
      <div className="ai-dialog">
        <div className="ai-dialog-header">
          <h3>AI Assistant</h3>
          <button className="close-button" onClick={onClose}>&times;</button>
        </div>
        
        <div className="ai-dialog-content">
          {initialText && (
            <div className="highlighted-text-preview">
              <strong>Context:</strong> "{initialText.substring(0, 100)}{initialText.length > 100 ? '...' : ''}"
            </div>
          )}
          
          <form onSubmit={handleSubmit} className="ai-query-form">
            <textarea
              value={query}
              onChange={(e) => setQuery(e.target.value)}
              placeholder="Ask the AI about the highlighted text..."
              disabled={loading}
              className="query-input"
            />
            <button 
              type="submit" 
              disabled={loading || !isConfigured}
              className="submit-button"
            >
              {loading ? 'Asking AI...' : 'Ask AI'}
            </button>
          </form>
          
          {error && <div className="error-message">{error}</div>}
          
          {response && (
            <div className="ai-response">
              <h4>AI Response:</h4>
              <div className="response-content">{response}</div>
            </div>
          )}
          
          {!isConfigured && (
            <div className="config-warning">
              <p>⚠️ AI is not configured. Please set up your API key first.</p>
            </div>
          )}
        </div>
      </div>
      
      <style jsx>{`
        .ai-dialog-overlay {
          position: fixed;
          z-index: 1000;
        }
        
        .ai-dialog {
          background: white;
          border-radius: 8px;
          box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
          width: 400px;
          max-height: 500px;
          display: flex;
          flex-direction: column;
        }
        
        .ai-dialog-header {
          display: flex;
          justify-content: space-between;
          align-items: center;
          padding: 0.75rem;
          border-bottom: 1px solid #eee;
        }
        
        .ai-dialog-header h3 {
          margin: 0;
          font-size: 1.1rem;
        }
        
        .close-button {
          background: none;
          border: none;
          font-size: 1.5rem;
          cursor: pointer;
          padding: 0.25rem;
        }
        
        .ai-dialog-content {
          padding: 1rem;
          flex: 1;
          overflow-y: auto;
        }
        
        .highlighted-text-preview {
          background-color: #f8f9fa;
          padding: 0.5rem;
          border-radius: 4px;
          margin-bottom: 0.75rem;
          font-size: 0.9rem;
          color: #555;
        }
        
        .ai-query-form {
          margin-bottom: 1rem;
        }
        
        .query-input {
          width: 100%;
          padding: 0.5rem;
          border: 1px solid #ccc;
          border-radius: 4px;
          resize: vertical;
          min-height: 80px;
          margin-bottom: 0.5rem;
        }
        
        .submit-button {
          background-color: #007bff;
          color: white;
          border: none;
          padding: 0.5rem 1rem;
          border-radius: 4px;
          cursor: pointer;
        }
        
        .submit-button:disabled {
          background-color: #6c757d;
          cursor: not-allowed;
        }
        
        .error-message {
          color: #dc3545;
          margin: 0.5rem 0;
          padding: 0.5rem;
          background-color: #f8d7da;
          border-radius: 4px;
        }
        
        .ai-response {
          border-top: 1px solid #eee;
          padding-top: 1rem;
        }
        
        .ai-response h4 {
          margin-top: 0;
          color: #007bff;
        }
        
        .response-content {
          background-color: #f8f9fa;
          padding: 0.75rem;
          border-radius: 4px;
          white-space: pre-wrap;
          max-height: 150px;
          overflow-y: auto;
        }
        
        .config-warning {
          background-color: #fff3cd;
          border: 1px solid #ffeaa7;
          color: #856404;
          padding: 0.75rem;
          border-radius: 4px;
          margin-top: 1rem;
        }
      `}</style>
    </div>
  );
};

export default AIDialog;