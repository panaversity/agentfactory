import React, { useState, useEffect } from 'react';
import AIService, { ConfigStatusResponse } from '../services/ai_service';

interface ConfigUIProps {
  onClose: () => void;
}

const ConfigUI: React.FC<ConfigUIProps> = ({ onClose }) => {
  const [apiKey, setApiKey] = useState<string>('');
  const [status, setStatus] = useState<ConfigStatusResponse | null>(null);
  const [loading, setLoading] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);

  // Load current configuration status on component mount
  useEffect(() => {
    loadConfigStatus();
  }, []);

  const loadConfigStatus = async () => {
    try {
      setLoading(true);
      setError(null);
      const configStatus = await AIService.getConfigStatus();
      setStatus(configStatus);
    } catch (err) {
      setError('Failed to load configuration status');
      console.error('Error loading config status:', err);
    } finally {
      setLoading(false);
    }
  };

  const handleSaveApiKey = async () => {
    if (!apiKey.trim()) {
      setError('API key is required');
      return;
    }

    try {
      setLoading(true);
      setError(null);
      setSuccess(null);

      // Configure the API key
      await AIService.configureAPIKey({ api_key: apiKey });
      
      // Load the new status to confirm it was saved
      const newStatus = await AIService.getConfigStatus();
      setStatus(newStatus);
      setSuccess('API key saved and validated successfully!');
    } catch (err) {
      setError((err as Error).message || 'Failed to save API key');
      console.error('Error saving API key:', err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="config-ui-overlay">
      <div className="config-ui">
        <div className="config-header">
          <h2>AI Configuration</h2>
          <button className="close-button" onClick={onClose}>&times;</button>
        </div>
        
        <div className="config-content">
          <div className="config-section">
            <label htmlFor="api-key">Gemini API Key:</label>
            <input
              id="api-key"
              type="password"
              value={status?.is_configured && !apiKey ? '••••••••' : apiKey}
              onChange={(e) => setApiKey(e.target.value)}
              placeholder="Enter your Gemini API key"
              disabled={loading}
            />
            <button 
              onClick={handleSaveApiKey} 
              disabled={loading}
              className="save-button"
            >
              {loading ? 'Saving...' : 'Save API Key'}
            </button>
            
            {success && <div className="success-message">{success}</div>}
            {error && <div className="error-message">{error}</div>}
          </div>
          
          <div className="config-section">
            <h3>Configuration Status</h3>
            {status ? (
              <div className="status-info">
                <p><strong>API Key Configured:</strong> {status.is_configured ? 'Yes' : 'No'}</p>
                {status.is_configured && (
                  <p>
                    <strong>API Key Valid:</strong> {status.is_valid ? 'Yes' : 'No'}
                    {status.message && <span> ({status.message})</span>}
                  </p>
                )}
                <p><strong>Model:</strong> gemini-2.5-flash</p>
              </div>
            ) : (
              <p>Loading configuration status...</p>
            )}
          </div>
          
          <div className="config-section">
            <h3>Instructions</h3>
            <ol>
              <li>Get your Gemini API key from the <a href="https://aistudio.google.com/" target="_blank" rel="noopener noreferrer">Google AI Studio</a></li>
              <li>Enter your API key in the field above</li>
              <li>Click "Save API Key" to configure the application</li>
              <li>Once saved, you can use the AI dialog by highlighting text in the e-book</li>
            </ol>
          </div>
        </div>
        
        <div className="config-actions">
          <button onClick={onClose} className="cancel-button">
            Close
          </button>
        </div>
      </div>
      
      <style jsx>{`
        .config-ui-overlay {
          position: fixed;
          top: 0;
          left: 0;
          right: 0;
          bottom: 0;
          background-color: rgba(0, 0, 0, 0.5);
          display: flex;
          justify-content: center;
          align-items: center;
          z-index: 1000;
        }
        
        .config-ui {
          background: white;
          border-radius: 8px;
          width: 90%;
          max-width: 600px;
          max-height: 90vh;
          overflow-y: auto;
        }
        
        .config-header {
          display: flex;
          justify-content: space-between;
          align-items: center;
          padding: 1rem;
          border-bottom: 1px solid #eee;
        }
        
        .config-header h2 {
          margin: 0;
        }
        
        .close-button {
          background: none;
          border: none;
          font-size: 1.5rem;
          cursor: pointer;
          padding: 0.25rem;
        }
        
        .config-content {
          padding: 1rem;
        }
        
        .config-section {
          margin-bottom: 1.5rem;
        }
        
        .config-section label {
          display: block;
          margin-bottom: 0.5rem;
          font-weight: bold;
        }
        
        .config-section input {
          width: 100%;
          padding: 0.5rem;
          border: 1px solid #ccc;
          border-radius: 4px;
          margin-bottom: 0.5rem;
        }
        
        .save-button, .cancel-button {
          background-color: #007bff;
          color: white;
          border: none;
          padding: 0.5rem 1rem;
          border-radius: 4px;
          cursor: pointer;
        }
        
        .save-button:disabled, .cancel-button:disabled {
          background-color: #6c757d;
          cursor: not-allowed;
        }
        
        .success-message {
          color: #28a745;
          margin-top: 0.5rem;
        }
        
        .error-message {
          color: #dc3545;
          margin-top: 0.5rem;
        }
        
        .status-info p {
          margin: 0.25rem 0;
        }
        
        .config-section ol {
          padding-left: 1.5rem;
          margin: 0.5rem 0;
        }
        
        .config-actions {
          padding: 1rem;
          border-top: 1px solid #eee;
          text-align: right;
        }
      `}</style>
    </div>
  );
};

export default ConfigUI;