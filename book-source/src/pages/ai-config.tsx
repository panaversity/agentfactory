import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';

function AiConfig() {
  const [apiKey, setApiKey] = useState('');
  const [customPrompt, setCustomPrompt] = useState('');
  const [status, setStatus] = useState<{ is_configured: boolean; is_valid?: boolean; message?: string } | null>(null);
  const [loading, setLoading] = useState<boolean>(false);
  const [message, setMessage] = useState<string>('');
  const [error, setError] = useState<string>('');

  useEffect(() => {
    loadConfigStatus();
  }, []);

  const loadConfigStatus = async () => {
    try {
      setLoading(true);
      setError('');
      
      const response = await fetch('http://localhost:8000/api/config/status');
      const data = await response.json();
      setStatus(data);
      
      if (!data.is_configured || !data.is_valid) {
        setMessage('API key is not configured or is invalid');
      } else {
        setMessage('API key is configured and valid');
      }
    } catch (err) {
      setError('Failed to load configuration status');
    } finally {
      setLoading(false);
    }
  };

  const handleSave = async () => {
    if (!apiKey.trim()) {
      setError('API key is required');
      return;
    }

    try {
      setLoading(true);
      setError('');
      
      // Save API key to backend
      const response = await fetch('http://localhost:8000/api/config/gemini-key', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ api_key: apiKey }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Failed to save API key');
      }

      // Save custom prompt to localStorage as fallback
      localStorage.setItem('ai_custom_prompt', customPrompt);
      
      setMessage('AI Configuration Saved and Validated!');
      
      // Reload status
      loadConfigStatus();
    } catch (err: any) {
      setError(err.message || 'Failed to save API key');
    } finally {
      setLoading(false);
    }
  };

  return (
    <Layout title="AI Config" description="Configure AI settings for the e-book.">
      <main className="container margin-vert--lg">
        <h1>AI Configuration</h1>
        
        <div className="card">
          <div className="card__header">
            <h2>Configuration Status</h2>
          </div>
          <div className="card__body">
            {loading ? (
              <p>Loading configuration status...</p>
            ) : (
              <div>
                <p><strong>API Key Configured:</strong> {status?.is_configured ? 'Yes' : 'No'}</p>
                {status?.is_configured && (
                  <p><strong>API Key Valid:</strong> {status.is_valid ? 'Yes' : 'No'}</p>
                )}
                {status?.message && <p><strong>Status Message:</strong> {status.message}</p>}
                {message && <p style={{ color: 'green' }}>{message}</p>}
                {error && <p style={{ color: 'red' }}>{error}</p>}
              </div>
            )}
          </div>
        </div>

        <div className="card margin-top--md">
          <div className="card__header">
            <h2>API Key Setup</h2>
          </div>
          <div className="card__body">
            <p>Enter your Google Gemini API Key to enable AI features.</p>
            <input
              type="password"
              className="input input--lg"
              placeholder="Your Gemini API Key"
              value={apiKey}
              onChange={(e) => setApiKey(e.target.value)}
              style={{ width: '100%', padding: '10px', marginBottom: '15px' }}
            />
          </div>
        </div>

        <div className="card margin-top--md">
          <div className="card__header">
            <h2>Custom Prompt Configuration</h2>
          </div>
          <div className="card__body">
            <p>Define a custom prompt that will be used when you highlight text.</p>
            <textarea
              className="textarea"
              placeholder="e.g., 'Explain this concept in simple terms:'"
              value={customPrompt}
              onChange={(e) => setCustomPrompt(e.target.value)}
              rows={5}
              style={{ width: '100%', padding: '10px', marginBottom: '15px' }}
            />
          </div>
        </div>

        <button 
          className="button button--primary button--lg margin-top--md" 
          onClick={handleSave}
          disabled={loading}
        >
          {loading ? 'Saving...' : 'Save Configuration'}
        </button>
        
        <div className="card margin-top--md">
          <div className="card__header">
            <h2>Instructions</h2>
          </div>
          <div className="card__body">
            <ol>
              <li>Get your Gemini API key from the <a href="https://aistudio.google.com/" target="_blank" rel="noopener noreferrer">Google AI Studio</a></li>
              <li>Enter your API key in the field above</li>
              <li>Click "Save Configuration" to store and validate your API key</li>
              <li>Once saved, you can use the AI dialog by highlighting text in the e-book</li>
            </ol>
          </div>
        </div>
      </main>
    </Layout>
  );
}

export default AiConfig;
