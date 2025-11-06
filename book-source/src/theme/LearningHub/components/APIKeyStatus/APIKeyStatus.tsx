/**
 * APIKeyStatus Component
 * Shows setup instructions if Gemini API key is missing
 */

import React, { useState, useEffect } from 'react';
import styles from './APIKeyStatus.module.css';

export function APIKeyStatus() {
  const [hasApiKey, setHasApiKey] = useState(true);
  const [isChecking, setIsChecking] = useState(true);

  useEffect(() => {
    // Check if API key is configured
    const checkApiKey = () => {
      try {
        // Try to access the API key from environment or config
        const apiKey = typeof process !== 'undefined' && process.env?.GEMINI_API_KEY;
        
        // Also check localStorage for user-provided key
        const userKey = typeof localStorage !== 'undefined' && localStorage.getItem('gemini_api_key');
        
        setHasApiKey(!!(apiKey || userKey));
      } catch (error) {
        console.error('[APIKeyStatus] Error checking API key:', error);
        setHasApiKey(false);
      } finally {
        setIsChecking(false);
      }
    };

    checkApiKey();
  }, []);

  // Don't show anything if API key exists
  if (isChecking || hasApiKey) {
    return null;
  }

  return (
    <div className={styles.apiKeyStatus}>
      <div className={styles.statusIcon}>🔑</div>
      <div className={styles.statusContent}>
        <h3 className={styles.statusTitle}>AI Features Require Setup</h3>
        <p className={styles.statusDescription}>
          To use AI-powered features (Chat, Quiz, Concepts), you need to configure a Google Gemini API key.
        </p>
        <ol className={styles.setupSteps}>
          <li>Get a free API key from <a href="https://makersuite.google.com/app/apikey" target="_blank" rel="noopener noreferrer">Google AI Studio</a></li>
          <li>Add it to your <code>.env</code> file as <code>GEMINI_API_KEY</code></li>
          <li>Restart the development server</li>
        </ol>
        <details className={styles.advancedSetup}>
          <summary>Advanced: Use your own API key temporarily</summary>
          <p className={styles.warningText}>
            ⚠️ This stores your key in browser localStorage. Only use this for testing.
          </p>
          <input 
            type="password" 
            className={styles.apiKeyInput}
            placeholder="Enter your Gemini API key"
            onBlur={(e) => {
              if (e.target.value) {
                localStorage.setItem('gemini_api_key', e.target.value);
                setHasApiKey(true);
              }
            }}
          />
        </details>
      </div>
    </div>
  );
}
