import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { getRedirectUri, getHomeUrl } from '@/lib/url-utils';

// OAuth callback page - exchanges authorization code for tokens using PKCE
export default function OAuthCallback(): React.JSX.Element {
  const [status, setStatus] = useState<'loading' | 'success' | 'error'>('loading');
  const [error, setError] = useState<string | null>(null);
  const { siteConfig } = useDocusaurusContext();

  // Get OAuth config from Docusaurus context
  const authUrl = (siteConfig.customFields?.authUrl as string) || 'http://localhost:3001';
  const oauthClientId = (siteConfig.customFields?.oauthClientId as string) || 'ai-native-public-client';
  // Generate redirect URI - auto-detects base path from current URL
  const redirectUri = getRedirectUri();

  useEffect(() => {
    const handleCallback = async () => {
      const urlParams = new URLSearchParams(window.location.search);
      const code = urlParams.get('code');
      const errorParam = urlParams.get('error');
      const errorDescription = urlParams.get('error_description');

      // Check for OAuth errors
      if (errorParam) {
        setError(errorDescription || errorParam);
        setStatus('error');
        return;
      }

      // Check for authorization code
      if (!code) {
        setError('No authorization code received');
        setStatus('error');
        return;
      }

      // Get PKCE code verifier from localStorage
      // Note: Using localStorage because sessionStorage doesn't persist across
      // full page navigations to different origins (auth server redirect)
      const codeVerifier = localStorage.getItem('pkce_code_verifier');
      if (!codeVerifier) {
        setError('PKCE code verifier not found. Please try signing in again.');
        setStatus('error');
        return;
      }

      try {
        // Exchange code for tokens using PKCE (no client_secret needed)
        const response = await fetch(`${authUrl}/api/auth/oauth2/token`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/x-www-form-urlencoded',
          },
          body: new URLSearchParams({
            grant_type: 'authorization_code',
            code,
            redirect_uri: redirectUri,
            client_id: oauthClientId,
            code_verifier: codeVerifier, // PKCE: use code_verifier instead of client_secret
          }),
          credentials: 'include',
        });

        // Clear the code verifier after use
        localStorage.removeItem('pkce_code_verifier');

        if (!response.ok) {
          const errorData = await response.json().catch(() => ({}));
          throw new Error(errorData.error_description || errorData.error || 'Token exchange failed');
        }

        const tokens = await response.json();

        // Store tokens in localStorage for the client
        if (tokens.access_token) {
          localStorage.setItem('ainative_access_token', tokens.access_token);
          if (tokens.refresh_token) {
            localStorage.setItem('ainative_refresh_token', tokens.refresh_token);
          }
          if (tokens.id_token) {
            localStorage.setItem('ainative_id_token', tokens.id_token);
          }
        }

        setStatus('success');

        // Redirect to home page after short delay (respects baseUrl)
        setTimeout(() => {
          window.location.href = getHomeUrl();
        }, 1500);

      } catch (err) {
        console.error('OAuth callback error:', err);
        setError(err instanceof Error ? err.message : 'Failed to complete authentication');
        setStatus('error');
      }
    };

    handleCallback();
  }, [authUrl, oauthClientId, redirectUri]);

  return (
    <Layout title="Authentication" description="Completing authentication...">
      <div style={{
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        minHeight: '60vh',
        padding: '2rem',
      }}>
        {status === 'loading' && (
          <>
            <div style={{
              width: '48px',
              height: '48px',
              border: '4px solid #e5e7eb',
              borderTopColor: '#001f3f',
              borderRadius: '50%',
              animation: 'spin 1s linear infinite',
            }} />
            <style>{`
              @keyframes spin {
                to { transform: rotate(360deg); }
              }
            `}</style>
            <h2 style={{ marginTop: '1.5rem', color: 'var(--ifm-font-color-base)' }}>
              Completing sign in...
            </h2>
            <p style={{ color: 'var(--ifm-font-color-secondary)' }}>
              Please wait while we authenticate you.
            </p>
          </>
        )}

        {status === 'success' && (
          <>
            <div style={{
              width: '64px',
              height: '64px',
              backgroundColor: '#10b981',
              borderRadius: '50%',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
            }}>
              <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="white" strokeWidth="3">
                <polyline points="20 6 9 17 4 12"></polyline>
              </svg>
            </div>
            <h2 style={{ marginTop: '1.5rem', color: 'var(--ifm-font-color-base)' }}>
              Successfully signed in!
            </h2>
            <p style={{ color: 'var(--ifm-font-color-secondary)' }}>
              Redirecting you back to AI Native Development...
            </p>
          </>
        )}

        {status === 'error' && (
          <>
            <div style={{
              width: '64px',
              height: '64px',
              backgroundColor: '#ef4444',
              borderRadius: '50%',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
            }}>
              <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="white" strokeWidth="3">
                <line x1="18" y1="6" x2="6" y2="18"></line>
                <line x1="6" y1="6" x2="18" y2="18"></line>
              </svg>
            </div>
            <h2 style={{ marginTop: '1.5rem', color: 'var(--ifm-font-color-base)' }}>
              Authentication failed
            </h2>
            <p style={{ color: '#ef4444', marginBottom: '1rem' }}>
              {error}
            </p>
            <a
              href={getHomeUrl()}
              style={{
                padding: '0.75rem 1.5rem',
                backgroundColor: '#001f3f',
                color: 'white',
                borderRadius: '0.5rem',
                textDecoration: 'none',
              }}
            >
              Go back to home
            </a>
          </>
        )}
      </div>
    </Layout>
  );
}
