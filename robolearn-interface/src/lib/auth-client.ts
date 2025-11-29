import { createAuthClient } from "better-auth/react";

// Auth server URL - defaults to localhost for development
const AUTH_URL = typeof window !== 'undefined'
  ? (window as unknown as { __DOCUSAURUS__?: { siteConfig?: { customFields?: { authUrl?: string } } } }).__DOCUSAURUS__?.siteConfig?.customFields?.authUrl || "http://localhost:3001"
  : "http://localhost:3001";

// OAuth client configuration for robolearn-interface (public client with PKCE)
const OAUTH_CLIENT_ID = "robolearn-interface";
const OAUTH_REDIRECT_URI = typeof window !== 'undefined'
  ? `${window.location.origin}/auth/callback`
  : "http://localhost:3000/auth/callback";

export const authClient = createAuthClient({
  baseURL: AUTH_URL,
});

export const { signIn, signUp, signOut, useSession } = authClient;

// PKCE Helper Functions
// Generate a cryptographically random code verifier
function generateCodeVerifier(): string {
  const array = new Uint8Array(32);
  crypto.getRandomValues(array);
  return base64UrlEncode(array);
}

// Generate code challenge from verifier using SHA-256
async function generateCodeChallenge(verifier: string): Promise<string> {
  const encoder = new TextEncoder();
  const data = encoder.encode(verifier);
  const hash = await crypto.subtle.digest('SHA-256', data);
  return base64UrlEncode(new Uint8Array(hash));
}

// Base64 URL encode (RFC 4648)
function base64UrlEncode(buffer: Uint8Array): string {
  let binary = '';
  for (let i = 0; i < buffer.length; i++) {
    binary += String.fromCharCode(buffer[i]);
  }
  return btoa(binary)
    .replace(/\+/g, '-')
    .replace(/\//g, '_')
    .replace(/=+$/, '');
}

// OAuth2 Authorization URL builder with PKCE
export async function getOAuthAuthorizationUrl(state?: string): Promise<string> {
  // Generate PKCE code verifier and challenge
  const codeVerifier = generateCodeVerifier();
  const codeChallenge = await generateCodeChallenge(codeVerifier);

  // Store verifier in localStorage for token exchange
  // Note: Using localStorage instead of sessionStorage because sessionStorage
  // doesn't persist across full page navigations to different origins (auth server)
  if (typeof window !== 'undefined') {
    localStorage.setItem('pkce_code_verifier', codeVerifier);
  }

  const params = new URLSearchParams({
    client_id: OAUTH_CLIENT_ID,
    redirect_uri: OAUTH_REDIRECT_URI,
    response_type: "code",
    scope: "openid profile email",
    state: state || Math.random().toString(36).substring(7),
    code_challenge: codeChallenge,
    code_challenge_method: "S256",
  });

  return `${AUTH_URL}/api/auth/oauth2/authorize?${params.toString()}`;
}

// Token refresh function
export async function refreshAccessToken(): Promise<string | null> {
  const refreshToken = localStorage.getItem('robolearn_refresh_token');
  if (!refreshToken) return null;

  try {
    const response = await fetch(`${AUTH_URL}/api/auth/oauth2/token`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: new URLSearchParams({
        grant_type: 'refresh_token',
        refresh_token: refreshToken,
        client_id: OAUTH_CLIENT_ID,
      }),
    });

    if (response.ok) {
      const tokens = await response.json();
      localStorage.setItem('robolearn_access_token', tokens.access_token);
      if (tokens.refresh_token) {
        localStorage.setItem('robolearn_refresh_token', tokens.refresh_token);
      }
      return tokens.access_token;
    }
  } catch (error) {
    console.error('Token refresh failed:', error);
  }
  return null;
}

// Export OAuth config for use in components
export const oauthConfig = {
  clientId: OAUTH_CLIENT_ID,
  redirectUri: OAUTH_REDIRECT_URI,
  authUrl: AUTH_URL,
};
