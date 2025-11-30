import { createAuthClient } from "better-auth/react";

// Default fallbacks for development
const DEFAULT_AUTH_URL = "http://localhost:3001";
const DEFAULT_CLIENT_ID = "robolearn-public-client"; // Matches trustedClients[0].clientId in auth-server

// Create auth client with default URL (will be overridden in components using useDocusaurusContext)
export const authClient = createAuthClient({
  baseURL: DEFAULT_AUTH_URL,
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

// OAuth config type for components to pass in
export interface OAuthConfig {
  authUrl?: string;
  clientId?: string;
  redirectUri?: string;
}

// OAuth2 Authorization URL builder with PKCE
// Config should be passed from component using useDocusaurusContext
export async function getOAuthAuthorizationUrl(state?: string, config?: OAuthConfig): Promise<string> {
  const authUrl = config?.authUrl || DEFAULT_AUTH_URL;
  const clientId = config?.clientId || DEFAULT_CLIENT_ID;
  const redirectUri = config?.redirectUri || (typeof window !== 'undefined'
    ? `${window.location.origin}/auth/callback`
    : "http://localhost:3000/auth/callback");

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
    client_id: clientId,
    redirect_uri: redirectUri,
    response_type: "code",
    scope: "openid profile email",
    state: state || Math.random().toString(36).substring(7),
    code_challenge: codeChallenge,
    code_challenge_method: "S256",
  });

  return `${authUrl}/api/auth/oauth2/authorize?${params.toString()}`;
}

// Token refresh function
export async function refreshAccessToken(config?: OAuthConfig): Promise<string | null> {
  const refreshToken = localStorage.getItem('robolearn_refresh_token');
  if (!refreshToken) return null;

  const authUrl = config?.authUrl || DEFAULT_AUTH_URL;
  const clientId = config?.clientId || DEFAULT_CLIENT_ID;

  try {
    const response = await fetch(`${authUrl}/api/auth/oauth2/token`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: new URLSearchParams({
        grant_type: 'refresh_token',
        refresh_token: refreshToken,
        client_id: clientId,
      }),
    });

    if (response.ok) {
      const tokens = await response.json();
      localStorage.setItem('robolearn_access_token', tokens.access_token);
      if (tokens.refresh_token) {
        localStorage.setItem('robolearn_refresh_token', tokens.refresh_token);
      }
      // Store new ID token if provided (for JWKS verification)
      if (tokens.id_token) {
        localStorage.setItem('robolearn_id_token', tokens.id_token);
      }
      return tokens.access_token;
    }
  } catch (error) {
    console.error('Token refresh failed:', error);
  }
  return null;
}
