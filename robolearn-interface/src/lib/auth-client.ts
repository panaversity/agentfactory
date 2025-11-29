import { createAuthClient } from "better-auth/react";

// Auth server URL - defaults to localhost for development
const AUTH_URL = typeof window !== 'undefined'
  ? (window as unknown as { __DOCUSAURUS__?: { siteConfig?: { customFields?: { authUrl?: string } } } }).__DOCUSAURUS__?.siteConfig?.customFields?.authUrl || "http://localhost:3001"
  : "http://localhost:3001";

// OAuth client configuration for robolearn-interface
const OAUTH_CLIENT_ID = "robolearn-interface";
const OAUTH_REDIRECT_URI = typeof window !== 'undefined'
  ? `${window.location.origin}/auth/callback`
  : "http://localhost:3000/auth/callback";

export const authClient = createAuthClient({
  baseURL: AUTH_URL,
});

export const { signIn, signUp, signOut, useSession } = authClient;

// OAuth2 Authorization URL builder
export function getOAuthAuthorizationUrl(state?: string): string {
  const params = new URLSearchParams({
    client_id: OAUTH_CLIENT_ID,
    redirect_uri: OAUTH_REDIRECT_URI,
    response_type: "code",
    scope: "openid profile email",
    state: state || Math.random().toString(36).substring(7),
  });

  return `${AUTH_URL}/api/auth/oauth2/authorize?${params.toString()}`;
}

// Export OAuth config for use in components
export const oauthConfig = {
  clientId: OAUTH_CLIENT_ID,
  redirectUri: OAUTH_REDIRECT_URI,
  authUrl: AUTH_URL,
};
