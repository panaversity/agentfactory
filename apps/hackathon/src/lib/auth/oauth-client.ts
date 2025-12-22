import {
  OAuth2Client,
  generateCodeVerifier,
  generateState,
  CodeChallengeMethod,
} from "arctic";

// OAuth2 client for PKCE flow (public client - no client secret)
// Arctic requires: clientId, clientPassword (null for public), redirectURI
export const oauthClient = new OAuth2Client(
  process.env.SSO_OAUTH_CLIENT_ID!,
  null, // Public client - no secret
  process.env.SSO_OAUTH_REDIRECT_URI!
);

// Endpoints from environment
const authorizationEndpoint = process.env.SSO_OAUTH_AUTHORIZE_URL!;
const tokenEndpoint = process.env.SSO_OAUTH_TOKEN_URL!;

/**
 * Generate OAuth authorization URL with PKCE
 * @returns Object containing URL, state, and code verifier
 */
export function createAuthorizationUrl() {
  const state = generateState();
  const codeVerifier = generateCodeVerifier();
  const scopes = ["openid", "profile", "email"];

  // Create authorization URL with PKCE
  const url = oauthClient.createAuthorizationURLWithPKCE(
    authorizationEndpoint,
    state,
    CodeChallengeMethod.S256,
    codeVerifier,
    scopes
  );

  return {
    url: url.toString(),
    state,
    codeVerifier,
  };
}

/**
 * Exchange authorization code for tokens
 * @param code - Authorization code from callback
 * @param codeVerifier - PKCE code verifier from session
 * @returns Token response containing access_token, id_token, etc.
 */
export async function exchangeCodeForTokens(
  code: string,
  codeVerifier: string
) {
  const tokens = await oauthClient.validateAuthorizationCode(
    tokenEndpoint,
    code,
    codeVerifier
  );

  return tokens;
}
