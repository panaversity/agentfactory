# Better Auth OAuth/OIDC Setup Skill

## Purpose
Guide implementation of OAuth 2.1 / OIDC authentication using Better Auth with the OIDC Provider plugin.

## When to Use
- Setting up centralized authentication for multiple apps
- Implementing SSO (Single Sign-On) across a platform
- Creating an OAuth authorization server
- Integrating Better Auth as an identity provider

## Key Questions to Ask

1. **Architecture**
   - How many apps will use this auth server?
   - Is this for first-party apps only or third-party OAuth clients too?
   - Do you need dynamic client registration?

2. **Database**
   - Which database? (Postgres recommended with Neon for serverless)
   - Need user profiles beyond core auth fields?

3. **Features**
   - Role-based access control needed?
   - Admin dashboard for user management?
   - Consent screen for third-party apps?

## Implementation Checklist

### 1. Auth Server Setup (Public Client with PKCE)

```typescript
// src/lib/auth.ts
import { betterAuth } from "better-auth";
import { drizzleAdapter } from "better-auth/adapters/drizzle";
import { oidcProvider } from "better-auth/plugins/oidc-provider";
import { admin } from "better-auth/plugins/admin";

export const auth = betterAuth({
  database: drizzleAdapter(db, { provider: "pg", schema }),

  emailAndPassword: { enabled: true },

  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24,     // Refresh daily
  },

  trustedOrigins: process.env.ALLOWED_ORIGINS?.split(","),

  plugins: [
    oidcProvider({
      loginPage: "/auth/sign-in",
      consentPage: "/auth/consent",
      trustedClients: [{
        clientId: "your-app",
        // No clientSecret for public clients - use PKCE instead
        type: "public",  // Public client for SPAs
        redirectUrls: ["http://localhost:3000/auth/callback"],  // Note: lowercase 'urls'
        skipConsent: true,  // First-party apps
      }],
      // Add custom claims to userinfo
      async getAdditionalUserInfoClaim(user) {
        return { role: user.role };
      },
    }),
    admin({
      defaultRole: "user",
      adminRoles: ["admin"],
    }),
  ],
});
```

### 2. OAuth Client with PKCE (Recommended for SPAs)

```typescript
// Client app: src/lib/auth-client.ts

// PKCE helpers
function generateCodeVerifier(): string {
  const array = new Uint8Array(32);
  crypto.getRandomValues(array);
  return base64UrlEncode(array);
}

async function generateCodeChallenge(verifier: string): Promise<string> {
  const hash = await crypto.subtle.digest('SHA-256', new TextEncoder().encode(verifier));
  return base64UrlEncode(new Uint8Array(hash));
}

// Authorization URL with PKCE
export async function getOAuthAuthorizationUrl(state: string) {
  const codeVerifier = generateCodeVerifier();
  const codeChallenge = await generateCodeChallenge(codeVerifier);

  // Store verifier for token exchange
  sessionStorage.setItem('pkce_code_verifier', codeVerifier);

  const params = new URLSearchParams({
    client_id: 'your-app',
    redirect_uri: 'http://localhost:3000/auth/callback',
    response_type: 'code',
    scope: 'openid profile email',
    state,
    code_challenge: codeChallenge,
    code_challenge_method: 'S256',
  });
  return `${AUTH_SERVER_URL}/api/auth/oauth2/authorize?${params}`;
}

// Callback: exchange code for tokens with PKCE (no client_secret!)
const codeVerifier = sessionStorage.getItem('pkce_code_verifier');
const tokenResponse = await fetch(`${AUTH_SERVER_URL}/api/auth/oauth2/token`, {
  method: 'POST',
  headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
  body: new URLSearchParams({
    grant_type: 'authorization_code',
    code,
    redirect_uri: 'http://localhost:3000/auth/callback',
    client_id: 'your-app',
    code_verifier: codeVerifier,  // PKCE: verifier instead of secret
  }),
});
sessionStorage.removeItem('pkce_code_verifier');
```

### 3. Session Management (Client)

```typescript
// AuthContext.tsx pattern
const checkSession = async () => {
  const accessToken = localStorage.getItem('access_token');
  if (accessToken) {
    const response = await fetch(`${AUTH_URL}/api/auth/oauth2/userinfo`, {
      headers: { Authorization: `Bearer ${accessToken}` },
    });
    if (response.ok) {
      setSession({ user: await response.json() });
    } else {
      localStorage.removeItem('access_token');
    }
  }
};

const signOut = () => {
  localStorage.removeItem('access_token');
  localStorage.removeItem('refresh_token');
  setSession(null);
  window.location.href = '/';
};
```

## Common Pitfalls

### 1. PKCE Parameters Lost During Sign-In Redirect

When the OAuth authorization endpoint redirects to a sign-in page, the sign-in form must preserve PKCE parameters and forward them after successful authentication:

```typescript
// In sign-in-form.tsx - MUST extract and preserve PKCE params
const codeChallenge = searchParams.get("code_challenge");
const codeChallengeMethod = searchParams.get("code_challenge_method");

// After successful sign-in, rebuild OAuth URL WITH PKCE params
if (clientId && redirectUri && responseType) {
  const oauthParams = new URLSearchParams({
    client_id: clientId,
    redirect_uri: redirectUri,
    response_type: responseType,
    ...(scope && { scope }),
    ...(state && { state }),
    ...(codeChallenge && { code_challenge: codeChallenge }),  // CRITICAL!
    ...(codeChallengeMethod && { code_challenge_method: codeChallengeMethod }),
  });
  window.location.href = `/api/auth/oauth2/authorize?${oauthParams.toString()}`;
}
```

**Symptom**: "code verification failed" error on first login or after logout
**Cause**: Sign-in form drops PKCE parameters when rebuilding OAuth URL
**Fix**: Extract and include code_challenge and code_challenge_method in redirect

### 2. Wrong Property Name
```typescript
// WRONG - causes "Cannot read properties of undefined (reading 'find')"
redirectURLs: ["http://..."]

// CORRECT
redirectUrls: ["http://..."]
```

### 3. Cookie vs Token Auth Confusion
- OAuth clients should ONLY use tokens from localStorage
- Don't fall back to cookie-based session checking
- Cookie sessions are for the auth server itself

### 4. CORS Configuration
```typescript
// Auth server must trust client origins
trustedOrigins: ["http://localhost:3000", "https://your-app.com"]

// Environment variable
ALLOWED_ORIGINS=http://localhost:3000,https://your-app.com
```

### 5. Logout Scope
- OAuth standard: client clears its own tokens
- Auth server session stays active (SSO pattern)
- Don't try to clear auth server session from client

## Database Schema (Drizzle)

Required tables for OIDC Provider:
- `user` - Core user data
- `session` - Server sessions
- `account` - Auth provider accounts
- `oauth_application` - Registered OAuth clients
- `oauth_access_token` - Issued tokens
- `oauth_consent` - User consent records

## Testing Checklist

1. [ ] OIDC Discovery endpoint works: `GET /.well-known/openid-configuration`
2. [ ] Authorization redirects to login when unauthenticated
3. [ ] Authorization returns code after login
4. [ ] Token exchange returns access_token
5. [ ] UserInfo returns user data with valid token
6. [ ] Sign out clears tokens and redirects

## Security Checklist

- [ ] HTTPS in production
- [ ] Strong BETTER_AUTH_SECRET (32+ chars)
- [ ] PKCE enabled for public clients (SPAs, mobile apps)
- [ ] No client secrets in browser code (use PKCE instead)
- [ ] Exact redirect URI matching
- [ ] Rate limiting enabled
- [ ] CORS properly configured via `trustedOrigins`
- [ ] Token refresh implemented for long sessions
- [ ] Global logout option for multi-app SSO
