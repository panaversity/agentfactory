# External App Integration Guide

Complete guide for integrating your application with the Panaversity SSO Server using OpenID Connect (OIDC).

---

## 📋 Table of Contents

1. [Overview](#overview)
2. [Quick Start](#quick-start)
3. [Public Clients (SPAs, Mobile Apps, Static Sites)](#public-clients-spas-mobile-apps-static-sites)
4. [Authentication Flow](#authentication-flow)
5. [OAuth2/OIDC Parameters Explained](#oauth2oidc-parameters-explained)
6. [Token Management](#token-management)
7. [Local Token Verification (JWKS)](#local-token-verification-jwks)
8. [Implementation Examples](#implementation-examples)
9. [Security Best Practices](#security-best-practices)
10. [Performance Optimization](#performance-optimization)
11. [Troubleshooting](#troubleshooting)
12. [API Reference](#api-reference)

---

## Overview

### What You'll Get

When a user successfully authenticates through our SSO server, your application receives:

```json
{
  "access_token": "eyJhbGci...",    // JWT - for API calls
  "id_token": "eyJhbGci...",        // JWT - user identity
  "refresh_token": "xP9mK3nR...",   // For getting new tokens
  "token_type": "Bearer",
  "expires_in": 3600,               // Token lifetime (seconds)
  "scope": "openid email profile"
}
```

### Key Concepts

| Token | Purpose | Format | Verify How? |
|-------|---------|--------|-------------|
| **Access Token** | API authorization | JWT | Locally via JWKS |
| **ID Token** | User identity | JWT | Locally via JWKS |
| **Refresh Token** | Get new tokens | Opaque string | Call SSO server |

### Architecture

```
┌─────────────────────┐
│   Your App          │
│                     │
│   Stores tokens in  │
│   own cookies       │
│                     │
│   Verifies tokens   │
│   LOCALLY (JWKS)    │──────┐
│                     │      │ Only for:
│   ❌ NO SSO calls   │      │ - Login
│      per request!   │      │ - Refresh
└─────────────────────┘      │ - Logout
                             │
                             ▼
                    ┌─────────────────┐
                    │   SSO Server    │
                    │                 │
                    │  localhost:3000 │
                    └─────────────────┘
```

---

## Quick Start

### 1. Register Your Application

Contact the SSO admin or use the dynamic registration endpoint:

```bash
curl -X POST http://localhost:3000/api/auth/oauth2/register \
  -H "Content-Type: application/json" \
  -d '{
    "client_name": "My Application",
    "redirect_uris": ["http://localhost:4000/auth/callback"]
  }'
```

**Response:**
```json
{
  "client_id": "RdwUTXGzXgIJCrySmMZTDioHkizYqUgp",
  "client_secret": "ItsFjRVHNnMfIMnrIaPaZZxebAOPtGXe",
  "redirect_uris": ["http://localhost:4000/auth/callback"]
}
```

**⚠️ Save these credentials securely!**

---

### 2. Install Dependencies

```bash
# For Next.js/Node.js applications
npm install jose

# jose provides JWT verification with JWKS support
```

---

### 3. Environment Variables

Create `.env.local`:

```env
# SSO Server
SSO_SERVER_URL=http://localhost:3000

# Your OIDC Client Credentials
OIDC_CLIENT_ID=RdwUTXGzXgIJCrySmMZTDioHkizYqUgp
OIDC_CLIENT_SECRET=ItsFjRVHNnMfIMnrIaPaZZxebAOPtGXe

# Your App
OIDC_REDIRECT_URI=http://localhost:4000/auth/callback
NEXT_PUBLIC_APP_URL=http://localhost:4000
```

---

## Public Clients (SPAs, Mobile Apps, Static Sites)

For applications that **cannot securely store a client secret** (Single Page Apps, Mobile Apps, Docusaurus, Static Site Generators), use the **Public Client** flow with **PKCE** (Proof Key for Code Exchange).

### What Are Public Clients?

Public clients are applications where the code runs in an **untrusted environment**:

| Client Type | Examples | Can Store Secret? | Security Method |
|-------------|----------|-------------------|-----------------|
| **Public** | React SPA, Vue.js, Docusaurus, Mobile Apps, CLI tools | ❌ NO | PKCE (mandatory) |
| **Confidential** | Next.js (server), Express.js, Django, .NET backend | ✅ YES | Client Secret |

---

### How Better-Auth Handles Public Clients

Better-Auth uses the **`type`** field in client registration to determine authentication method:

```typescript
// Database schema
{
  type: "public",        // ← Public client (no secret)
  clientId: "abc123",
  clientSecret: null,    // ← NULL for public clients
  // ...
}
```

**Server-side validation logic:**
```typescript
if (client.type === "public") {
  // PKCE is MANDATORY
  if (!code_verifier) {
    throw Error("code verifier is required for public clients");
  }
  // Validates code_verifier (no secret check)
} else {
  // Validates client_secret
}
```

---

### Register a Public Client

#### **Option 1: Dynamic Registration (Recommended)**

```bash
curl -X POST http://localhost:3000/api/auth/oauth2/register \
  -H "Content-Type: application/json" \
  -d '{
    "client_name": "My SPA Application",
    "redirect_uris": ["https://myapp.com/auth/callback"],
    "type": "public",
    "grant_types": ["authorization_code", "refresh_token"],
    "response_types": ["code"],
    "scope": "openid profile email"
  }'
```

**Response:**
```json
{
  "client_id": "spa-client-abc123",
  "type": "public",
  "redirect_uris": ["https://myapp.com/auth/callback"]
  // Note: NO client_secret! ✅
}
```

#### **Option 2: Trusted Client (Pre-configured)**

Add to your `auth-config/index.ts`:

```typescript
oidcProvider({
  trustedClients: [
    {
      clientId: 'my-spa-client',
      clientSecret: null,  // ← No secret for public clients
      name: 'My SPA',
      type: 'public',      // ← Important!
      redirectURLs: ['https://myapp.com/auth/callback'],
      skipConsent: true,
    },
  ],
}),
```

---

### Implementation Guide for Public Clients

#### **Step 1: Generate PKCE Parameters**

```typescript
// utils/pkce.ts
export async function generatePKCE() {
  // Generate random code verifier
  const array = new Uint8Array(32);
  crypto.getRandomValues(array);
  const codeVerifier = Array.from(array, byte =>
    byte.toString(16).padStart(2, '0')
  ).join('');

  // Generate code challenge (SHA-256 hash)
  const encoder = new TextEncoder();
  const data = encoder.encode(codeVerifier);
  const hashBuffer = await crypto.subtle.digest('SHA-256', data);

  const codeChallenge = btoa(
    String.fromCharCode(...new Uint8Array(hashBuffer))
  )
    .replace(/\+/g, '-')
    .replace(/\//g, '_')
    .replace(/=/g, '');

  return { codeVerifier, codeChallenge };
}
```

#### **Step 2: Authorization Request (Client-Side)**

```typescript
// Start authentication flow
async function login() {
  // Generate PKCE (REQUIRED for public clients)
  const { codeVerifier, codeChallenge } = await generatePKCE();

  // Store verifier for token exchange
  sessionStorage.setItem('pkce_code_verifier', codeVerifier);

  // Generate state for CSRF protection
  const state = generateRandomString(32);
  sessionStorage.setItem('oauth_state', state);

  // Build authorization URL
  const authUrl = new URL(`${SSO_SERVER_URL}/api/auth/oauth2/authorize`);
  authUrl.searchParams.set('client_id', 'spa-client-abc123');
  authUrl.searchParams.set('redirect_uri', 'https://myapp.com/auth/callback');
  authUrl.searchParams.set('response_type', 'code');
  authUrl.searchParams.set('scope', 'openid profile email');
  authUrl.searchParams.set('code_challenge', codeChallenge);
  authUrl.searchParams.set('code_challenge_method', 'S256');
  authUrl.searchParams.set('state', state);
  authUrl.searchParams.set('nonce', generateRandomString(32));

  // Redirect to SSO
  window.location.href = authUrl.toString();
}

function generateRandomString(length: number): string {
  const array = new Uint8Array(length);
  crypto.getRandomValues(array);
  return Array.from(array, b => b.toString(16).padStart(2, '0')).join('');
}
```

#### **Step 3: Callback Handler (Token Exchange)**

```typescript
// app/auth/callback/page.tsx (or route handler)
async function handleCallback() {
  const params = new URLSearchParams(window.location.search);
  const code = params.get('code');
  const state = params.get('state');

  // Verify state (CSRF protection)
  const savedState = sessionStorage.getItem('oauth_state');
  if (state !== savedState) {
    throw new Error('Invalid state - possible CSRF attack');
  }

  // Get PKCE verifier
  const codeVerifier = sessionStorage.getItem('pkce_code_verifier');
  if (!codeVerifier) {
    throw new Error('Missing code verifier');
  }

  // Exchange code for tokens
  const response = await fetch(`${SSO_SERVER_URL}/api/auth/oauth2/token`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: new URLSearchParams({
      grant_type: 'authorization_code',
      code: code!,
      redirect_uri: 'https://myapp.com/auth/callback',
      client_id: 'spa-client-abc123',
      // NO client_secret! ✅
      code_verifier: codeVerifier,  // PKCE verifier
    }),
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.error_description || 'Token exchange failed');
  }

  const tokens = await response.json();
  // {
  //   access_token: "eyJhbGci...",
  //   id_token: "eyJhbGci...",
  //   refresh_token: "xP9mK3nR...",
  //   token_type: "Bearer",
  //   expires_in: 3600
  // }

  // Clean up
  sessionStorage.removeItem('pkce_code_verifier');
  sessionStorage.removeItem('oauth_state');

  // Store tokens (use httpOnly cookies in production!)
  localStorage.setItem('access_token', tokens.access_token);
  localStorage.setItem('id_token', tokens.id_token);

  return tokens;
}
```

#### **Step 4: Verify Tokens Locally (JWKS)**

```typescript
// lib/auth.ts
import { jwtVerify, createRemoteJWKSet } from 'jose';

const JWKS = createRemoteJWKSet(
  new URL(`${SSO_SERVER_URL}/api/auth/.well-known/jwks.json`)
);

export async function verifyAccessToken(token: string) {
  try {
    const { payload } = await jwtVerify(token, JWKS, {
      issuer: SSO_SERVER_URL,
      audience: 'spa-client-abc123',  // Your client_id
    });

    return {
      valid: true,
      userId: payload.sub as string,
      email: payload.email as string,
      name: payload.name as string,
    };
  } catch (error) {
    return { valid: false };
  }
}
```

---

### Security Considerations for Public Clients

#### ✅ **Do's**

1. **Always use PKCE** - Prevents authorization code interception
2. **Use state parameter** - Prevents CSRF attacks
3. **Use nonce parameter** - Prevents token replay attacks
4. **Verify tokens locally** - Use JWKS endpoint (no SSO calls needed)
5. **Use httpOnly cookies** - When possible (e.g., with API routes)
6. **Implement token refresh** - Keep users logged in securely

#### ❌ **Don'ts**

1. **Never hardcode client_id in public code** - Use environment variables
2. **Never store tokens in localStorage** - Vulnerable to XSS (use httpOnly cookies when possible)
3. **Never skip PKCE** - Server will reject requests from public clients
4. **Never trust client-side data** - Always verify tokens server-side before critical operations

---

### PKCE Flow Diagram

```
Client (Browser)                    SSO Server
     |                                   |
     | 1. Generate code_verifier         |
     |    hash → code_challenge          |
     |                                   |
     | 2. /authorize?                    |
     |    code_challenge=xyz...          |
     |---------------------------------→ |
     |                                   |
     |                            3. Store code_challenge
     |                               with auth code
     |                                   |
     | 4. Redirect with code             |
     | ←---------------------------------|
     |                                   |
     | 5. /token with:                   |
     |    code + code_verifier           |
     |---------------------------------→ |
     |                                   |
     |                            6. Verify:
     |                               hash(verifier) == challenge ✓
     |                                   |
     | 7. access_token + id_token        |
     | ←---------------------------------|
```

---

### Public vs Confidential Clients Comparison

| Aspect | Public Client | Confidential Client |
|--------|---------------|---------------------|
| **Type** | `"public"` | `"web"` |
| **Client Secret** | ❌ Not used | ✅ Required |
| **PKCE** | ✅ **Mandatory** | Optional (recommended) |
| **Token Exchange** | `code_verifier` only | `client_secret` + optional PKCE |
| **Use Cases** | SPAs, Mobile, Static sites | Server-side apps |
| **Secret Exposure Risk** | ✅ No risk (no secret) | ⚠️ Must protect secret |
| **Code Interception Risk** | ✅ Protected by PKCE | ⚠️ Needs PKCE for full protection |

---

### Example: Docusaurus Integration

```typescript
// docusaurus.config.js
module.exports = {
  // ... other config

  customFields: {
    ssoClientId: 'docusaurus-client-abc123',
    ssoServerUrl: 'https://sso.yourcompany.com',
    ssoCallbackUrl: 'https://docs.yourcompany.com/auth/callback',
  },
};

// src/components/SSOLogin.tsx
import { generatePKCE } from '../utils/pkce';

export function SSOLogin() {
  const handleLogin = async () => {
    const { codeVerifier, codeChallenge } = await generatePKCE();
    sessionStorage.setItem('pkce_verifier', codeVerifier);

    const authUrl = new URL(
      `${siteConfig.customFields.ssoServerUrl}/api/auth/oauth2/authorize`
    );
    authUrl.searchParams.set('client_id', siteConfig.customFields.ssoClientId);
    authUrl.searchParams.set('redirect_uri', siteConfig.customFields.ssoCallbackUrl);
    authUrl.searchParams.set('response_type', 'code');
    authUrl.searchParams.set('scope', 'openid profile email');
    authUrl.searchParams.set('code_challenge', codeChallenge);
    authUrl.searchParams.set('code_challenge_method', 'S256');

    window.location.href = authUrl.toString();
  };

  return <button onClick={handleLogin}>Sign In with SSO</button>;
}
```

---

### Testing Public Client Flow

```bash
# 1. Register public client
curl -X POST http://localhost:3000/api/auth/oauth2/register \
  -H "Content-Type: application/json" \
  -d '{
    "client_name": "Test Public Client",
    "redirect_uris": ["http://localhost:4000/callback"],
    "type": "public"
  }'

# Save the client_id from response

# 2. Generate PKCE (in browser console or Node.js)
# code_verifier: random string
# code_challenge: base64url(sha256(code_verifier))

# 3. Authorization request (in browser)
http://localhost:3000/api/auth/oauth2/authorize?client_id=CLIENT_ID&redirect_uri=http://localhost:4000/callback&response_type=code&scope=openid%20profile%20email&code_challenge=CHALLENGE&code_challenge_method=S256

# 4. After redirect, exchange code (must include code_verifier!)
curl -X POST http://localhost:3000/api/auth/oauth2/token \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -d "grant_type=authorization_code" \
  -d "code=AUTHORIZATION_CODE" \
  -d "redirect_uri=http://localhost:4000/callback" \
  -d "client_id=CLIENT_ID" \
  -d "code_verifier=VERIFIER"
  # Note: NO client_secret!
```

---

## Authentication Flow

### Complete OIDC Authorization Code Flow

```
User                 Your App              SSO Server
  |                     |                      |
  |  1. Click Login     |                      |
  |-------------------->|                      |
  |                     |                      |
  |  2. Redirect to SSO |                      |
  |<--------------------|                      |
  |                                            |
  |  3. Login & Consent                        |
  |------------------------------------------> |
  |                                            |
  |  4. Redirect with code                     |
  |<------------------------------------------ |
  |                                            |
  |  5. Code in callback                       |
  |-------------------->|                      |
  |                     |                      |
  |                     | 6. Exchange code     |
  |                     |--------------------->|
  |                     |                      |
  |                     | 7. Tokens            |
  |                     |<---------------------|
  |                     |                      |
  |  8. Set cookie      |                      |
  |     & redirect      |                      |
  |<--------------------|                      |
  |                     |                      |
  |  9. Access app      |                      |
  |-------------------->|                      |
  |                     |                      |
  |                     | 10. Verify token     |
  |                     |     LOCALLY (JWKS)   |
  |                     | ✅ No SSO call!      |
  |                     |                      |
  | 11. Show content    |                      |
  |<--------------------|                      |
```

---

## OAuth2/OIDC Parameters Explained

Understanding the key parameters in the authorization request is crucial for security and proper implementation.

### Authorization Request URL Structure

```
GET /api/auth/oauth2/authorize
  ?client_id=RdwUTXGzXgIJCrySmMZTDioHkizYqUgp
  &redirect_uri=http://localhost:4000/callback
  &response_type=code
  &scope=openid email profile
  &state=random_state_12345
  &nonce=random_nonce_67890
```

---

### 1. `scope` - What Data You're Requesting

**Purpose:** Defines what user information your application wants access to.

#### Standard OIDC Scopes

| Scope | Required? | What You Get in Tokens |
|-------|-----------|------------------------|
| `openid` | ✅ **YES** (mandatory) | Enables OIDC flow, returns `id_token` |
| `email` | ❌ Optional | `email`, `email_verified` claims |
| `profile` | ❌ Optional | `name`, `given_name`, `family_name`, `picture`, `locale` |
| `address` | ❌ Optional | Physical address information |
| `phone` | ❌ Optional | `phone_number`, `phone_number_verified` |

#### Examples: Different Scopes = Different Data

**With `scope=openid` only:**
```json
{
  "sub": "cm3lldgzv000012f9tqvkryai",
  "iss": "http://localhost:3000",
  "aud": "RdwUTXGzXgIJCrySmMZTDioHkizYqUgp",
  "exp": 1732512021
}
```

**With `scope=openid email profile`:**
```json
{
  "sub": "cm3lldgzv000012f9tqvkryai",
  "email": "user@example.com",
  "email_verified": true,
  "name": "John Doe",
  "given_name": "John",
  "family_name": "Doe",
  "picture": "https://example.com/photo.jpg",
  "locale": "en-US",
  "iss": "http://localhost:3000",
  "aud": "RdwUTXGzXgIJCrySmMZTDioHkizYqUgp",
  "exp": 1732512021
}
```

**Best Practice:** Only request scopes you actually need for privacy and better user experience.

---

### 2. `state` - CSRF Protection

**Purpose:** Prevents Cross-Site Request Forgery (CSRF) attacks.

#### The Attack Without State

```
1. Attacker creates authorization request with THEIR SSO account
2. Attacker tricks victim into clicking malicious link
3. Victim authorizes, callback goes to legitimate app
4. Legitimate app creates session with attacker's identity
5. Victim enters sensitive data thinking it's their account
6. Attacker accesses their own account and sees victim's data!
```

#### How State Prevents This

```typescript
// Step 1: Generate and save BEFORE redirecting to SSO
const state = crypto.randomBytes(32).toString('hex');
cookies().set('oauth_state', state, {
  httpOnly: true,
  maxAge: 600  // 10 minutes
});

// Step 2: Include in authorization URL
const authUrl = `${SSO_URL}/authorize?state=${state}&...`;

// Step 3: Verify in callback
const receivedState = searchParams.get('state');
const savedState = cookies().get('oauth_state')?.value;

if (receivedState !== savedState) {
  throw new Error('State mismatch - CSRF attack detected!');
}

// Step 4: Clear after verification
cookies().delete('oauth_state');
```

#### Why Save in Session/Cookie?

- You generate `state` **before** redirecting to SSO
- User goes to SSO, logs in, comes back (minutes later)
- You need to compare returned state with YOUR generated state
- **Only way to know**: Save it in YOUR session/cookie before redirect

**Security Flow:**
```
Your App generates state → Save in cookie → Redirect to SSO
                                              ↓
                        SSO returns same state in callback
                                              ↓
                    Compare returned state with saved cookie
                                              ↓
                        ✅ Match = legitimate  ❌ Mismatch = attack
```

---

### 3. `nonce` - Replay Attack Protection

**Purpose:** Prevents ID token replay attacks.

#### The Attack Without Nonce

```
1. Attacker intercepts a valid ID token from victim's session
2. Attacker replays the token to your app
3. Your app accepts it (it's validly signed!)
4. Attacker impersonates victim
```

#### How Nonce Prevents This

```typescript
// Step 1: Generate and save BEFORE redirecting to SSO
const nonce = crypto.randomBytes(32).toString('hex');
cookies().set('oauth_nonce', nonce, {
  httpOnly: true,
  maxAge: 600
});

// Step 2: Include in authorization URL
const authUrl = `${SSO_URL}/authorize?nonce=${nonce}&...`;

// Step 3: SSO embeds nonce in ID token
// ID Token will contain: { "nonce": "abc123...", "sub": "user123", ... }

// Step 4: Verify nonce in ID token after callback
const tokens = await exchangeCodeForTokens(code);
const idToken = jwtDecode(tokens.id_token);
const savedNonce = cookies().get('oauth_nonce')?.value;

if (idToken.nonce !== savedNonce) {
  throw new Error('Nonce mismatch - replay attack detected!');
}

// Step 5: Clear after verification
cookies().delete('oauth_nonce');
```

#### Why Save in Session/Cookie?

- To verify the ID token is from **THIS** specific auth session
- Prevents replayed tokens from old/stolen sessions
- Each login generates a new nonce

**Security Flow:**
```
Client generates nonce → Save in cookie → Send to SSO
                                            ↓
                        SSO embeds nonce in ID token
                                            ↓
                    Client verifies nonce in ID token matches cookie
                                            ↓
                        ✅ Match = fresh token  ❌ Mismatch = replayed token
```

---

### 4. Dynamic Return URLs with `state`

You can also use the `state` parameter to preserve the page users were trying to access:

```typescript
// Encode both CSRF token AND return URL in state
const stateData = {
  csrf: crypto.randomBytes(32).toString('hex'),
  returnTo: '/dashboard/settings'  // Where user was trying to go
};

const state = Buffer.from(JSON.stringify(stateData)).toString('base64url');

// Save CSRF token separately for verification
cookies().set('oauth_csrf', stateData.csrf, { httpOnly: true, maxAge: 600 });

// In callback, decode state and redirect to original page
const decoded = JSON.parse(Buffer.from(state, 'base64url').toString());
const savedCsrf = cookies().get('oauth_csrf')?.value;

if (decoded.csrf !== savedCsrf) {
  throw new Error('CSRF check failed');
}

// Redirect to original page
redirect(decoded.returnTo);
```

**Result:** User clicks login on `/dashboard/settings` → After SSO login → Returns to `/dashboard/settings` ✅

---

### Complete Implementation Example

```typescript
// app/login/page.tsx
'use client';

import { useSearchParams } from 'next/navigation';

export default function LoginPage() {
  const searchParams = useSearchParams();

  const handleLogin = () => {
    // 1. Get return URL (from middleware or default)
    const returnTo = searchParams.get('returnTo') || '/dashboard';

    // 2. Generate security parameters
    const csrf = generateRandom(32);
    const nonce = generateRandom(32);

    // 3. Encode state with CSRF + return URL
    const stateData = { csrf, returnTo };
    const state = btoa(JSON.stringify(stateData));

    // 4. Save for verification
    document.cookie = `oauth_csrf=${csrf}; path=/; max-age=600; SameSite=Lax; Secure`;
    document.cookie = `oauth_nonce=${nonce}; path=/; max-age=600; SameSite=Lax; Secure`;

    // 5. Build authorization URL
    const params = new URLSearchParams({
      client_id: process.env.NEXT_PUBLIC_OIDC_CLIENT_ID!,
      redirect_uri: process.env.NEXT_PUBLIC_OIDC_REDIRECT_URI!,
      response_type: 'code',
      scope: 'openid email profile',  // What data you want
      state: state,                    // CSRF + return URL
      nonce: nonce,                    // Replay protection
    });

    window.location.href = `${process.env.NEXT_PUBLIC_SSO_URL}/api/auth/oauth2/authorize?${params}`;
  };

  return <button onClick={handleLogin}>Sign in with SSO</button>;
}

function generateRandom(length: number): string {
  const array = new Uint8Array(length);
  crypto.getRandomValues(array);
  return Array.from(array, b => b.toString(16).padStart(2, '0')).join('');
}
```

```typescript
// app/auth/callback/route.ts
import { NextRequest, NextResponse } from 'next/server';
import { jwtDecode } from 'jwt-decode';
import { cookies } from 'next/headers';

export async function GET(request: NextRequest) {
  const searchParams = request.nextUrl.searchParams;
  const code = searchParams.get('code');
  const encodedState = searchParams.get('state');

  if (!code || !encodedState) {
    return new Response('Missing code or state', { status: 400 });
  }

  try {
    const cookieStore = cookies();

    // 1. Decode and verify state (CSRF protection)
    const stateData = JSON.parse(atob(encodedState));
    const savedCsrf = cookieStore.get('oauth_csrf')?.value;

    if (stateData.csrf !== savedCsrf) {
      return new Response('Invalid state - CSRF check failed', { status: 403 });
    }

    // 2. Exchange code for tokens
    const tokenResponse = await fetch(`${process.env.SSO_SERVER_URL}/api/auth/oauth2/token`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: new URLSearchParams({
        grant_type: 'authorization_code',
        code,
        redirect_uri: process.env.OIDC_REDIRECT_URI!,
        client_id: process.env.OIDC_CLIENT_ID!,
        client_secret: process.env.OIDC_CLIENT_SECRET!,
      }),
    });

    if (!tokenResponse.ok) {
      throw new Error('Token exchange failed');
    }

    const tokens = await tokenResponse.json();

    // 3. Verify nonce in ID token (replay protection)
    const idToken = jwtDecode<{ nonce?: string }>(tokens.id_token);
    const savedNonce = cookieStore.get('oauth_nonce')?.value;

    if (idToken.nonce !== savedNonce) {
      return new Response('Invalid nonce - replay attack detected', { status: 403 });
    }

    // 4. Clear security cookies
    cookieStore.delete('oauth_csrf');
    cookieStore.delete('oauth_nonce');

    // 5. Store session and redirect to original page
    const response = NextResponse.redirect(
      new URL(stateData.returnTo || '/dashboard', request.url)
    );

    response.cookies.set('app-session', JSON.stringify({
      access_token: tokens.access_token,
      id_token: tokens.id_token,
      refresh_token: tokens.refresh_token,
      expires_at: Date.now() + (tokens.expires_in * 1000),
    }), {
      httpOnly: true,
      secure: process.env.NODE_ENV === 'production',
      sameSite: 'lax',
      maxAge: tokens.expires_in,
      path: '/',
    });

    return response;

  } catch (error) {
    console.error('Auth callback error:', error);
    return NextResponse.redirect(new URL('/login?error=auth_failed', request.url));
  }
}
```

---

### Summary Table

| Parameter | Purpose | Save in Session? | Verify in Callback? | Who Uses It? |
|-----------|---------|------------------|---------------------|--------------|
| **`client_id`** | Identify your app | ❌ No (public) | ✅ Token endpoint | Browser (public) |
| **`client_secret`** | Prove your app identity | ❌ No (env var) | ✅ Token endpoint | **Server only** |
| **`scope`** | What data to request | ❌ No | ❌ No | Browser (public) |
| **`state`** | CSRF protection + return URL | ✅ **YES** | ✅ **YES** (must match) | Both |
| **`nonce`** | Replay protection | ✅ **YES** | ✅ **YES** (in ID token) | Both |
| **`redirect_uri`** | Where to return | ❌ No (static) | ✅ Token endpoint | Both |

---

### Security Checklist

Before going to production, ensure:

- ✅ `state` parameter generated with crypto-secure random
- ✅ `state` saved in httpOnly cookie before redirect
- ✅ `state` verified on callback (matches cookie)
- ✅ `nonce` parameter generated with crypto-secure random
- ✅ `nonce` saved in httpOnly cookie before redirect
- ✅ `nonce` verified in ID token (matches cookie)
- ✅ Both cookies cleared after successful verification
- ✅ Cookie settings: `httpOnly: true`, `secure: true` (production), `sameSite: 'lax'`
- ✅ `client_secret` stored in environment variables (never in code)
- ✅ `client_secret` only used server-side (never exposed to browser)
- ✅ Return URLs validated to prevent open redirect attacks

---

## Token Management

### What to Store in Your App's Cookie

After receiving tokens from the SSO server, store them in **your own cookie**:

```typescript
// Your app's cookie structure
const sessionData = {
  access_token: "eyJhbGci...",      // Full JWT
  id_token: "eyJhbGci...",          // Full JWT
  refresh_token: "xP9mK3nR...",     // Opaque string
  expires_at: 1732512021000,        // Timestamp

  // Optional: Decoded user info for quick access
  user: {
    id: "cm3lldgzv000012f9tqvkryai",
    email: "user@example.com",
    name: "John Doe",
    emailVerified: true
  }
};

// Store in httpOnly cookie
response.cookies.set('app-session', JSON.stringify(sessionData), {
  httpOnly: true,              // Prevent XSS
  secure: true,                // HTTPS only in production
  sameSite: 'lax',            // CSRF protection
  maxAge: 3600,               // 1 hour
  path: '/',
});
```

### Cookie Characteristics

| Attribute | Value | Purpose |
|-----------|-------|---------|
| `httpOnly` | `true` | Prevent JavaScript access (XSS protection) |
| `secure` | `true` (production) | HTTPS only |
| `sameSite` | `lax` | CSRF protection |
| `maxAge` | `3600` (1 hour) | Match token expiry |
| `path` | `/` | Available site-wide |
| `domain` | Your domain | Cookie scope |

**Important:**
- ❌ You do NOT receive the SSO server's cookies
- ❌ You do NOT share cookies with the SSO server
- ✅ You CREATE your own cookie with the tokens
- ✅ You manage your own session lifecycle

---

## Local Token Verification (JWKS)

### Why Local Verification?

**❌ Bad Approach:** Call SSO server on every request
```
100 page visits = 100 SSO calls
= High latency + Heavy SSO load
```

**✅ Good Approach:** Verify tokens locally using JWKS
```
100 page visits = 0 SSO calls
= Low latency + Minimal SSO load
```

### How JWKS Works

1. **SSO server signs tokens** with private key (RS256)
2. **SSO server publishes public keys** at `/api/auth/.well-known/jwks.json`
3. **Your app downloads public keys** once (caches for 24 hours)
4. **Your app verifies signatures locally** using cached public keys
5. **No SSO server call needed** for verification!

### Implementation

#### Step 1: Create JWKS Client

```typescript
// lib/jwks.ts
import { createRemoteJWKSet } from 'jose';

const SSO_URL = process.env.SSO_SERVER_URL || 'http://localhost:3000';

// Create JWKS client (automatically caches keys for 24h)
export const JWKS = createRemoteJWKSet(
  new URL(`${SSO_URL}/api/auth/.well-known/jwks.json`)
);
```

#### Step 2: Verify Tokens Locally

```typescript
// lib/auth.ts
import { jwtVerify } from 'jose';
import { JWKS } from './jwks';

const SSO_URL = process.env.SSO_SERVER_URL!;
const CLIENT_ID = process.env.OIDC_CLIENT_ID!;

/**
 * Verify access token locally (no SSO call)
 * Returns user info if valid, null if invalid
 */
export async function verifyAccessToken(token: string) {
  try {
    const { payload } = await jwtVerify(token, JWKS, {
      issuer: SSO_URL,
      audience: CLIENT_ID,
    });

    return {
      valid: true,
      userId: payload.sub as string,
      email: payload.email as string,
      name: payload.name as string,
      expiresAt: payload.exp! * 1000,
    };
  } catch (error) {
    return { valid: false };
  }
}

/**
 * Verify ID token locally (no SSO call)
 */
export async function verifyIdToken(token: string) {
  try {
    const { payload } = await jwtVerify(token, JWKS, {
      issuer: SSO_URL,
      audience: CLIENT_ID,
    });

    return {
      valid: true,
      user: {
        id: payload.sub as string,
        email: payload.email as string,
        emailVerified: payload.email_verified as boolean,
        name: payload.name as string,
      }
    };
  } catch (error) {
    return { valid: false };
  }
}
```

#### Step 3: Refresh Tokens (When Needed)

```typescript
// lib/auth.ts

const CLIENT_SECRET = process.env.OIDC_CLIENT_SECRET!;

/**
 * Refresh access token using refresh token
 * This DOES call the SSO server (but only when token expires)
 */
export async function refreshTokens(refreshToken: string) {
  const response = await fetch(`${SSO_URL}/api/auth/oauth2/token`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: new URLSearchParams({
      grant_type: 'refresh_token',
      refresh_token: refreshToken,
      client_id: CLIENT_ID,
      client_secret: CLIENT_SECRET,
    }),
  });

  if (!response.ok) {
    throw new Error('Failed to refresh token');
  }

  return response.json();
}
```

---

## Implementation Examples

### Complete Next.js App Integration

#### 1. Login Button

```typescript
// app/login/page.tsx

export default function LoginPage() {
  const handleLogin = () => {
    const params = new URLSearchParams({
      client_id: process.env.NEXT_PUBLIC_OIDC_CLIENT_ID!,
      redirect_uri: process.env.NEXT_PUBLIC_OIDC_REDIRECT_URI!,
      response_type: 'code',
      scope: 'openid profile email',
      state: generateRandomState(), // For CSRF protection
    });

    window.location.href = `${process.env.NEXT_PUBLIC_SSO_URL}/api/auth/oauth2/authorize?${params}`;
  };

  return (
    <button onClick={handleLogin}>
      Sign in with SSO
    </button>
  );
}

function generateRandomState() {
  return Math.random().toString(36).substring(7);
}
```

#### 2. Callback Handler

```typescript
// app/auth/callback/route.ts
import { NextRequest, NextResponse } from 'next/server';

export async function GET(request: NextRequest) {
  const searchParams = request.nextUrl.searchParams;
  const code = searchParams.get('code');
  const state = searchParams.get('state');

  if (!code) {
    return new Response('Missing authorization code', { status: 400 });
  }

  // Verify state (CSRF protection)
  // In production, compare with state stored in cookie/session

  try {
    // Exchange code for tokens
    const tokenResponse = await fetch(`${process.env.SSO_SERVER_URL}/api/auth/oauth2/token`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: new URLSearchParams({
        grant_type: 'authorization_code',
        code,
        redirect_uri: process.env.OIDC_REDIRECT_URI!,
        client_id: process.env.OIDC_CLIENT_ID!,
        client_secret: process.env.OIDC_CLIENT_SECRET!,
      }),
    });

    if (!tokenResponse.ok) {
      throw new Error('Token exchange failed');
    }

    const tokens = await tokenResponse.json();

    // Store tokens in cookie
    const sessionData = {
      access_token: tokens.access_token,
      id_token: tokens.id_token,
      refresh_token: tokens.refresh_token,
      expires_at: Date.now() + (tokens.expires_in * 1000),
    };

    const response = NextResponse.redirect(new URL('/dashboard', request.url));

    response.cookies.set('app-session', JSON.stringify(sessionData), {
      httpOnly: true,
      secure: process.env.NODE_ENV === 'production',
      sameSite: 'lax',
      maxAge: tokens.expires_in,
      path: '/',
    });

    return response;

  } catch (error) {
    console.error('Auth callback error:', error);
    return NextResponse.redirect(new URL('/login?error=auth_failed', request.url));
  }
}
```

#### 3. Middleware (Protect Routes)

```typescript
// middleware.ts
import { NextResponse } from 'next/server';
import type { NextRequest } from 'next/server';
import { verifyAccessToken, refreshTokens } from '@/lib/auth';

export async function middleware(request: NextRequest) {
  const sessionCookie = request.cookies.get('app-session');

  // No session - redirect to login
  if (!sessionCookie) {
    return NextResponse.redirect(new URL('/login', request.url));
  }

  try {
    const session = JSON.parse(sessionCookie.value);

    // Verify token locally (no SSO call!)
    const verification = await verifyAccessToken(session.access_token);

    if (verification.valid) {
      // Token is valid - allow access
      return NextResponse.next();
    }

    // Token invalid/expired - try to refresh
    const newTokens = await refreshTokens(session.refresh_token);

    const response = NextResponse.next();
    response.cookies.set('app-session', JSON.stringify({
      access_token: newTokens.access_token,
      id_token: newTokens.id_token,
      refresh_token: newTokens.refresh_token,
      expires_at: Date.now() + (newTokens.expires_in * 1000),
    }), {
      httpOnly: true,
      secure: process.env.NODE_ENV === 'production',
      sameSite: 'lax',
      maxAge: newTokens.expires_in,
      path: '/',
    });

    return response;

  } catch (error) {
    // Refresh failed - clear session and redirect to login
    const response = NextResponse.redirect(new URL('/login', request.url));
    response.cookies.delete('app-session');
    return response;
  }
}

export const config = {
  matcher: [
    '/dashboard/:path*',
    '/profile/:path*',
    '/settings/:path*',
  ],
};
```

#### 4. Using User Info in Pages

```typescript
// app/dashboard/page.tsx
import { cookies } from 'next/headers';
import { jwtDecode } from 'jwt-decode';
import { redirect } from 'next/navigation';

export default async function DashboardPage() {
  const sessionCookie = cookies().get('app-session');

  if (!sessionCookie) {
    redirect('/login');
  }

  const session = JSON.parse(sessionCookie.value);

  // Decode ID token to get user info
  const userInfo = jwtDecode(session.id_token);

  return (
    <div>
      <h1>Welcome, {userInfo.name}!</h1>
      <p>Email: {userInfo.email}</p>
      <p>Email verified: {userInfo.email_verified ? 'Yes' : 'No'}</p>
    </div>
  );
}
```

#### 5. Logout

```typescript
// app/logout/route.ts
import { NextResponse } from 'next/server';

export async function POST() {
  const response = NextResponse.redirect(new URL('/login', request.url));

  // Clear session cookie
  response.cookies.delete('app-session');

  return response;
}
```

---

## Security Best Practices

### 1. State Parameter (CSRF Protection)

Always use the `state` parameter to prevent CSRF attacks:

```typescript
// Generate random state
const state = crypto.randomBytes(32).toString('hex');

// Store in session/cookie
cookies().set('oauth_state', state, { httpOnly: true });

// Include in authorization request
const authUrl = `${SSO_URL}/api/auth/oauth2/authorize?state=${state}&...`;

// Verify in callback
const receivedState = searchParams.get('state');
const storedState = cookies().get('oauth_state')?.value;

if (receivedState !== storedState) {
  throw new Error('Invalid state - possible CSRF attack');
}
```

### 2. PKCE (Optional but Recommended)

For additional security, implement PKCE:

```typescript
// Generate code verifier
const codeVerifier = crypto.randomBytes(64).toString('base64url');

// Generate code challenge
const hash = crypto.createHash('sha256').update(codeVerifier).digest();
const codeChallenge = hash.toString('base64url');

// Store verifier
cookies().set('code_verifier', codeVerifier, { httpOnly: true });

// Include challenge in authorization request
const authUrl = `${SSO_URL}/api/auth/oauth2/authorize?code_challenge=${codeChallenge}&code_challenge_method=S256&...`;

// Include verifier in token exchange
body: new URLSearchParams({
  grant_type: 'authorization_code',
  code,
  code_verifier: cookies().get('code_verifier')?.value,
  // ...
});
```

### 3. Secure Cookie Settings

Always use these cookie settings:

```typescript
{
  httpOnly: true,              // ✅ Prevent JavaScript access
  secure: true,                // ✅ HTTPS only (production)
  sameSite: 'lax',            // ✅ CSRF protection
  maxAge: 3600,               // ✅ Match token expiry
  path: '/',                   // ✅ Available site-wide
}
```

### 4. Token Storage

**DO:**
- ✅ Store tokens in httpOnly cookies (server-side only)
- ✅ Use secure flag in production
- ✅ Set appropriate maxAge

**DON'T:**
- ❌ Store tokens in localStorage (XSS vulnerable)
- ❌ Store tokens in sessionStorage
- ❌ Expose tokens to client-side JavaScript

### 5. Client Secret Protection

**DO:**
- ✅ Store client secret in environment variables
- ✅ Only use client secret in server-side code
- ✅ Never expose client secret to browser

**DON'T:**
- ❌ Include client secret in client-side code
- ❌ Commit client secret to version control
- ❌ Log client secret

---

## Performance Optimization

### Traffic Comparison

| Scenario | SSO Calls | Latency | Load |
|----------|-----------|---------|------|
| **With Local Verification ✅** | 3 per hour | 100ms | Minimal |
| **Without Local Verification ❌** | 100+ per hour | 5-10s | Heavy |

### Best Practices

#### 1. Cache JWKS Keys

```typescript
// ✅ GOOD - Keys cached for 24 hours
const JWKS = createRemoteJWKSet(
  new URL(`${SSO_URL}/api/auth/.well-known/jwks.json`)
);
```

#### 2. Verify Locally

```typescript
// ✅ GOOD - Local verification (1ms)
await jwtVerify(token, JWKS, { issuer, audience });

// ❌ BAD - Network call to SSO (50-100ms)
await fetch(`${SSO_URL}/api/auth/oauth2/userinfo`, {
  headers: { Authorization: `Bearer ${token}` }
});
```

#### 3. Refresh Strategically

```typescript
// ✅ GOOD - Refresh when needed
if (Date.now() > session.expires_at) {
  await refreshTokens(session.refresh_token);
}

// ❌ BAD - Check with SSO on every request
await fetch(`${SSO_URL}/api/auth/oauth2/userinfo`);
```

#### 4. Pre-decode User Info (Optional)

```typescript
// Store decoded user info in cookie for quick access
const sessionData = {
  access_token: tokens.access_token,
  id_token: tokens.id_token,
  refresh_token: tokens.refresh_token,
  expires_at: Date.now() + tokens.expires_in * 1000,

  // Pre-decoded for quick UI rendering
  user: jwtDecode(tokens.id_token),
};
```

---

## Troubleshooting

### Common Issues

#### 1. "Invalid signature" Error

**Problem:** JWT verification fails

**Solutions:**
- Verify `issuer` matches SSO server URL
- Verify `audience` matches your client_id
- Check JWKS endpoint is accessible
- Clear JWKS cache if keys were rotated

#### 2. Tokens Expired

**Problem:** Access token expired

**Solution:**
```typescript
// Use refresh token to get new tokens
const newTokens = await refreshTokens(session.refresh_token);
```

#### 3. Redirect Loop

**Problem:** Middleware keeps redirecting to login

**Solutions:**
- Exclude `/login` and `/auth/callback` from middleware matcher
- Check cookie is being set correctly
- Verify cookie domain/path settings

```typescript
export const config = {
  matcher: [
    '/((?!login|auth/callback|_next/static|_next/image|favicon.ico).*)',
  ],
};
```

#### 4. CORS Errors

**Problem:** Browser blocks SSO requests

**Solution:**
- Use server-side code for token exchange (not client-side)
- Ensure proper redirect URIs are registered
- SSO server allows your domain

#### 5. State Mismatch

**Problem:** "Invalid state" error in callback

**Solutions:**
- Ensure state is stored before redirect
- Verify cookie settings allow state to persist
- Check state is passed through authorization flow

---

## API Reference

### SSO Server Endpoints

#### Discovery Document
```
GET http://localhost:3000/api/auth/.well-known/openid-configuration
```

Returns all OIDC endpoints and capabilities.

#### JWKS (Public Keys)
```
GET http://localhost:3000/api/auth/.well-known/jwks.json
```

Returns public keys for JWT verification.

#### Authorization Endpoint
```
GET http://localhost:3000/api/auth/oauth2/authorize
  ?client_id={client_id}
  &redirect_uri={redirect_uri}
  &response_type=code
  &scope=openid+profile+email
  &state={random_state}
```

Initiates OIDC login flow.

#### Token Endpoint
```
POST http://localhost:3000/api/auth/oauth2/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code
code={authorization_code}
redirect_uri={redirect_uri}
client_id={client_id}
client_secret={client_secret}
```

Exchanges authorization code for tokens.

#### Token Refresh
```
POST http://localhost:3000/api/auth/oauth2/token
Content-Type: application/x-www-form-urlencoded

grant_type=refresh_token
refresh_token={refresh_token}
client_id={client_id}
client_secret={client_secret}
```

Gets new tokens using refresh token.

#### UserInfo Endpoint (Optional)
```
GET http://localhost:3000/api/auth/oauth2/userinfo
Authorization: Bearer {access_token}
```

Returns user information. **Note:** Prefer using ID token instead to avoid this call.

#### Client Registration
```
POST http://localhost:3000/api/auth/oauth2/register
Content-Type: application/json

{
  "client_name": "My App",
  "redirect_uris": ["http://localhost:4000/auth/callback"]
}
```

Registers a new OIDC client.

---

## Token Structures

### Access Token (JWT)

```json
{
  "sub": "cm3lldgzv000012f9tqvkryai",
  "email": "user@example.com",
  "email_verified": true,
  "name": "John Doe",
  "iss": "http://localhost:3000",
  "aud": "RdwUTXGzXgIJCrySmMZTDioHkizYqUgp",
  "iat": 1732508421,
  "exp": 1732512021
}
```

### ID Token (JWT)

```json
{
  "sub": "cm3lldgzv000012f9tqvkryai",
  "email": "user@example.com",
  "email_verified": true,
  "name": "John Doe",
  "iss": "http://localhost:3000",
  "aud": "RdwUTXGzXgIJCrySmMZTDioHkizYqUgp",
  "iat": 1732508421,
  "exp": 1732512021
}
```

---

## Testing

### Test OIDC Flow

```bash
# 1. Get authorization code (browser)
http://localhost:3000/api/auth/oauth2/authorize?client_id=YOUR_CLIENT_ID&redirect_uri=http://localhost:4000/auth/callback&response_type=code&scope=openid+profile+email&state=test123

# 2. Exchange code for tokens
curl -X POST http://localhost:3000/api/auth/oauth2/token \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -d "grant_type=authorization_code" \
  -d "code=AUTHORIZATION_CODE" \
  -d "redirect_uri=http://localhost:4000/auth/callback" \
  -d "client_id=YOUR_CLIENT_ID" \
  -d "client_secret=YOUR_CLIENT_SECRET"

# 3. Verify JWT locally (using jose library)
node -e "
const { jwtVerify, createRemoteJWKSet } = require('jose');
const JWKS = createRemoteJWKSet(new URL('http://localhost:3000/api/auth/.well-known/jwks.json'));
jwtVerify('YOUR_ACCESS_TOKEN', JWKS).then(console.log);
"
```

---

## Support

### Documentation
- Main SSO Server README: `/apps/sso-server/README.md`
- API Testing Guide: `/API_TESTING_GUIDE.md`
- OIDC Flow Guide: `/OIDC_FLOW_GUIDE.md`

### Contact
For issues or questions, contact the SSO server administrator.

---

## License

Proprietary - Panaversity SSO Platform
