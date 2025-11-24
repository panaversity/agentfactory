# JWT, JWKS & OIDC Multi-Site SSO Guide

**Last Updated:** 2025-11-23

## 📚 Table of Contents

1. [Overview](#overview)
2. [Current Configuration](#current-configuration)
3. [Understanding JWT vs JWKS](#understanding-jwt-vs-jwks)
4. [Why JWK Table is Empty](#why-jwk-table-is-empty)
5. [OIDC Authorization Code Flow Explained](#oidc-authorization-code-flow-explained)
6. [How It Works for Multi-Site SSO](#how-it-works-for-multi-site-sso)
7. [Testing & Verification](#testing--verification)
8. [Common Issues & Solutions](#common-issues--solutions)

---

## Overview

This SSO platform uses **JWT tokens signed with JWKS (JSON Web Key Sets)** to enable secure, scalable multi-site authentication using the **OIDC (OpenID Connect)** protocol.

### Key Features

✅ **JWT Tokens**: Self-contained tokens with user information
✅ **JWKS (RS256)**: Asymmetric encryption with public/private keys
✅ **OIDC Provider**: Standard OAuth 2.0 / OpenID Connect flow
✅ **Multi-Site SSO**: Multiple client sites can verify tokens independently
✅ **Auto-Generated Keys**: Keys are created automatically on first use

---

## Current Configuration

### Location
`/packages/auth-config/index.ts`

### JWT Plugin Configuration

```typescript
jwt({
  disableSettingJwtHeader: true,
  jwks: {
    keyPairConfig: {
      alg: 'RS256',  // RSA with SHA-256 (asymmetric encryption)
    },
  },
}),
```

**What this means:**
- **Algorithm**: RS256 (RSA with SHA-256)
- **Key Type**: Asymmetric (public/private key pair)
- **Private Key**: Stored in database, used to SIGN tokens
- **Public Key**: Exposed at `/api/auth/jwks`, used to VERIFY tokens

### OIDC Provider Configuration

```typescript
oidcProvider({
  loginPage: '/auth/login',
  consentPage: '/auth/consent',
  useJWTPlugin: true,  // ← Uses JWT tokens for ID tokens
  allowDynamicClientRegistration: true,

  trustedClients: [
    {
      clientId: 'internal-dashboard',
      clientSecret: 'secret-for-internal-dashboard',
      name: 'Internal Dashboard',
      type: 'web',
      redirectURLs: [
        'http://localhost:3001/auth/callback',
        'http://localhost:3002/auth/callback',
      ],
      skipConsent: true,  // No consent screen for trusted clients
    },
  ],
}),
```

---

## Understanding JWT vs JWKS

### What is JWT?

**JWT (JSON Web Token)** is a self-contained token that includes:
- **Header**: Algorithm and token type
- **Payload**: User information (claims)
- **Signature**: Cryptographic signature to verify authenticity

**Example JWT:**
```
eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCIsImtpZCI6ImM1Yzc5OTVkLi4uIn0.
eyJzdWIiOiJ1c2VyXzEyMyIsImVtYWlsIjoidXNlckBleGFtcGxlLmNvbSIsImlhdCI6MTcwMzQ2MjQwMH0.
abc123def456...signature...
```

**Decoded Payload:**
```json
{
  "sub": "user_123",           // User ID
  "email": "user@example.com", // Email
  "iat": 1703462400,           // Issued at
  "exp": 1703548800,           // Expires at
  "iss": "http://localhost:3000", // Issuer (your SSO server)
  "aud": "http://localhost:3000"  // Audience
}
```

### What is JWKS?

**JWKS (JSON Web Key Set)** is a set of public keys used to verify JWT signatures.

**Why JWKS?**
- **Asymmetric Encryption**: Private key signs, public key verifies
- **Secure Distribution**: Public keys can be shared safely
- **No Shared Secret**: Client sites don't need the private key
- **Key Rotation**: Can rotate keys without breaking existing tokens

### JWT vs Session Tokens

| Aspect | Session Token (Cookie) | JWT with JWKS |
|--------|------------------------|---------------|
| **Storage** | Server-side database | Self-contained (no DB lookup) |
| **Verification** | Query database on every request | Verify signature with public key |
| **Multi-site** | Requires shared session store or federated DB | ✅ Each site verifies independently |
| **Scalability** | Limited (DB queries on every request) | ✅ Highly scalable (stateless) |
| **Use case** | Single application | ✅ **Multi-site SSO (your setup)** |
| **Token exposure** | Cookie-based, httpOnly | Can be stored in localStorage or httpOnly cookie |

**Your Configuration:** You're using JWT tokens with JWKS for multi-site SSO! ✅

---

## Why JWK Table is Empty

### The Answer: Keys are Auto-Generated on First Use

**Your `jwks` database table is empty because:**

1. **Lazy Generation**: BetterAuth generates JWKS keys **on-demand** when first needed
2. **Not Yet Triggered**: No one has accessed the JWKS endpoint or requested a JWT token yet
3. **This is Normal**: It's not an error - keys will be created automatically

### When Keys Are Generated

Keys are automatically created when:

1. ✅ **First JWKS endpoint access**: `GET /api/auth/jwks`
2. ✅ **First OIDC token request**: When a client exchanges an authorization code for tokens
3. ✅ **First JWT generation**: Any operation that requires signing a JWT

### How to Initialize Keys (3 Methods)

#### Method 1: Access JWKS Endpoint (Recommended)

```bash
# This will automatically generate keys if they don't exist
curl http://localhost:3000/api/auth/jwks | jq
```

**Expected Response (after generation):**
```json
{
  "keys": [
    {
      "kty": "RSA",
      "use": "sig",
      "alg": "RS256",
      "kid": "c5c7995d-0037-4553-8aee-b5b620b89b23",
      "n": "xGOr1TF...",  // Public key modulus
      "e": "AQAB"         // Public key exponent
    }
  ]
}
```

#### Method 2: Complete an OIDC Login Flow

```bash
# 1. Start OIDC flow (do this in browser)
open "http://localhost:3000/api/auth/authorize?client_id=internal-dashboard&response_type=code&redirect_uri=http://localhost:3001/auth/callback&scope=openid%20profile%20email"

# 2. Login with email/GitHub/Google
# 3. Keys will be generated when exchanging code for tokens
```

#### Method 3: Sign Up a Test User

```bash
# Sign up will create a session, which may trigger key generation
curl -X POST http://localhost:3000/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "TestPass123!",
    "name": "Test User"
  }' | jq
```

### Verify Keys Were Created

```bash
# Check database
psql $DATABASE_URL -c "SELECT id, created_at FROM jwks;"

# Check JWKS endpoint
curl http://localhost:3000/api/auth/jwks | jq '.keys[].kid'
```

---

## OIDC Authorization Code Flow Explained

### The Question: "Why not return tokens directly?"

**Great question!** Let's understand why OIDC uses a two-step process (code exchange) instead of returning tokens directly.

### Security Comparison

#### ❌ BAD: Implicit Flow (Tokens in URL)

```
User clicks "Sign in with SSO"
    ↓
SSO Server redirects with tokens IN THE URL
    ↓
http://localhost:3001/auth/callback#
  access_token=abc123&
  id_token=eyJhbG...
```

**Problems:**
1. 🔴 **Browser History**: Tokens stored in browser history (HUGE security risk!)
2. 🔴 **URL Leakage**: Tokens visible in referrer headers, analytics, logs
3. 🔴 **No Client Authentication**: Can't verify which client is making the request
4. 🔴 **XSS Attacks**: Easier to steal tokens via JavaScript
5. 🔴 **No Refresh Tokens**: Can't issue refresh tokens securely

#### ✅ GOOD: Authorization Code Flow (Your Setup)

```
User clicks "Sign in with SSO"
    ↓
SSO Server redirects with ONE-TIME CODE (not the token!)
    ↓
http://localhost:3001/auth/callback?code=xyz789
    ↓
Client site makes BACKEND request to exchange code for tokens
    ↓
POST /api/auth/token
  code=xyz789
  client_id=internal-dashboard
  client_secret=secret-for-internal-dashboard  ← Client authenticates!
    ↓
SSO Server returns tokens (NEVER exposed to browser URL)
    ↓
{
  "access_token": "abc123",
  "id_token": "eyJhbG...",    ← JWT signed with private key
  "refresh_token": "refresh123"
}
```

**Benefits:**
1. ✅ **No Token Leakage**: Tokens never appear in browser URL or history
2. ✅ **Client Authentication**: Server verifies client identity with `client_secret`
3. ✅ **Short-lived Code**: Authorization code expires in ~60 seconds, usable once
4. ✅ **Refresh Tokens**: Can issue long-lived refresh tokens securely
5. ✅ **PKCE Support**: Additional protection against authorization code interception
6. ✅ **Industry Standard**: OAuth 2.0 / OpenID Connect best practice

### Visual Comparison

#### Implicit Flow (Insecure)
```
Browser URL: http://localhost:3001/callback#access_token=SECRET_TOKEN_HERE
                                          ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                                          EXPOSED IN BROWSER HISTORY! 🔴
```

#### Authorization Code Flow (Secure)
```
Browser URL: http://localhost:3001/callback?code=TEMPORARY_CODE_XYZ
                                          ^^^^^^^^^^^^^^^^^^^^
                                          Expires in 60 seconds ✅

Backend Request (not visible to browser):
POST /api/auth/token
{
  "code": "TEMPORARY_CODE_XYZ",
  "client_secret": "secret"  ← Proves client identity
}

Response (sent directly to backend):
{
  "access_token": "SECRET_TOKEN",  ← Never exposed to browser URL
  "id_token": "eyJhbG..."
}
```

### Why This Matters for Multi-Site SSO

**With Authorization Code Flow:**
1. ✅ **Secure Token Exchange**: Tokens transmitted via secure backend-to-backend requests
2. ✅ **Client Verification**: SSO server verifies which client site is making the request
3. ✅ **Prevent Token Theft**: Even if authorization code is intercepted, attacker needs `client_secret`
4. ✅ **Audit Trail**: Server knows exactly which client received which tokens

**Summary:**
The two-step process (code → token exchange) is NOT inefficient - it's a **critical security feature** that prevents token leakage and enables client authentication. This is why OAuth 2.0 and OIDC use this pattern.

---

## How It Works for Multi-Site SSO

### Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    SSO Server (port 3000)                   │
│                                                             │
│  ┌─────────────┐         ┌──────────────┐                 │
│  │  Auth API   │────────▶│ JWKS Storage │                 │
│  │             │         │  (Database)   │                 │
│  │ - Login     │         │               │                 │
│  │ - Register  │         │ Private Keys  │                 │
│  │ - OAuth     │         │ Public Keys   │                 │
│  └─────────────┘         └──────────────┘                 │
│         │                                                   │
│         │  Signs JWT with Private Key                      │
│         ▼                                                   │
│  ┌─────────────┐                                           │
│  │   /jwks     │────────▶ Public Keys (no auth required)   │
│  │  Endpoint   │                                           │
│  └─────────────┘                                           │
└─────────────────────────────────────────────────────────────┘
         │                            ▲
         │ ID Token (JWT)             │ Fetch public keys
         │                            │
         ▼                            │
┌──────────────────┐        ┌──────────────────┐
│  Client Site 1   │        │  Client Site 2   │
│  (port 3001)     │        │  (port 3002)     │
│                  │        │                  │
│ Verifies JWT     │        │ Verifies JWT     │
│ with public key  │        │ with public key  │
└──────────────────┘        └──────────────────┘
```

### Complete Flow

#### Step 1: User Initiates Login on Client Site

```
User visits: http://localhost:3001
    ↓
Client site redirects to SSO server
    ↓
http://localhost:3000/api/auth/authorize?
  client_id=internal-dashboard&
  response_type=code&
  redirect_uri=http://localhost:3001/auth/callback&
  scope=openid+profile+email&
  state=random_state_123
```

#### Step 2: User Authenticates on SSO Server

```
User sees login page at SSO server
    ↓
User logs in with:
  - Email/Password
  - GitHub OAuth
  - Google OAuth
    ↓
SSO server creates session
    ↓
SSO server generates ONE-TIME authorization code
```

#### Step 3: SSO Server Redirects with Code

```
http://localhost:3001/auth/callback?
  code=TEMPORARY_AUTH_CODE_XYZ&
  state=random_state_123
```

#### Step 4: Client Site Exchanges Code for Tokens (Backend)

```typescript
// Client site backend (NOT in browser!)
const response = await fetch('http://localhost:3000/api/auth/token', {
  method: 'POST',
  headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
  body: new URLSearchParams({
    grant_type: 'authorization_code',
    code: 'TEMPORARY_AUTH_CODE_XYZ',
    client_id: 'internal-dashboard',
    client_secret: 'secret-for-internal-dashboard',  // ← Authenticates client
    redirect_uri: 'http://localhost:3001/auth/callback',
  }),
});

const tokens = await response.json();
// {
//   "access_token": "...",
//   "id_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCIsImtpZCI6ImM1Yzc5OTVkLi4uIn0...",
//   "token_type": "Bearer",
//   "expires_in": 3600
// }
```

#### Step 5: Client Site Verifies JWT Token

```typescript
// Client site verifies ID token with public key
import { jwtVerify, createRemoteJWKSet } from 'jose'

const JWKS = createRemoteJWKSet(
  new URL('http://localhost:3000/api/auth/jwks')
)

const { payload } = await jwtVerify(tokens.id_token, JWKS, {
  issuer: 'http://localhost:3000',
  audience: 'http://localhost:3000',
})

// payload contains:
// {
//   "sub": "user_123",
//   "email": "user@example.com",
//   "name": "John Doe",
//   "email_verified": true,
//   "iat": 1703462400,
//   "exp": 1703548800
// }
```

#### Step 6: Client Site Creates Local Session

```typescript
// Client site creates its own session
req.session.user = {
  id: payload.sub,
  email: payload.email,
  name: payload.name,
}

// Redirect to dashboard
res.redirect('/dashboard')
```

### Key Benefits

1. ✅ **Centralized Authentication**: Users log in once on SSO server
2. ✅ **Independent Verification**: Each client site verifies tokens using public keys (no database)
3. ✅ **No Shared Database**: Client sites don't need access to SSO database
4. ✅ **Scalable**: No bottleneck on SSO server (stateless token verification)
5. ✅ **Secure**: Private key never leaves SSO server

---

## Testing & Verification

### 1. Generate JWKS (If Empty)

```bash
# Access JWKS endpoint to trigger key generation
curl http://localhost:3000/api/auth/jwks | jq

# Verify keys in database
psql $DATABASE_URL -c "SELECT id, created_at FROM jwks;"
```

### 2. Test OIDC Flow (Browser)

```bash
# Open in browser
open "http://localhost:3000/api/auth/authorize?client_id=internal-dashboard&response_type=code&redirect_uri=http://localhost:3001/auth/callback&scope=openid%20profile%20email"
```

**What happens:**
1. Redirects to login page
2. After login, redirects to `http://localhost:3001/auth/callback?code=...`
3. Client site exchanges code for tokens (backend)
4. Client site verifies JWT with public keys

### 3. Verify JWT Token (Client Site)

```typescript
// Install jose
// npm install jose

import { jwtVerify, createRemoteJWKSet } from 'jose'

async function verifyToken(idToken: string) {
  const JWKS = createRemoteJWKSet(
    new URL('http://localhost:3000/api/auth/jwks')
  )

  const { payload } = await jwtVerify(idToken, JWKS, {
    issuer: 'http://localhost:3000',
    audience: 'http://localhost:3000',
  })

  console.log('User ID:', payload.sub)
  console.log('Email:', payload.email)
  console.log('Email Verified:', payload.email_verified)

  return payload
}

// Usage
const payload = await verifyToken('eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCIsImtpZCI6ImM1Yzc5OTVkLi4uIn0...')
```

### 4. Test Multi-Site SSO

```bash
# Start all servers
cd apps/sso-server && npm run dev     # Port 3000
cd apps/client-1 && npm run dev       # Port 3001
cd apps/client-2 && npm run dev       # Port 3002

# Test flow:
# 1. Login on client-1 (port 3001)
# 2. Open client-2 (port 3002) - should auto-login (SSO magic!)
```

---

## Common Issues & Solutions

### Issue 1: JWKS Table is Empty

**Symptom:**
```sql
SELECT * FROM jwks;
-- (0 rows)
```

**Cause:** Keys not yet generated (lazy initialization)

**Solution:**
```bash
# Trigger key generation
curl http://localhost:3000/api/auth/jwks | jq

# Verify
psql $DATABASE_URL -c "SELECT id, created_at FROM jwks;"
```

---

### Issue 2: "Invalid signature" when verifying JWT

**Symptom:**
```
Error: signature verification failed
```

**Causes:**
1. Wrong issuer/audience in verification
2. Using wrong JWKS endpoint
3. Token signed with different key

**Solution:**
```typescript
// Ensure issuer and audience match your config
const { payload } = await jwtVerify(token, JWKS, {
  issuer: 'http://localhost:3000',      // ← Must match BETTER_AUTH_URL
  audience: 'http://localhost:3000',    // ← Must match BETTER_AUTH_URL
})

// Check token header
const header = JSON.parse(atob(token.split('.')[0]))
console.log('Token kid:', header.kid)

// Check JWKS has matching kid
const jwks = await fetch('http://localhost:3000/api/auth/jwks').then(r => r.json())
console.log('JWKS kids:', jwks.keys.map(k => k.kid))
```

---

### Issue 3: "Redirect URI mismatch" in OIDC flow

**Symptom:**
```
Error: redirect_uri does not match
```

**Cause:** Redirect URI not in `trustedClients` config

**Solution:**
```typescript
// Add to /packages/auth-config/index.ts
oidcProvider({
  trustedClients: [
    {
      clientId: 'internal-dashboard',
      redirectURLs: [
        'http://localhost:3001/auth/callback',  // ← Must match exactly
        'http://localhost:3002/auth/callback',
        // Add more as needed
      ],
    },
  ],
}),
```

---

### Issue 4: Token expired

**Symptom:**
```
Error: token is expired
```

**Cause:** JWT tokens have expiration (default: 1 hour)

**Solution:**
```typescript
// Use refresh token to get new access token
const response = await fetch('http://localhost:3000/api/auth/token', {
  method: 'POST',
  body: new URLSearchParams({
    grant_type: 'refresh_token',
    refresh_token: 'your_refresh_token',
    client_id: 'internal-dashboard',
    client_secret: 'secret-for-internal-dashboard',
  }),
})

const { access_token, id_token } = await response.json()
```

---

### Issue 5: Keys rotated, old tokens fail verification

**Symptom:**
```
Error: no key found with kid "old-key-id"
```

**Cause:** JWKS rotated, but JWKS cache not refreshed

**Solution:**
```typescript
// Refetch JWKS when kid not found
try {
  const { payload } = await jwtVerify(token, JWKS, { ... })
} catch (error) {
  if (error.code === 'ERR_JWKS_NO_MATCHING_KEY') {
    // Refetch JWKS and retry
    const freshJWKS = createRemoteJWKSet(
      new URL('http://localhost:3000/api/auth/jwks')
    )
    const { payload } = await jwtVerify(token, freshJWKS, { ... })
  }
}
```

---

## Environment Variables

```bash
# SSO Server (.env)
DATABASE_URL=postgresql://...
BETTER_AUTH_SECRET=your-secret-key-here
BETTER_AUTH_URL=http://localhost:3000

# OIDC Clients
INTERNAL_CLIENT_ID=internal-dashboard
INTERNAL_CLIENT_SECRET=secret-for-internal-dashboard

# OAuth Providers
GITHUB_CLIENT_ID=...
GITHUB_CLIENT_SECRET=...
GOOGLE_CLIENT_ID=...
GOOGLE_CLIENT_SECRET=...

# Email
RESEND_API_KEY=re_...
RESEND_FROM_EMAIL=noreply@yourdomain.com
```

```bash
# Client Site 1 (.env)
NEXT_PUBLIC_SSO_URL=http://localhost:3000
OAUTH_CLIENT_ID=internal-dashboard
OAUTH_CLIENT_SECRET=secret-for-internal-dashboard
```

---

## Quick Reference

### Endpoints

```bash
# JWKS (Public Keys)
GET  http://localhost:3000/api/auth/jwks

# OIDC Authorization (Browser)
GET  http://localhost:3000/api/auth/authorize?client_id=...&response_type=code&redirect_uri=...&scope=openid+profile+email

# OIDC Token Exchange (Backend)
POST http://localhost:3000/api/auth/token
Body: grant_type=authorization_code&code=...&client_id=...&client_secret=...

# User Session
GET  http://localhost:3000/api/auth/session
Cookie: better-auth.session_token=...
```

### Key Files

```
/packages/auth-config/index.ts       ← JWT & OIDC configuration
/packages/database/schema/auth-schema.ts  ← Database schema (jwks table)
/apps/sso-server/.env.local          ← SSO server environment variables
```

---

## Summary

✅ **Your Setup**: JWT tokens signed with JWKS (RS256) + OIDC Provider
✅ **Empty Table**: Normal - keys auto-generated on first use
✅ **Generate Keys**: Access `/api/auth/jwks` endpoint
✅ **Code Exchange**: Security best practice (prevents token leakage)
✅ **Multi-Site SSO**: Client sites verify tokens independently with public keys

**Next Steps:**
1. Access `http://localhost:3000/api/auth/jwks` to generate keys
2. Verify keys in database: `SELECT * FROM jwks;`
3. Test OIDC flow with a client site
4. Implement token verification on client sites using `jose` library

---

**Questions? Issues?**
Refer to this guide or check BetterAuth documentation: https://better-auth.com
