# OIDC Authorization Code Flow Guide

Complete guide explaining how users authenticate from other applications using your SSO server.

## 🎯 Overview

This document explains the **Authorization Code Flow** - the most secure and commonly used OIDC authentication flow for web applications.

### Scenario
User wants to login to **Client App** (e.g., Dashboard at `localhost:3001`) using your **SSO Server** (`localhost:3000`)

---

## 🔄 Complete Step-by-Step Flow

### **Step 1: User Clicks "Login with SSO" on Client App**

**Client App** creates an authorization URL and redirects the user:

```javascript
// On Client App (localhost:3001)
const authUrl = new URL('http://localhost:3000/api/auth/oauth2/authorize');
authUrl.searchParams.set('client_id', 'internal-dashboard');
authUrl.searchParams.set('redirect_uri', 'http://localhost:3001/auth/callback');
authUrl.searchParams.set('response_type', 'code');
authUrl.searchParams.set('scope', 'openid email profile');
authUrl.searchParams.set('state', 'random_state_12345'); // CSRF protection
authUrl.searchParams.set('nonce', 'random_nonce_67890'); // Replay attack protection

// Redirect user to SSO server
window.location.href = authUrl.toString();
```

**Result URL:**
```
http://localhost:3000/api/auth/oauth2/authorize?client_id=internal-dashboard&redirect_uri=http://localhost:3001/auth/callback&response_type=code&scope=openid+email+profile&state=random_state_12345&nonce=random_nonce_67890
```

**What happens:** Browser redirects user to SSO Server's authorization endpoint

---

### **Step 2: SSO Server Checks if User is Logged In**

**Endpoint Called:** `GET /api/auth/oauth2/authorize` (on SSO Server)

**SSO Server checks:**
- ✅ Is the user already logged in? (checks session cookie)
- ❌ **No** → Redirect to login page
- ✅ **Yes** → Skip to Step 4 (consent)

**If NOT logged in, SSO Server redirects to:**
```
http://localhost:3000/auth/login?returnTo=/api/auth/oauth2/authorize?client_id=...
```

---

### **Step 3: User Logs In on SSO Server**

User enters credentials on your SSO login page:

```bash
# Behind the scenes, browser calls:
POST http://localhost:3000/api/auth/sign-in/email
Content-Type: application/json

{
  "email": "user@example.com",
  "password": "SecurePass123!"
}
```

**Response:**
```json
{
  "token": "session_token_abc123",
  "user": {
    "id": "user_123",
    "email": "user@example.com",
    "name": "John Doe"
  }
}
```

**SSO Server:**
- Sets session cookie: `better-auth.session_token=session_token_abc123`
- Redirects back to authorization endpoint (Step 2 again)

---

### **Step 4: User Grants Consent (if required)**

**Endpoint Called:** `GET /api/auth/oauth2/authorize` (again, now user is logged in)

**SSO Server checks:**
- Does this client require consent?
- For `internal-dashboard`: **No** (skipConsent: true)
- For other clients: **Yes** → Show consent page

**If consent required, SSO Server redirects to:**
```
http://localhost:3000/auth/consent?client_id=internal-dashboard&scope=openid+email+profile
```

User clicks "Allow" → consent granted

---

### **Step 5: SSO Server Generates Authorization Code**

**SSO Server:**
1. Creates an authorization code (short-lived, single-use)
2. Associates it with:
   - User ID
   - Client ID
   - Requested scopes
   - Redirect URI
   - Nonce
3. Redirects user back to Client App

**Redirect:**
```
http://localhost:3001/auth/callback?code=AUTH_CODE_ABC123XYZ&state=random_state_12345
```

**User's browser is now back at Client App!**

---

### **Step 6: Client App Exchanges Code for Tokens**

**Client App (backend)** receives the callback and extracts the `code`:

```javascript
// On Client App backend (localhost:3001)
const code = 'AUTH_CODE_ABC123XYZ';
const state = 'random_state_12345';

// Verify state matches (CSRF protection)
if (state !== savedState) {
  throw new Error('Invalid state');
}

// Exchange code for tokens (server-to-server call)
const response = await fetch('http://localhost:3000/api/auth/oauth2/token', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/x-www-form-urlencoded',
    'Authorization': 'Basic ' + btoa('internal-dashboard:secret-for-internal-dashboard')
  },
  body: new URLSearchParams({
    grant_type: 'authorization_code',
    code: code,
    redirect_uri: 'http://localhost:3001/auth/callback'
  })
});

const tokens = await response.json();
```

**cURL equivalent:**
```bash
curl -X POST http://localhost:3000/api/auth/oauth2/token \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -u "internal-dashboard:secret-for-internal-dashboard" \
  -d "grant_type=authorization_code" \
  -d "code=AUTH_CODE_ABC123XYZ" \
  -d "redirect_uri=http://localhost:3001/auth/callback"
```

**Response (Tokens!):**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiJ1c2VyXzEyMyIsInNjb3BlIjoib3BlbmlkIGVtYWlsIHByb2ZpbGUiLCJpYXQiOjE3MDAwMDAwMDAsImV4cCI6MTcwMDAwMzYwMH0...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "refresh_xyz789...",
  "id_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiJ1c2VyXzEyMyIsImVtYWlsIjoidXNlckBleGFtcGxlLmNvbSIsImVtYWlsX3ZlcmlmaWVkIjp0cnVlLCJuYW1lIjoiSm9obiBEb2UiLCJpYXQiOjE3MDAwMDAwMDAsImV4cCI6MTcwMDAwMzYwMCwiYXVkIjoiaW50ZXJuYWwtZGFzaGJvYXJkIiwiaXNzIjoiaHR0cDovL2xvY2FsaG9zdDozMDAwIiwibm9uY2UiOiJyYW5kb21fbm9uY2VfNjc4OTAifQ...",
  "scope": "openid email profile"
}
```

**Token Explanation:**
- **access_token**: Use to call protected APIs and UserInfo endpoint
- **refresh_token**: Use to get new access tokens when they expire
- **id_token**: JWT containing user identity information
- **expires_in**: Access token lifetime in seconds (3600 = 1 hour)

---

### **Step 7: Client App Decodes ID Token**

**Client App** decodes the `id_token` to get user info:

```javascript
// Decode JWT (just base64 decode, no verification yet)
const idTokenPayload = JSON.parse(
  atob(tokens.id_token.split('.')[1])
);

console.log(idTokenPayload);
```

**ID Token Payload:**
```json
{
  "sub": "user_123",               // User ID (subject)
  "email": "user@example.com",
  "email_verified": true,
  "name": "John Doe",
  "picture": null,
  "iat": 1700000000,               // Issued at (Unix timestamp)
  "exp": 1700003600,               // Expires at (Unix timestamp)
  "aud": "internal-dashboard",     // Audience (client_id)
  "iss": "http://localhost:3000",  // Issuer (SSO server URL)
  "nonce": "random_nonce_67890"    // Must match the nonce sent in Step 1
}
```

**Client App MUST verify:**
1. ✅ **Signature** - Using public keys from `/api/auth/jwks`
2. ✅ **iss** (issuer) - Matches your SSO server URL
3. ✅ **aud** (audience) - Matches your client_id
4. ✅ **exp** (expiration) - Token is not expired
5. ✅ **nonce** - Matches the one sent in authorization request

**Example token verification (Node.js):**
```javascript
const jose = require('jose');

// Fetch JWKS from SSO server
const JWKS = jose.createRemoteJWKSet(
  new URL('http://localhost:3000/api/auth/jwks')
);

// Verify ID token
const { payload } = await jose.jwtVerify(tokens.id_token, JWKS, {
  issuer: 'http://localhost:3000',
  audience: 'internal-dashboard',
});

// Check nonce
if (payload.nonce !== 'random_nonce_67890') {
  throw new Error('Invalid nonce');
}

console.log('Token verified!', payload);
```

---

### **Step 8: Client App Uses Access Token (Optional)**

To get additional user info or call protected APIs:

```javascript
// Call UserInfo endpoint
const userInfo = await fetch('http://localhost:3000/api/auth/oauth2/userinfo', {
  headers: {
    'Authorization': `Bearer ${tokens.access_token}`
  }
});

const user = await userInfo.json();
```

**cURL equivalent:**
```bash
curl -X GET http://localhost:3000/api/auth/oauth2/userinfo \
  -H "Authorization: Bearer ACCESS_TOKEN_HERE"
```

**Response:**
```json
{
  "sub": "user_123",
  "email": "user@example.com",
  "email_verified": true,
  "name": "John Doe",
  "picture": null
}
```

---

### **Step 9: User is Logged In!**

**Client App:**
1. Stores tokens securely:
   - **Best practice**: httpOnly cookies (prevents XSS)
   - **Alternative**: Secure storage with encryption
2. Creates a local session for the user
3. Redirects user to dashboard/home page

```javascript
// Example: Store tokens in httpOnly cookie
res.cookie('access_token', tokens.access_token, {
  httpOnly: true,
  secure: true, // HTTPS only
  sameSite: 'strict',
  maxAge: 3600000 // 1 hour
});

res.cookie('refresh_token', tokens.refresh_token, {
  httpOnly: true,
  secure: true,
  sameSite: 'strict',
  maxAge: 2592000000 // 30 days
});

// Redirect to dashboard
res.redirect('/dashboard');
```

---

## 🔄 Token Refresh Flow

When `access_token` expires (after 1 hour), Client App uses `refresh_token` to get new tokens **without user interaction**:

```javascript
// On Client App backend
const response = await fetch('http://localhost:3000/api/auth/oauth2/token', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/x-www-form-urlencoded',
    'Authorization': 'Basic ' + btoa('internal-dashboard:secret-for-internal-dashboard')
  },
  body: new URLSearchParams({
    grant_type: 'refresh_token',
    refresh_token: savedRefreshToken
  })
});

const newTokens = await response.json();
```

**cURL equivalent:**
```bash
curl -X POST http://localhost:3000/api/auth/oauth2/token \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -u "internal-dashboard:secret-for-internal-dashboard" \
  -d "grant_type=refresh_token" \
  -d "refresh_token=REFRESH_TOKEN_HERE"
```

**Response:** New `access_token`, `refresh_token`, and `id_token`

---

## 📊 Visual Flow Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                    OIDC Authorization Code Flow                  │
└──────────────────────────────────────────────────────────────────┘

┌─────────────────┐
│   User clicks   │
│  "Login with    │
│   SSO" on       │
│  Client App     │
│ (localhost:3001)│
└────────┬────────┘
         │ 1. Browser redirect with authorization params
         │    GET /api/auth/oauth2/authorize?client_id=...
         ↓
┌─────────────────────────────┐
│    SSO Server               │
│  (localhost:3000)           │
│  Authorization Endpoint     │
│  Checks: User logged in?    │
└────────┬────────────────────┘
         │ 2. No → Redirect to login
         ↓
┌─────────────────────────────┐
│    Login Page               │
│    /auth/login              │
└────────┬────────────────────┘
         │ 3. User submits credentials
         │    POST /api/auth/sign-in/email
         ↓
┌─────────────────────────────┐
│  Authentication Successful  │
│  Session cookie set         │
│  better-auth.session_token  │
└────────┬────────────────────┘
         │ 4. Redirect back to authorize endpoint
         ↓
┌─────────────────────────────┐
│  Consent Page (optional)    │
│  User grants permissions    │
│  Scopes: openid, email,     │
│          profile            │
└────────┬────────────────────┘
         │ 5. Generate authorization code
         │    Redirect to Client App callback
         │    http://localhost:3001/auth/callback?code=ABC123&state=...
         ↓
┌─────────────────────────────┐
│  Client App Callback        │
│  (localhost:3001)           │
│  Receives: code, state      │
└────────┬────────────────────┘
         │ 6. Backend: Exchange code for tokens
         │    POST /api/auth/oauth2/token
         │    grant_type=authorization_code
         ↓
┌─────────────────────────────┐
│   SSO Server Returns        │
│   Tokens:                   │
│   - access_token            │
│   - id_token                │
│   - refresh_token           │
└────────┬────────────────────┘
         │ 7. Client App validates tokens
         │    - Verify signature (JWKS)
         │    - Check expiration
         │    - Validate claims
         ↓
┌─────────────────────────────┐
│   User Logged In!           │
│   - Tokens stored securely  │
│   - Session created         │
│   - Redirect to dashboard   │
└─────────────────────────────┘

         │ (Later, when access token expires)
         ↓
┌─────────────────────────────┐
│   Token Refresh             │
│   POST /api/auth/oauth2/    │
│        token                │
│   grant_type=refresh_token  │
└─────────────────────────────┘
```

---

## 🎯 Key Endpoints Summary

| Step | Who Calls | Endpoint | Method | Purpose | Location |
|------|-----------|----------|--------|---------|----------|
| 1 | Browser (redirect) | `/api/auth/oauth2/authorize` | GET | Start OIDC flow | SSO Server |
| 3 | Browser (form submit) | `/api/auth/sign-in/email` | POST | User login | SSO Server |
| 6 | Client Backend | `/api/auth/oauth2/token` | POST | Exchange code for tokens | SSO Server |
| 8 | Client Backend | `/api/auth/oauth2/userinfo` | GET | Get user info | SSO Server |
| Refresh | Client Backend | `/api/auth/oauth2/token` | POST | Refresh access token | SSO Server |

---

## 🔐 Security Best Practices

### 1. State Parameter (CSRF Protection)
```javascript
// Generate random state
const state = crypto.randomBytes(32).toString('hex');

// Store in session
req.session.oauth_state = state;

// Include in authorization URL
authUrl.searchParams.set('state', state);

// Verify on callback
if (req.query.state !== req.session.oauth_state) {
  throw new Error('CSRF attack detected');
}
```

### 2. Nonce Parameter (Replay Attack Protection)
```javascript
// Generate random nonce
const nonce = crypto.randomBytes(32).toString('hex');

// Store in session
req.session.oauth_nonce = nonce;

// Include in authorization URL
authUrl.searchParams.set('nonce', nonce);

// Verify in ID token
if (idToken.nonce !== req.session.oauth_nonce) {
  throw new Error('Replay attack detected');
}
```

### 3. PKCE (Proof Key for Code Exchange)
For extra security, especially for public clients:

```javascript
// Generate code verifier
const codeVerifier = crypto.randomBytes(32).toString('base64url');

// Generate code challenge
const codeChallenge = crypto
  .createHash('sha256')
  .update(codeVerifier)
  .digest('base64url');

// Add to authorization request
authUrl.searchParams.set('code_challenge', codeChallenge);
authUrl.searchParams.set('code_challenge_method', 'S256');

// Include verifier when exchanging code
body.append('code_verifier', codeVerifier);
```

### 4. Token Storage
**DO:**
- ✅ Store in httpOnly cookies
- ✅ Use secure flag (HTTPS only)
- ✅ Set appropriate expiration
- ✅ Use sameSite=strict or sameSite=lax

**DON'T:**
- ❌ Store in localStorage (vulnerable to XSS)
- ❌ Store in sessionStorage (vulnerable to XSS)
- ❌ Expose tokens in URLs
- ❌ Store refresh tokens in browser

### 5. Token Validation
**Always validate:**
- ✅ Signature using JWKS
- ✅ Issuer (iss claim)
- ✅ Audience (aud claim)
- ✅ Expiration (exp claim)
- ✅ Not before (nbf claim)
- ✅ Nonce (if used)

---

## ⚠️ Important Notes

### Token Lifetimes
- **Authorization Code**: ~10 minutes (single-use)
- **Access Token**: 1 hour
- **Refresh Token**: 30 days (can be configured)
- **ID Token**: 1 hour (same as access token)

### Client Secret Security
- **NEVER** expose client_secret in:
  - ❌ Frontend code (JavaScript)
  - ❌ Mobile apps
  - ❌ Git repositories
  - ❌ Public documentation

- **ALWAYS** keep client_secret:
  - ✅ On backend server only
  - ✅ In environment variables
  - ✅ Encrypted at rest
  - ✅ Rotated regularly

### Required Configuration

**Before testing, ensure:**

1. **Token endpoint is enabled** in `packages/auth-config/index.ts`:
   ```typescript
   // Remove or comment out:
   // disabledPaths: ['/token'],
   ```

2. **Client is registered** in your config:
   ```typescript
   trustedClients: [
     {
       clientId: 'internal-dashboard',
       clientSecret: 'secret-for-internal-dashboard',
       redirectURLs: ['http://localhost:3001/auth/callback']
     }
   ]
   ```

3. **Environment variables are set**:
   ```bash
   BETTER_AUTH_SECRET=your-secret-key
   BETTER_AUTH_URL=http://localhost:3000
   ```

---

## 🧪 Testing the Flow

### Quick Test Script

Save this as `test-oidc-flow.sh`:

```bash
#!/bin/bash

# Step 1: Generate authorization URL
CLIENT_ID="internal-dashboard"
REDIRECT_URI="http://localhost:3001/auth/callback"
STATE="test_state_123"
NONCE="test_nonce_456"

AUTH_URL="http://localhost:3000/api/auth/oauth2/authorize"
AUTH_URL+="?client_id=${CLIENT_ID}"
AUTH_URL+="&redirect_uri=${REDIRECT_URI}"
AUTH_URL+="&response_type=code"
AUTH_URL+="&scope=openid%20email%20profile"
AUTH_URL+="&state=${STATE}"
AUTH_URL+="&nonce=${NONCE}"

echo "Step 1: Open this URL in browser:"
echo "$AUTH_URL"
echo ""
echo "Step 2: Login with your credentials"
echo ""
echo "Step 3: You'll be redirected to callback URL with 'code' parameter"
echo "        Copy the code value"
echo ""
echo "Step 4: Exchange code for tokens (paste code when prompted):"
read -p "Enter authorization code: " CODE

# Exchange code for tokens
curl -X POST http://localhost:3000/api/auth/oauth2/token \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -u "internal-dashboard:secret-for-internal-dashboard" \
  -d "grant_type=authorization_code" \
  -d "code=${CODE}" \
  -d "redirect_uri=${REDIRECT_URI}"
```

Make executable and run:
```bash
chmod +x test-oidc-flow.sh
./test-oidc-flow.sh
```

---

## 📚 Additional Resources

- [OpenID Connect Specification](https://openid.net/specs/openid-connect-core-1_0.html)
- [OAuth 2.0 RFC 6749](https://datatracker.ietf.org/doc/html/rfc6749)
- [Better Auth Documentation](https://www.better-auth.com)
- [JWT.io Debugger](https://jwt.io) - Decode and verify JWTs

---

## 🆘 Troubleshooting

### Issue: 404 on token endpoint
**Solution:** Enable token endpoint in `packages/auth-config/index.ts`:
```typescript
// Remove this line:
disabledPaths: ['/token'],
```

### Issue: Invalid redirect_uri
**Solution:** Ensure redirect_uri is registered in client config:
```typescript
redirectURLs: ['http://localhost:3001/auth/callback']
```

### Issue: CORS error
**Solution:** Add client origin to trustedOrigins:
```typescript
trustedOrigins: ['http://localhost:3001']
```

### Issue: Invalid client credentials
**Solution:** Verify client_id and client_secret match config:
```bash
# Check current client credentials
echo "Client ID: internal-dashboard"
echo "Client Secret: secret-for-internal-dashboard"
```

### Issue: Token signature verification failed
**Solution:** Fetch fresh JWKS and verify issuer URL:
```bash
curl http://localhost:3000/api/auth/jwks
```

---

## 📝 Summary

The OIDC Authorization Code Flow is the **gold standard** for web application authentication because:

1. ✅ **Secure**: Client secret never exposed to browser
2. ✅ **Standard**: Works with all major identity providers
3. ✅ **User-friendly**: Single Sign-On (SSO) experience
4. ✅ **Token refresh**: Long-lived sessions without re-authentication
5. ✅ **Flexible**: Supports multiple scopes and claims

Your SSO server is now ready to authenticate users for any OIDC-compliant application! 🚀
