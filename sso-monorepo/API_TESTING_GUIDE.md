# API Testing Guide (No UI Required)

Complete guide to test all authentication flows using **only cURL and browser console**.

## 🎯 Prerequisites

1. Server running on `http://localhost:3000`
2. Resend API key added to `.env`
3. Your real email address for testing

## 📧 Flow 1: Email Verification (Complete Test)

### Step 1: Sign Up New User

```bash
curl -X POST http://localhost:3000/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{
    "email": "YOUR_EMAIL@gmail.com",
    "password": "TestPass123!",
    "name": "Test User"
  }' | jq
```

**Expected Response:**
```json
{
  "token": "some_session_token",
  "user": {
    "id": "user_abc123",
    "email": "YOUR_EMAIL@gmail.com",
    "name": "Test User",
    "emailVerified": false,  ← Not verified yet
    "createdAt": "2025-11-23T..."
  }
}
```

**What happened:**
- ✅ User created in database
- ✅ Verification email sent to your inbox
- ✅ Session created (user is logged in)
- ⚠️ Email not verified yet

### Step 2: Check Your Email

1. Open your email inbox
2. Look for email with subject: "Verify your email for SSO Platform"
3. You'll see a beautiful email with a "Verify Email Address" button
4. **Right-click the button** → Copy Link Address
5. The link looks like: `http://localhost:3000/api/auth/verify-email?token=abc123&callbackURL=/`

### Step 3: Verify Email (Using cURL)

Extract the token from the email link and run:

```bash
# Replace TOKEN with the actual token from your email
curl -X GET "http://localhost:3000/api/auth/verify-email?token=TOKEN_FROM_EMAIL&callbackURL=/" \
  -v
```

**Expected Response:**
```
HTTP/1.1 302 Found
Location: /?verified=true
Set-Cookie: better-auth.session_token=...
```

**What happened:**
- ✅ Email marked as verified
- ✅ User redirected to callback URL
- ✅ New session created

### Step 4: Verify It Worked - Check User Status

```bash
# Get session info (use the session token from sign-up response)
curl -X GET http://localhost:3000/api/auth/session \
  -H "Cookie: better-auth.session_token=YOUR_SESSION_TOKEN" \
  | jq
```

**Expected Response:**
```json
{
  "session": {
    "id": "session_xyz",
    "userId": "user_abc123",
    "expiresAt": "..."
  },
  "user": {
    "id": "user_abc123",
    "email": "YOUR_EMAIL@gmail.com",
    "emailVerified": true  ← NOW VERIFIED!
  }
}
```

### Step 5: Test Resending Verification Email

```bash
curl -X POST http://localhost:3000/api/auth/send-verification-email \
  -H "Content-Type: application/json" \
  -d '{
    "email": "YOUR_EMAIL@gmail.com",
    "callbackURL": "/"
  }' | jq
```

**Expected:**
- You'll receive another verification email
- Useful if user didn't receive the first one

---

## 🔐 Flow 2: Password Reset (Complete Test)

### Step 1: Request Password Reset

```bash
curl -X POST http://localhost:3000/api/auth/forget-password \
  -H "Content-Type: application/json" \
  -d '{
    "email": "YOUR_EMAIL@gmail.com",
    "redirectTo": "http://localhost:3001/reset-password"
  }' | jq
```

**Expected Response:**
```json
{
  "success": true,
  "message": "Password reset email sent"
}
```

**What happened:**
- ✅ Password reset email sent
- ✅ Token generated (expires in 1 hour)

### Step 2: Check Your Email

1. Open your email inbox
2. Look for email with subject: "Reset your password for SSO Platform"
3. You'll see a red-themed email with "Reset Password" button
4. **Right-click the button** → Copy Link Address
5. The link looks like: `http://localhost:3001/reset-password?token=xyz789`

### Step 3: Extract Token from Email Link

From the link `http://localhost:3001/reset-password?token=xyz789`, extract: `xyz789`

### Step 4: Reset Password (Using cURL)

```bash
# Replace TOKEN with the actual token from your email
curl -X POST http://localhost:3000/api/auth/reset-password \
  -H "Content-Type: application/json" \
  -d '{
    "newPassword": "NewSecurePass123!",
    "token": "TOKEN_FROM_EMAIL"
  }' | jq
```

**Expected Response:**
```json
{
  "success": true,
  "message": "Password successfully reset"
}
```

### Step 5: Test New Password - Sign In

```bash
curl -X POST http://localhost:3000/api/auth/sign-in/email \
  -H "Content-Type: application/json" \
  -d '{
    "email": "YOUR_EMAIL@gmail.com",
    "password": "NewSecurePass123!"
  }' | jq
```

**Expected:**
```json
{
  "token": "new_session_token",
  "user": {
    "email": "YOUR_EMAIL@gmail.com",
    "emailVerified": true
  }
}
```

✅ **Success!** Login works with new password!

### Step 6: Verify Old Password No Longer Works

```bash
curl -X POST http://localhost:3000/api/auth/sign-in/email \
  -H "Content-Type: application/json" \
  -d '{
    "email": "YOUR_EMAIL@gmail.com",
    "password": "TestPass123!"
  }' | jq
```

**Expected:**
```json
{
  "error": "Invalid email or password"
}
```

✅ Old password rejected!

---

## 🧪 Flow 3: Testing with Browser Console

Open your browser at `http://localhost:3001` and use the console:

### Test Sign Up + Verification

```javascript
// Step 1: Sign up
const { authClient } = await import('@repo/auth-config/client');

const result = await authClient.signUp.email({
  email: "YOUR_EMAIL@gmail.com",
  password: "TestPass123!",
  name: "Console Test User"
});

console.log('Sign up result:', result);
// Check your email for verification link

// Step 2: After clicking email link, check session
const session = await authClient.getSession();
console.log('Current session:', session);
console.log('Email verified:', session.user?.emailVerified);
```

### Test Password Reset

```javascript
// Step 1: Request reset
await authClient.forgetPassword({
  email: "YOUR_EMAIL@gmail.com",
  redirectTo: "http://localhost:3001/reset"
});
console.log('Check your email!');

// Step 2: After getting token from email, reset password
// (You'll need to manually extract token from email link)
await authClient.resetPassword({
  newPassword: "NewPassword123!",
  token: "TOKEN_FROM_EMAIL"
});

// Step 3: Test new password
await authClient.signIn.email({
  email: "YOUR_EMAIL@gmail.com",
  password: "NewPassword123!"
});

const session = await authClient.getSession();
console.log('Logged in:', session.user);
```

---

## 🔍 Flow 4: Testing Error Cases

### Test 1: Expired Token

Wait 1 hour after password reset email, then try to reset:

```bash
curl -X POST http://localhost:3000/api/auth/reset-password \
  -H "Content-Type: application/json" \
  -d '{
    "newPassword": "SomePassword123!",
    "token": "EXPIRED_TOKEN"
  }' | jq
```

**Expected:**
```json
{
  "error": "Invalid or expired token"
}
```

### Test 2: Invalid Token

```bash
curl -X POST http://localhost:3000/api/auth/reset-password \
  -H "Content-Type: application/json" \
  -d '{
    "newPassword": "SomePassword123!",
    "token": "COMPLETELY_FAKE_TOKEN"
  }' | jq
```

**Expected:**
```json
{
  "error": "Invalid or expired token"
}
```

### Test 3: Already Verified Email

Try verifying an already-verified email:

```bash
curl -X GET "http://localhost:3000/api/auth/verify-email?token=OLD_USED_TOKEN&callbackURL=/" \
  -v
```

**Expected:**
```json
{
  "error": "Token already used or expired"
}
```

---

## 🔗 Flow 5: GitHub & Google Social Login (OAuth)

### Overview

BetterAuth supports OAuth social login providers like GitHub and Google. This section covers:
- Setting up OAuth apps in GitHub/Google
- Configuring BetterAuth with credentials
- Testing social login flows via API

### Part A: GitHub OAuth Setup

#### Step 1: Create GitHub OAuth App

1. Go to GitHub Settings → Developer settings → OAuth Apps
2. Click "New OAuth App"
3. Fill in the form:
   - **Application name**: `SSO Platform` (or your app name)
   - **Homepage URL**: `http://localhost:3000`
   - **Authorization callback URL**: `http://localhost:3000/api/auth/callback/github`
4. Click "Register application"
5. You'll receive:
   - **Client ID**: `Iv1.abc123...`
   - Click "Generate a new client secret" to get **Client Secret**: `abc123xyz...`

#### Step 2: Configure BetterAuth with GitHub

Add to your `.env` file:

```bash
GITHUB_CLIENT_ID=Iv1.abc123...
GITHUB_CLIENT_SECRET=abc123xyz...
```

Add to your BetterAuth configuration (server-side):

```typescript
// packages/auth-config/src/index.ts
import { betterAuth } from "better-auth";

export const auth = betterAuth({
  // ... existing config
  socialProviders: {
    github: {
      clientId: process.env.GITHUB_CLIENT_ID as string,
      clientSecret: process.env.GITHUB_CLIENT_SECRET as string,
      // Optional: custom redirect URI
      // redirectURI: "http://localhost:3000/api/auth/callback/github"
    },
  },
});
```

#### Step 3: Test GitHub OAuth Flow

**Option 1: Via Browser (Recommended for OAuth)**

1. Navigate to:
```
http://localhost:3000/api/auth/signin/github?callbackURL=/dashboard
```

2. You'll be redirected to GitHub's authorization page
3. Click "Authorize" to grant permissions
4. GitHub redirects back to your app at the callback URL
5. BetterAuth creates/updates user and session

**Option 2: Via API Client (Programmatic)**

```typescript
// Client-side code
import { authClient } from "@repo/auth-config/client";

const signInWithGitHub = async () => {
  const { data, error } = await authClient.signIn.social({
    provider: "github",
    callbackURL: "/dashboard",
  });

  if (error) {
    console.error("GitHub sign-in failed:", error);
  } else {
    console.log("Signed in:", data.user);
  }
};
```

**Option 3: Test via cURL (Get Authorization URL)**

```bash
# This will return a redirect to GitHub's OAuth page
curl -X GET "http://localhost:3000/api/auth/signin/github?callbackURL=/dashboard" \
  -L -v
```

#### Step 4: Verify GitHub User Creation

After successful GitHub OAuth:

```bash
curl -X GET http://localhost:3000/api/auth/session \
  -H "Cookie: better-auth.session_token=YOUR_SESSION_TOKEN" \
  | jq
```

**Expected Response:**
```json
{
  "user": {
    "id": "user_github123",
    "email": "user@github.com",
    "name": "GitHub Username",
    "image": "https://avatars.githubusercontent.com/...",
    "emailVerified": true  // Auto-verified via GitHub
  },
  "session": {
    "id": "session_xyz",
    "userId": "user_github123"
  }
}
```

**Check linked accounts:**

```bash
curl -X GET http://localhost:3000/api/auth/list-accounts \
  -H "Cookie: better-auth.session_token=YOUR_SESSION_TOKEN" \
  | jq
```

**Expected Response:**
```json
{
  "accounts": [
    {
      "id": "acc_123",
      "provider": "github",
      "providerId": "12345678",  // GitHub user ID
      "userId": "user_github123"
    }
  ]
}
```

---

### Part B: Google OAuth Setup

#### Step 1: Create Google OAuth App

1. Go to [Google Cloud Console](https://console.cloud.google.com/)
2. Create a new project or select existing one
3. Enable "Google+ API" (in APIs & Services → Library)
4. Go to "Credentials" → Click "Create Credentials" → "OAuth client ID"
5. Configure OAuth consent screen first (if not done):
   - User Type: External
   - App name: `SSO Platform`
   - User support email: your email
   - Authorized domains: `localhost` (for testing)
   - Developer contact: your email
6. Create OAuth Client ID:
   - Application type: **Web application**
   - Name: `SSO Platform Web Client`
   - Authorized JavaScript origins: `http://localhost:3000`
   - Authorized redirect URIs: `http://localhost:3000/api/auth/callback/google`
7. You'll receive:
   - **Client ID**: `123456789-abc.apps.googleusercontent.com`
   - **Client Secret**: `GOCSPX-abc123xyz...`

#### Step 2: Configure BetterAuth with Google

Add to your `.env` file:

```bash
GOOGLE_CLIENT_ID=123456789-abc.apps.googleusercontent.com
GOOGLE_CLIENT_SECRET=GOCSPX-abc123xyz...
```

Add to your BetterAuth configuration:

```typescript
// packages/auth-config/src/index.ts
import { betterAuth } from "better-auth";

export const auth = betterAuth({
  // ... existing config
  socialProviders: {
    github: {
      // ... existing GitHub config
    },
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID as string,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET as string,
      // Optional: custom redirect URI
      // redirectURI: "http://localhost:3000/api/auth/callback/google"
    },
  },
});
```

#### Step 3: Test Google OAuth Flow

**Via Browser (Recommended):**

1. Navigate to:
```
http://localhost:3000/api/auth/signin/google?callbackURL=/dashboard
```

2. You'll be redirected to Google's sign-in page
3. Select your Google account
4. Click "Allow" to grant permissions
5. Google redirects back to your app
6. BetterAuth creates/updates user and session

**Via API Client:**

```typescript
// Client-side code
import { authClient } from "@repo/auth-config/client";

const signInWithGoogle = async () => {
  const { data, error } = await authClient.signIn.social({
    provider: "google",
    callbackURL: "/dashboard",
  });

  if (error) {
    console.error("Google sign-in failed:", error);
  } else {
    console.log("Signed in:", data.user);
  }
};
```

#### Step 4: Verify Google User Creation

```bash
curl -X GET http://localhost:3000/api/auth/session \
  -H "Cookie: better-auth.session_token=YOUR_SESSION_TOKEN" \
  | jq
```

**Expected Response:**
```json
{
  "user": {
    "id": "user_google456",
    "email": "user@gmail.com",
    "name": "John Doe",
    "image": "https://lh3.googleusercontent.com/...",
    "emailVerified": true  // Auto-verified via Google
  }
}
```

---

### Part C: Linking Multiple Social Accounts

Users can link multiple social accounts (GitHub + Google) to one account.

#### Link Additional Account (When Already Logged In)

```typescript
// Client-side: User is logged in with email, now linking GitHub
const linkGitHub = async () => {
  const { data, error } = await authClient.linkSocial({
    provider: "github",
    callbackURL: "/settings/accounts",
  });
};

// Or link Google
const linkGoogle = async () => {
  const { data, error } = await authClient.linkSocial({
    provider: "google",
    callbackURL: "/settings/accounts",
  });
};
```

#### List All Linked Accounts

```bash
curl -X GET http://localhost:3000/api/auth/list-accounts \
  -H "Cookie: better-auth.session_token=YOUR_SESSION_TOKEN" \
  | jq
```

**Expected Response:**
```json
{
  "accounts": [
    {
      "id": "acc_email",
      "provider": "credential",  // Email/password account
      "userId": "user_123"
    },
    {
      "id": "acc_gh",
      "provider": "github",
      "providerId": "87654321",
      "userId": "user_123"
    },
    {
      "id": "acc_google",
      "provider": "google",
      "providerId": "109876543210",
      "userId": "user_123"
    }
  ]
}
```

#### Unlink Social Account

```typescript
// Client-side: Unlink GitHub account
const unlinkGitHub = async () => {
  await authClient.unlinkAccount({
    accountId: "acc_gh",  // ID from list-accounts
  });
};
```

Or via API:

```bash
curl -X POST http://localhost:3000/api/auth/unlink-account \
  -H "Content-Type: application/json" \
  -H "Cookie: better-auth.session_token=YOUR_SESSION_TOKEN" \
  -d '{
    "accountId": "acc_gh"
  }' | jq
```

---

### Part D: Environment Variables Summary

Complete `.env` file for all auth methods:

```bash
# Database
DATABASE_URL=your-database-url

# Email (Resend)
RESEND_API_KEY=re_...
RESEND_FROM_EMAIL=noreply@yourdomain.com

# GitHub OAuth
GITHUB_CLIENT_ID=Iv1.abc123...
GITHUB_CLIENT_SECRET=abc123xyz...

# Google OAuth
GOOGLE_CLIENT_ID=123456789-abc.apps.googleusercontent.com
GOOGLE_CLIENT_SECRET=GOCSPX-abc123xyz...

# BetterAuth Secret (for session signing)
BETTER_AUTH_SECRET=your-secret-key-here
BETTER_AUTH_URL=http://localhost:3000
```

---

### Part E: Testing Complete OAuth Integration

#### Test Scenario 1: Sign up with email, then link social accounts

```bash
# 1. Sign up with email
curl -X POST http://localhost:3000/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "Pass123!",
    "name": "Test User"
  }' | jq

# Save the session token from response

# 2. Link GitHub (must be done via browser)
# Navigate to: http://localhost:3000/api/auth/link/github?callbackURL=/settings

# 3. Link Google (must be done via browser)
# Navigate to: http://localhost:3000/api/auth/link/google?callbackURL=/settings

# 4. Verify all accounts linked
curl -X GET http://localhost:3000/api/auth/list-accounts \
  -H "Cookie: better-auth.session_token=SESSION_TOKEN" | jq
```

#### Test Scenario 2: Sign in with GitHub, verify auto-created user

```bash
# 1. Sign in with GitHub (via browser)
# Navigate to: http://localhost:3000/api/auth/signin/github?callbackURL=/

# 2. After redirect, check session
curl -X GET http://localhost:3000/api/auth/session \
  -H "Cookie: better-auth.session_token=SESSION_TOKEN" | jq

# 3. Verify email is auto-verified
# emailVerified should be true
```

#### Test Scenario 3: Sign in with Google, then add password

```bash
# 1. Sign in with Google (via browser)
# Navigate to: http://localhost:3000/api/auth/signin/google?callbackURL=/

# 2. User now wants to add password to their account
# (This requires implementing a "set password" endpoint)
```

---

### Part F: Common OAuth Issues & Troubleshooting

#### Issue 1: "Redirect URI mismatch"

**Cause:** The callback URL in your OAuth app doesn't match BetterAuth's callback URL.

**Fix:**
- GitHub: Must be exactly `http://localhost:3000/api/auth/callback/github`
- Google: Must be exactly `http://localhost:3000/api/auth/callback/google`
- No trailing slashes
- Check port number matches

#### Issue 2: "Invalid client credentials"

**Cause:** Wrong Client ID or Client Secret in `.env`

**Fix:**
```bash
# Double-check your .env file
cat .env | grep CLIENT_ID
cat .env | grep CLIENT_SECRET

# Restart server after changing .env
```

#### Issue 3: User signs in with both email and Google using same email

**Behavior:** BetterAuth automatically links accounts if emails match.

**Verify:**
```bash
curl -X GET http://localhost:3000/api/auth/list-accounts \
  -H "Cookie: better-auth.session_token=TOKEN" | jq
```

You should see both `credential` and `google` providers for the same `userId`.

#### Issue 4: OAuth works in development but not production

**Checklist:**
- Update OAuth app callback URLs to production domain
- Update `BETTER_AUTH_URL` in production `.env`
- Ensure HTTPS is enabled in production
- Check CORS settings if using separate frontend domain

---

### Part G: OAuth Flow Diagram (For Reference)

```
User clicks "Sign in with GitHub"
    ↓
Browser redirects to GitHub: https://github.com/login/oauth/authorize?client_id=...
    ↓
User authorizes app on GitHub
    ↓
GitHub redirects to: http://localhost:3000/api/auth/callback/github?code=abc123
    ↓
BetterAuth exchanges code for access token (server-side)
    ↓
BetterAuth fetches user info from GitHub API
    ↓
BetterAuth creates/updates user in database
    ↓
BetterAuth creates session and sets cookie
    ↓
User is redirected to callbackURL (e.g., /dashboard)
```

---

### Part H: Available OAuth Endpoints

```bash
# Social Sign-In (redirects to OAuth provider)
GET  /api/auth/signin/github?callbackURL=/dashboard
GET  /api/auth/signin/google?callbackURL=/dashboard

# OAuth Callbacks (handled by BetterAuth automatically)
GET  /api/auth/callback/github?code=...
GET  /api/auth/callback/google?code=...

# Link Social Account (requires existing session)
GET  /api/auth/link/github?callbackURL=/settings
GET  /api/auth/link/google?callbackURL=/settings

# Unlink Social Account
POST /api/auth/unlink-account
Body: { "accountId": "acc_123" }

# List Linked Accounts
GET  /api/auth/list-accounts
```

---

## 🔐 Flow 6: OIDC Provider Endpoints (OpenID Connect)

Your SSO server is configured as an **OpenID Connect (OIDC) Provider**, allowing other applications to use it for authentication. This section covers testing the OIDC endpoints.

### OIDC Architecture Overview

```
Client Application (e.g., Dashboard)
    ↓ (1) Authorization Request
SSO Server (OIDC Provider) - http://localhost:3000
    ↓ (2) User Login + Consent
    ↓ (3) Authorization Code
Client Application
    ↓ (4) Exchange Code for Tokens
SSO Server
    ↓ (5) ID Token + Access Token
Client Application (User Authenticated)
```

### Current OIDC Configuration

Based on your `packages/auth-config/index.ts`:

- **JWT Signing**: RS256 (Asymmetric)
- **Login Page**: `/auth/login`
- **Consent Page**: `/auth/consent`
- **Dynamic Client Registration**: Enabled
- **Trusted Client**: `internal-dashboard`
- **Token Endpoint**: Disabled (line 41)

---

### Part A: OIDC Discovery Endpoint

The discovery endpoint returns all available OIDC endpoints and supported features.

#### Test Discovery Endpoint

```bash
curl -X GET http://localhost:3000/api/auth/.well-known/openid-configuration | jq
```

**Note:** All OIDC endpoints are under `/api/auth/` because that's Better Auth's basePath.

**Expected Response:**

```json
{
  "issuer": "http://localhost:3000",
  "authorization_endpoint": "http://localhost:3000/api/auth/oauth2/authorize",
  "token_endpoint": "http://localhost:3000/api/auth/oauth2/token",
  "userinfo_endpoint": "http://localhost:3000/api/auth/oauth2/userinfo",
  "jwks_uri": "http://localhost:3000/api/auth/jwks",
  "registration_endpoint": "http://localhost:3000/api/auth/oauth2/register",
  "response_types_supported": ["code"],
  "response_modes_supported": ["query"],
  "grant_types_supported": ["authorization_code", "refresh_token"],
  "subject_types_supported": ["public"],
  "id_token_signing_alg_values_supported": ["RS256", "EdDSA"],
  "scopes_supported": ["openid", "profile", "email", "offline_access"],
  "token_endpoint_auth_methods_supported": ["client_secret_basic", "client_secret_post", "none"],
  "code_challenge_methods_supported": ["S256"],
  "claims_supported": ["sub", "iss", "aud", "exp", "nbf", "iat", "jti", "email", "email_verified", "name"]
}
```

**What it tells you:**
- All available OIDC endpoints
- Supported authentication flows
- Supported scopes and claims
- Signing algorithms

---

### Part B: JWKS Endpoint (Public Keys)

The JWKS endpoint exposes public keys used to verify ID tokens signed by your SSO server.

#### Test JWKS Endpoint

```bash
curl -X GET http://localhost:3000/api/auth/jwks | jq
```

**Expected Response:**

```json
{
  "keys": [
    {
      "kty": "RSA",
      "use": "sig",
      "kid": "key-id-123",
      "n": "modulus...",
      "e": "AQAB",
      "alg": "RS256"
    }
  ]
}
```

**What this is used for:**
- Client applications fetch these public keys to verify ID tokens
- Ensures tokens were actually issued by your SSO server
- Rotates automatically when keys change

---

### Part C: Dynamic Client Registration

Register a new OIDC client application dynamically.

#### Register New OIDC Client

```bash
curl -X POST http://localhost:3000/api/auth/oauth2/register \
  -H "Content-Type: application/json" \
  -d '{
    "client_name": "My Test App",
    "redirect_uris": ["http://localhost:4000/callback"],
    "grant_types": ["authorization_code", "refresh_token"],
    "response_types": ["code"],
    "scope": "openid email profile"
  }' | jq
```

**Expected Response:**

```json
{
  "client_id": "generated_client_id_abc123",
  "client_secret": "generated_secret_xyz789",
  "client_name": "My Test App",
  "redirect_uris": ["http://localhost:4000/callback"],
  "grant_types": ["authorization_code", "refresh_token"],
  "response_types": ["code"],
  "token_endpoint_auth_method": "client_secret_basic",
  "registration_access_token": "access_token_for_updates",
  "registration_client_uri": "http://localhost:3000/api/auth/oauth2/register/generated_client_id_abc123"
}
```

**Save these values:**
- `client_id` - Use for authorization requests
- `client_secret` - Use for token exchange
- `registration_access_token` - Use to update client config later

---

### Part D: Authorization Code Flow (Complete OIDC Flow)

This is the most common OIDC authentication flow.

#### Step 1: Create Authorization URL

```bash
# Build the authorization URL
CLIENT_ID="internal-dashboard"  # Or use dynamically registered client_id
REDIRECT_URI="http://localhost:3001/auth/callback"
STATE="random_state_string_12345"
NONCE="random_nonce_67890"

echo "http://localhost:3000/api/auth/oauth2/authorize?client_id=${CLIENT_ID}&redirect_uri=${REDIRECT_URI}&response_type=code&scope=openid%20email%20profile&state=${STATE}&nonce=${NONCE}"
```

#### Step 2: Open URL in Browser

Copy the generated URL and open it in your browser. You'll be:

1. **Redirected to login page** (if not logged in)
   - URL: `http://localhost:3000/auth/login?...`
   - Log in using email/password or social login

2. **Redirected to consent page** (if consent required)
   - URL: `http://localhost:3000/auth/consent?...`
   - Grant permissions for scopes (openid, email, profile)

3. **Redirected back to your callback URL with authorization code**
   - Example: `http://localhost:3001/auth/callback?code=AUTH_CODE_HERE&state=random_state_string_12345`

#### Step 3: Extract Authorization Code

From the redirect URL, extract the `code` parameter:

```bash
CODE="AUTH_CODE_FROM_REDIRECT"
```

#### Step 4: Exchange Code for Tokens

**⚠️ IMPORTANT**: Your config has `/token` endpoint disabled (line 41 in `index.ts`). You need to enable it first:

To enable the token endpoint, remove or comment out line 41:
```typescript
// disabledPaths: ['/token'],  // Comment this out
```

Then exchange the code:

```bash
curl -X POST http://localhost:3000/api/auth/oauth2/token \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -u "internal-dashboard:secret-for-internal-dashboard" \
  -d "grant_type=authorization_code" \
  -d "code=${CODE}" \
  -d "redirect_uri=http://localhost:3001/auth/callback" | jq
```

**Expected Response:**

```json
{
  "access_token": "access_token_abc123...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "refresh_token_xyz789...",
  "id_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "scope": "openid email profile"
}
```

**Token Breakdown:**
- `access_token`: Use to call UserInfo endpoint or APIs
- `id_token`: JWT containing user identity (verify with JWKS)
- `refresh_token`: Use to get new tokens when access_token expires

#### Step 5: Decode and Verify ID Token

The `id_token` is a JWT. Decode it to see user information:

**Online Tool:** Copy the `id_token` to https://jwt.io

**Expected Payload:**

```json
{
  "sub": "user_abc123",
  "email": "user@example.com",
  "email_verified": true,
  "name": "Test User",
  "picture": null,
  "iat": 1700000000,
  "exp": 1700003600,
  "aud": "internal-dashboard",
  "iss": "http://localhost:3000",
  "nonce": "random_nonce_67890"
}
```

**Verification:**
1. Verify `iss` matches your SSO server URL
2. Verify `aud` matches your `client_id`
3. Verify `exp` (expiration) is in the future
4. Verify `nonce` matches what you sent
5. Verify signature using public key from JWKS endpoint

#### Step 6: Call UserInfo Endpoint

Use the `access_token` to get user information:

```bash
curl -X GET http://localhost:3000/api/auth/oauth2/userinfo \
  -H "Authorization: Bearer ACCESS_TOKEN_HERE" | jq
```

**Expected Response:**

```json
{
  "sub": "user_abc123",
  "email": "user@example.com",
  "email_verified": true,
  "name": "Test User",
  "picture": null
}
```

---

### Part E: Refresh Token Flow

When your `access_token` expires, use the `refresh_token` to get new tokens without requiring user login.

```bash
curl -X POST http://localhost:3000/api/auth/oauth2/token \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -u "internal-dashboard:secret-for-internal-dashboard" \
  -d "grant_type=refresh_token" \
  -d "refresh_token=REFRESH_TOKEN_HERE" | jq
```

**Expected Response:**

```json
{
  "access_token": "new_access_token_def456...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "new_refresh_token_uvw321...",
  "id_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "scope": "openid email profile"
}
```

---

### Part F: Client Credentials Flow (Machine-to-Machine)

For server-to-server authentication without a user.

```bash
curl -X POST http://localhost:3000/api/auth/oauth2/token \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -u "internal-dashboard:secret-for-internal-dashboard" \
  -d "grant_type=client_credentials" \
  -d "scope=api:read api:write" | jq
```

**Expected Response:**

```json
{
  "access_token": "machine_token_ghi789...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "api:read api:write"
}
```

**Note:** No `id_token` or `refresh_token` since there's no user context.

---

### Part G: OIDC Scopes and Claims

#### Standard OIDC Scopes

| Scope | Claims Returned | Description |
|-------|----------------|-------------|
| `openid` | `sub` | Required for OIDC; returns user ID |
| `email` | `email`, `email_verified` | User's email address |
| `profile` | `name`, `picture`, etc. | User's profile information |
| `offline_access` | N/A | Request refresh token |

#### Example: Request Only Email

```bash
# Only request email scope (no profile)
curl "http://localhost:3000/api/auth/oauth2/authorize?client_id=internal-dashboard&redirect_uri=http://localhost:3001/auth/callback&response_type=code&scope=openid%20email&state=state123"
```

The resulting ID token will only contain:
```json
{
  "sub": "user_123",
  "email": "user@example.com",
  "email_verified": true
}
```

---

### Part H: Testing with Trusted Client (Skip Consent)

Your config has a trusted client: `internal-dashboard` with `skipConsent: true`.

```bash
# This client skips the consent screen
curl "http://localhost:3000/api/auth/oauth2/authorize?client_id=internal-dashboard&redirect_uri=http://localhost:3001/auth/callback&response_type=code&scope=openid%20email%20profile&state=xyz"
```

**Flow:**
1. User logs in (if needed)
2. **Consent screen is skipped** ✅
3. Immediately redirected with authorization code

---

### Part I: Update Client Configuration

If you saved the `registration_access_token` from registration, you can update the client:

```bash
curl -X PUT http://localhost:3000/api/auth/oauth2/register/YOUR_CLIENT_ID \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer REGISTRATION_ACCESS_TOKEN" \
  -d '{
    "client_id": "YOUR_CLIENT_ID",
    "client_name": "Updated App Name",
    "redirect_uris": [
      "http://localhost:4000/callback",
      "http://localhost:5000/callback"
    ]
  }' | jq
```

---

### Part J: OIDC Logout / End Session

End the user's SSO session:

```bash
# Via browser redirect
curl -L "http://localhost:3000/api/auth/oauth2/end-session?id_token_hint=ID_TOKEN&post_logout_redirect_uri=http://localhost:3001/"
```

**Parameters:**
- `id_token_hint`: The ID token from login (optional but recommended)
- `post_logout_redirect_uri`: Where to redirect after logout

---

### Part K: Common OIDC Testing Scenarios

#### Scenario 1: Multi-App SSO (Single Sign-On)

Test that logging into one app automatically logs you into another:

```bash
# 1. Login to App A (localhost:3001)
# Open: http://localhost:3000/api/auth/oauth2/authorize?client_id=internal-dashboard&redirect_uri=http://localhost:3001/auth/callback&response_type=code&scope=openid&state=a

# User logs in, gets redirected with code

# 2. Immediately login to App B (localhost:3002) WITHOUT entering credentials
# Open: http://localhost:3000/api/auth/oauth2/authorize?client_id=another-client&redirect_uri=http://localhost:3002/auth/callback&response_type=code&scope=openid&state=b

# Should get code immediately without login screen! ✅
```

#### Scenario 2: Token Verification

Verify an ID token's signature using JWKS:

```bash
# 1. Get JWKS
curl http://localhost:3000/api/auth/jwks | jq > jwks.json

# 2. Use a JWT library to verify the token signature
# Example in Node.js:
node -e "
const jose = require('jose');
const fs = require('fs');
const jwks = JSON.parse(fs.readFileSync('jwks.json'));
const token = 'YOUR_ID_TOKEN';
// Verify token using jwks
"
```

#### Scenario 3: Expired Token Handling

```bash
# Wait for access_token to expire (default: 1 hour)
# Try calling UserInfo with expired token

curl -X GET http://localhost:3000/api/auth/oauth2/userinfo \
  -H "Authorization: Bearer EXPIRED_TOKEN" | jq

# Expected: 401 Unauthorized
# Solution: Use refresh_token to get new access_token
```

---

### Part L: Environment Variables for OIDC

Update your `.env` file:

```bash
# Required for OIDC Provider
BETTER_AUTH_SECRET=your-long-random-secret-here
BETTER_AUTH_URL=http://localhost:3000

# Trusted Client Credentials (optional, defaults provided)
INTERNAL_CLIENT_ID=internal-dashboard
INTERNAL_CLIENT_SECRET=secret-for-internal-dashboard

# Production URLs (optional)
CLIENT_PRODUCTION_URL=https://client.yourdomain.com
ADMIN_PRODUCTION_URL=https://admin.yourdomain.com
```

---

### Part M: Enable Token Endpoint

**Current Issue:** The `/token` endpoint is disabled in your config (line 41).

To enable OIDC token exchange, update `packages/auth-config/index.ts`:

```typescript
export const auth = betterAuth({
  // ... other config

  // Option 1: Remove this line entirely
  // disabledPaths: ['/token'],

  // Option 2: Comment it out
  // disabledPaths: ['/token'],

  // Option 3: Use different paths to disable
  disabledPaths: [],

  // ... rest of config
});
```

**After making this change:**
1. Restart your server
2. Test the token endpoint works:

```bash
curl -X POST http://localhost:3000/api/auth/oauth2/token \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -u "internal-dashboard:secret-for-internal-dashboard" \
  -d "grant_type=client_credentials" | jq

# Should return tokens, not 404
```

---

### Part N: All OIDC Endpoints Summary

```bash
# Discovery & Metadata
GET  /api/auth/.well-known/openid-configuration   # OIDC discovery endpoint
GET  /api/auth/jwks                               # Public keys for token verification

# Client Management
POST /api/auth/oauth2/register                    # Register new OIDC client
PUT  /api/auth/oauth2/register/:client_id         # Update client configuration
GET  /api/auth/oauth2/register/:client_id         # Get client configuration

# Authentication Flow
GET  /api/auth/oauth2/authorize                   # Authorization endpoint (start login)
POST /api/auth/oauth2/token                       # Token endpoint (exchange code for tokens)
GET  /api/auth/oauth2/userinfo                    # Get user info with access token

# Session Management
GET  /api/auth/oauth2/end-session                 # Logout / end session

# Better Auth Endpoints (still available)
POST /api/auth/sign-up/email             # Create account
POST /api/auth/sign-in/email             # Email login
GET  /api/auth/session                   # Get current session
```

---

## 📊 Quick Testing Cheat Sheet

### All Available Endpoints

```bash
# User Management
POST /api/auth/sign-up/email              # Create user + send verification
POST /api/auth/sign-in/email              # Login
POST /api/auth/sign-out                   # Logout
GET  /api/auth/session                    # Get current session

# Email Verification
POST /api/auth/send-verification-email    # Manually send verification
GET  /api/auth/verify-email               # Verify with token

# Password Reset
POST /api/auth/forget-password            # Request reset
POST /api/auth/reset-password             # Reset with token

# OAuth Social Login
GET  /api/auth/signin/github              # Sign in with GitHub
GET  /api/auth/signin/google              # Sign in with Google
GET  /api/auth/callback/github            # GitHub OAuth callback
GET  /api/auth/callback/google            # Google OAuth callback

# Account Linking
GET  /api/auth/link/github                # Link GitHub to existing account
GET  /api/auth/link/google                # Link Google to existing account
POST /api/auth/unlink-account             # Unlink social account
GET  /api/auth/list-accounts              # List all linked accounts

# OIDC Provider (SSO Server)
GET  /api/auth/.well-known/openid-configuration    # OIDC discovery endpoint
GET  /api/auth/jwks                                # Public keys for token verification
GET  /api/auth/oauth2/authorize                    # Authorization endpoint (start OIDC flow)
POST /api/auth/oauth2/token                        # Token endpoint (⚠️ currently disabled)
GET  /api/auth/oauth2/userinfo                     # Get user info with access token
POST /api/auth/oauth2/register                     # Register new OIDC client
PUT  /api/auth/oauth2/register/:client_id          # Update OIDC client
GET  /api/auth/oauth2/end-session                  # OIDC logout
```

### Example: Complete Test Flow (Copy & Paste)

```bash
# 1. Sign up
curl -X POST http://localhost:3000/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"Pass123!","name":"Tester"}' | jq

# 2. Check email, copy verification token

# 3. Verify email
curl -X GET "http://localhost:3000/api/auth/verify-email?token=YOUR_TOKEN&callbackURL=/" -v

# 4. Request password reset
curl -X POST http://localhost:3000/api/auth/forget-password \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","redirectTo":"http://localhost:3001/reset"}' | jq

# 5. Check email, copy reset token

# 6. Reset password
curl -X POST http://localhost:3000/api/auth/reset-password \
  -H "Content-Type: application/json" \
  -d '{"newPassword":"NewPass123!","token":"YOUR_RESET_TOKEN"}' | jq

# 7. Login with new password
curl -X POST http://localhost:3000/api/auth/sign-in/email \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"NewPass123!"}' | jq
```

---

## 🐛 Troubleshooting

### Email Not Arriving

```bash
# Check server logs for email sending errors
# Look for lines like:
# "Email sent successfully: { id: 're_...' }"
# or
# "Failed to send email: ..."
```

### Token Issues

```bash
# Check token in URL is complete (no truncation)
# Tokens are long strings, ensure you copied the entire thing
```

### Session Issues

```bash
# Check if session token is in cookies
curl -X GET http://localhost:3000/api/auth/session \
  -v  # -v shows cookies in response
```

---

## ✅ Success Criteria

After testing, you should have:

- ✅ Received verification email with beautiful HTML template
- ✅ Successfully verified email using token
- ✅ User's `emailVerified` changed from `false` to `true`
- ✅ Received password reset email
- ✅ Successfully reset password
- ✅ Logged in with new password
- ✅ Old password rejected

---

## 💡 Pro Tips

1. **Use `| jq`** at the end of curl commands for pretty JSON formatting
2. **Save tokens** in a text file for easy copy-paste during testing
3. **Check spam folder** if emails don't arrive in inbox
4. **Use your real email** to see actual email templates
5. **Server logs** show detailed email sending status

---

**Ready to test?** Start with the email verification flow!
