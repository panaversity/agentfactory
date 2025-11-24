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
