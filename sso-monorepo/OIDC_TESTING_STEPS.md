# OIDC Flow - Step-by-Step Testing Guide

This guide walks you through testing the OIDC Authorization Code Flow with your registered client.

## 🔐 Your Client Credentials

**Save these somewhere safe!**

```
Client ID: RdwUTXGzXgIJCrySmMZTDioHkizYqUgp
Client Secret: ItsFjRVHNnMfIMnrIaPaZZxebAOPtGXe
Redirect URI: http://localhost:4000/callback
```

**Test User:**
```
Email: zeeshan922837@gmail.com
Password: TestPassword123!
```

---

## 📋 STEP 1: Client Registration ✅ DONE

**What you sent:**
```bash
curl -X POST http://localhost:3000/api/auth/oauth2/register \
  -H "Content-Type: application/json" \
  -d '{
    "client_name": "Test Dashboard App",
    "redirect_uris": ["http://localhost:4000/callback"],
    "grant_types": ["authorization_code", "refresh_token"],
    "response_types": ["code"],
    "scope": "openid email profile"
  }'
```

**What you received:**
```json
{
  "client_id": "RdwUTXGzXgIJCrySmMZTDioHkizYqUgp",
  "client_secret": "ItsFjRVHNnMfIMnrIaPaZZxebAOPtGXe",
  "redirect_uris": ["http://localhost:4000/callback"],
  "grant_types": ["authorization_code", "refresh_token"],
  "response_types": ["code"],
  "client_name": "Test Dashboard App"
}
```

✅ **Client is now registered and ready to use!**

---

## 📋 STEP 2: User Clicks "Login with SSO"

When a user on your client app clicks "Login", your app redirects them to:

### Authorization URL:
```
http://localhost:3000/api/auth/oauth2/authorize?client_id=RdwUTXGzXgIJCrySmMZTDioHkizYqUgp&redirect_uri=http://localhost:4000/callback&response_type=code&scope=openid%20email%20profile&state=random_state_1764046685&nonce=random_nonce_1764046685
```

### URL Parameters Explained:
- `client_id`: Your registered client ID
- `redirect_uri`: Where to send the user after login
- `response_type=code`: We want an authorization code
- `scope=openid email profile`: What user data we want access to
- `state`: Random string for CSRF protection
- `nonce`: Random string for replay attack protection

### ⚡ ACTION REQUIRED:
**Copy the URL above and open it in your browser!**

---

## 📋 STEP 3: What Happens in the Browser

### Scenario A: User NOT Logged In (First Time)

1. **Browser redirects to SSO login page:**
   ```
   http://localhost:3000/auth/login?returnTo=/api/auth/oauth2/authorize?...
   ```

2. **User sees your SSO login form**

3. **User enters credentials:**
   - Email: `zeeshan922837@gmail.com`
   - Password: `TestPassword123!`

4. **Behind the scenes:**
   ```bash
   POST http://localhost:3000/api/auth/sign-in/email
   {
     "email": "zeeshan922837@gmail.com",
     "password": "TestPassword123!"
   }
   ```

5. **SSO Server response:**
   ```json
   {
     "token": "session_token_abc123",
     "user": {
       "id": "user_123",
       "email": "zeeshan922837@gmail.com",
       "name": "Zeeshan"
     }
   }
   ```

6. **Session cookie is set:**
   ```
   Set-Cookie: better-auth.session_token=M658oF9wosXbxxwQtz7tpcyIhT37s7eX
   ```

7. **Browser automatically redirects back to authorization endpoint** (now logged in)

### Scenario B: User Already Logged In (SSO)

If the user already logged in to another app using this SSO:
- ✅ Login page is **skipped**
- ✅ Goes directly to consent (or straight to redirect if skipConsent)
- ✅ This is the "Single Sign-On" magic! 🎉

---

## 📋 STEP 4: Consent Screen (May be Skipped)

**If your client requires consent:**
1. User sees: "Test Dashboard App wants to access your:"
   - Email address
   - Profile information
2. User clicks "Allow"

**For your client:**
Since this is a newly registered client (not in trustedClients with skipConsent), consent **will be shown**.

---

## 📋 STEP 5: Get Authorization Code

After login + consent, SSO server redirects browser to:

```
http://localhost:4000/callback?code=AUTHORIZATION_CODE_HERE&state=random_state_1764046685
```

### Example:
```
http://localhost:4000/callback?code=PXbF8xCE8F5JgGG4zWS78tLXKzQnbDMPXTXE8eEz3hA&state=random_state_1764046685
```

### What you get:
- `code`: Authorization code (single-use, expires in ~10 minutes)
- `state`: Same state you sent (verify it matches!)

### ⚡ ACTION REQUIRED:
**From the browser address bar, copy the `code` value!**

Example: If you see `?code=ABC123XYZ&state=...`, copy `ABC123XYZ`

---

## 📋 STEP 6: Exchange Code for Tokens

Now your **client app backend** exchanges the authorization code for tokens.

### Command to run:
```bash
# Replace CODE_FROM_BROWSER with the actual code you copied
CODE="CODE_FROM_BROWSER"

curl -X POST http://localhost:3000/api/auth/oauth2/token \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -u "RdwUTXGzXgIJCrySmMZTDioHkizYqUgp:ItsFjRVHNnMfIMnrIaPaZZxebAOPtGXe" \
  -d "grant_type=authorization_code" \
  -d "code=${CODE}" \
  -d "redirect_uri=http://localhost:4000/callback"
```

### Alternative (if above doesn't work):
```bash
curl -X POST http://localhost:3000/api/auth/oauth2/token \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -d "grant_type=authorization_code" \
  -d "code=${CODE}" \
  -d "redirect_uri=http://localhost:4000/callback" \
  -d "client_id=RdwUTXGzXgIJCrySmMZTDioHkizYqUgp" \
  -d "client_secret=ItsFjRVHNnMfIMnrIaPaZZxebAOPtGXe"
```

### Expected Response:
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiJTaDU3WmpaUjYwWVRQaEZoQjJ6emxoTnIycnd1Q1lKVCIsInNjb3BlIjoib3BlbmlkIGVtYWlsIHByb2ZpbGUiLCJpYXQiOjE3MDAwMDAwMDAsImV4cCI6MTcwMDAwMzYwMH0...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "refresh_token_xyz789...",
  "id_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiJTaDU3WmpaUjYwWVRQaEZoQjJ6emxoTnIycnd1Q1lKVCIsImVtYWlsIjoiemVlc2hhbjkyMjgzN0BnbWFpbC5jb20iLCJlbWFpbF92ZXJpZmllZCI6ZmFsc2UsIm5hbWUiOiJaZWVzaGFuIiwiaWF0IjoxNzAwMDAwMDAwLCJleHAiOjE3MDAwMDM2MDAsImF1ZCI6IlJkd1VUWEd6WGdJSkNyeVNtTVpURGlvSGtpellxVWdwIiwiaXNzIjoiaHR0cDovL2xvY2FsaG9zdDozMDAwIiwibm9uY2UiOiJyYW5kb21fbm9uY2VfMTc2NDA0NjY4NSJ9...",
  "scope": "openid email profile"
}
```

### What you received:
- ✅ **access_token**: Use to call APIs (expires in 1 hour)
- ✅ **id_token**: Contains user identity (JWT)
- ✅ **refresh_token**: Use to get new access tokens
- ✅ **expires_in**: 3600 seconds (1 hour)

---

## 📋 STEP 7: Decode the ID Token

The `id_token` is a JWT containing user information. Decode it to see the user data.

### Online Method:
1. Copy the `id_token` value
2. Go to https://jwt.io
3. Paste the token
4. See decoded payload

### Expected Payload:
```json
{
  "sub": "Sh57ZjZR60YTPhFhB2zzlhNr2rwuCYJT",
  "email": "zeeshan922837@gmail.com",
  "email_verified": false,
  "name": "Zeeshan",
  "picture": null,
  "iat": 1764046685,
  "exp": 1764050285,
  "aud": "RdwUTXGzXgIJCrySmMZTDioHkizYqUgp",
  "iss": "http://localhost:3000",
  "nonce": "random_nonce_1764046685"
}
```

### Fields Explained:
- `sub`: User ID (subject)
- `email`: User's email address
- `email_verified`: Is email verified?
- `name`: User's display name
- `iat`: Issued at (Unix timestamp)
- `exp`: Expires at (Unix timestamp)
- `aud`: Audience (your client_id)
- `iss`: Issuer (SSO server URL)
- `nonce`: Must match what you sent

### ⚠️ IMPORTANT: Verify These Claims!
Your client app MUST verify:
1. ✅ Signature (using public keys from `/api/auth/jwks`)
2. ✅ `iss` = "http://localhost:3000"
3. ✅ `aud` = "RdwUTXGzXgIJCrySmMZTDioHkizYqUgp"
4. ✅ `exp` > current time
5. ✅ `nonce` = "random_nonce_1764046685"

---

## 📋 STEP 8: Get User Info (Optional)

Use the `access_token` to call the UserInfo endpoint:

```bash
# Replace ACCESS_TOKEN with your actual token
ACCESS_TOKEN="your_access_token_here"

curl -X GET http://localhost:3000/api/auth/oauth2/userinfo \
  -H "Authorization: Bearer ${ACCESS_TOKEN}"
```

### Expected Response:
```json
{
  "sub": "Sh57ZjZR60YTPhFhB2zzlhNr2rwuCYJT",
  "email": "zeeshan922837@gmail.com",
  "email_verified": false,
  "name": "Zeeshan",
  "picture": null
}
```

This gives you the same user info as the ID token, but fetched dynamically.

---

## 📋 STEP 9: Refresh Access Token (When Expired)

After 1 hour, the `access_token` expires. Use `refresh_token` to get a new one:

```bash
# Replace REFRESH_TOKEN with your actual refresh token
REFRESH_TOKEN="your_refresh_token_here"

curl -X POST http://localhost:3000/api/auth/oauth2/token \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -u "RdwUTXGzXgIJCrySmMZTDioHkizYqUgp:ItsFjRVHNnMfIMnrIaPaZZxebAOPtGXe" \
  -d "grant_type=refresh_token" \
  -d "refresh_token=${REFRESH_TOKEN}"
```

### Expected Response:
```json
{
  "access_token": "new_access_token...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "new_refresh_token...",
  "id_token": "new_id_token...",
  "scope": "openid email profile"
}
```

✅ **User stays logged in without re-entering password!**

---

## 🎯 Complete Flow Summary

```
1. Register Client → Get client_id & client_secret
                   ↓
2. User clicks "Login" on Client App
                   ↓
3. Redirect to: /api/auth/oauth2/authorize
                   ↓
4. User logs in on SSO Server (if not already)
                   ↓
5. User grants consent (if required)
                   ↓
6. Redirect back to Client App with authorization code
                   ↓
7. Client Backend: Exchange code for tokens
                   ↓
8. Client receives: access_token, id_token, refresh_token
                   ↓
9. Client decodes & verifies id_token
                   ↓
10. User is logged in! ✅
```

---

## 🧪 Quick Test Checklist

- [ ] Step 1: Register client ✅ (Done)
- [ ] Step 2: Create test user ✅ (Done)
- [ ] Step 3: Open authorization URL in browser
- [ ] Step 4: Login with credentials
- [ ] Step 5: Grant consent (if shown)
- [ ] Step 6: Copy authorization code from redirect URL
- [ ] Step 7: Exchange code for tokens
- [ ] Step 8: Decode ID token (verify claims)
- [ ] Step 9: Call UserInfo endpoint (optional)
- [ ] Step 10: Test refresh token (optional)

---

## 🔧 Troubleshooting

### Issue: "redirect_uri_mismatch"
**Cause:** The redirect_uri in your request doesn't match what's registered.
**Fix:** Use exactly: `http://localhost:4000/callback`

### Issue: "invalid_client"
**Cause:** Wrong client_id or client_secret.
**Fix:** Double-check:
- Client ID: `RdwUTXGzXgIJCrySmMZTDioHkizYqUgp`
- Client Secret: `ItsFjRVHNnMfIMnrIaPaZZxebAOPtGXe`

### Issue: "invalid_grant" or "code already used"
**Cause:** Authorization code expired or already used.
**Fix:** Authorization codes are single-use and expire in ~10 minutes. Start over from Step 2.

### Issue: Browser shows "Cannot connect" at callback
**Cause:** There's no server running on `http://localhost:4000`
**Fix:** This is expected! Just copy the `code` from the URL bar. The code is in the URL even though the page doesn't load.

### Issue: Token endpoint returns 404
**Cause:** Token endpoint is disabled in config.
**Fix:** Edit `packages/auth-config/index.ts` and remove:
```typescript
disabledPaths: ['/token'],
```

---

## 📚 Next Steps

1. **Implement in your client app:**
   - Add "Login with SSO" button
   - Handle authorization redirect
   - Exchange code for tokens
   - Verify ID token
   - Store tokens securely

2. **Test SSO (Single Sign-On):**
   - Register a second client
   - Login to first client
   - Try logging into second client
   - Should skip login page! ✅

3. **Production Setup:**
   - Use HTTPS (required for production)
   - Rotate client secrets regularly
   - Set up proper CORS
   - Configure session timeouts
   - Add rate limiting

---

## 🎉 Congratulations!

You've successfully tested the complete OIDC Authorization Code Flow!

Your SSO server is now ready to authenticate users for any OIDC-compliant application! 🚀
