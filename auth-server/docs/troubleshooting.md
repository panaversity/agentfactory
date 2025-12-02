# Troubleshooting Guide

Common issues and solutions for the RoboLearn Auth Server.

## Authentication Issues

### "Invalid credentials" on sign-in

**Causes:**
1. Wrong email or password
2. User doesn't exist
3. Password too short (minimum 8 characters)

**Solutions:**
- Verify email/password are correct
- Check if user exists in database
- Ensure password meets minimum requirements

### "Email not verified"

**Causes:**
- User signed up but didn't verify email
- Verification link expired (1 hour)

**Solutions:**
```bash
# Option 1: Resend verification email
POST /api/auth/send-verification-email
{"email": "user@example.com"}

# Option 2: Manually verify (development only)
npx tsx scripts/verify-test-user.ts
# Or directly in SQL:
UPDATE "user" SET email_verified = true WHERE email = 'user@example.com';
```

### "User is banned"

**Causes:**
- Admin banned the user

**Solutions:**
```sql
-- Check ban status
SELECT id, email, banned, ban_reason, ban_expires FROM "user" WHERE email = 'user@example.com';

-- Unban user
UPDATE "user" SET banned = false, ban_reason = NULL, ban_expires = NULL WHERE email = 'user@example.com';
```

## OAuth Issues

### "invalid_client" Error

**Causes:**
1. Client not registered
2. Client disabled
3. Wrong client_id

**Solutions:**
```bash
# Check if client exists
SELECT * FROM oauth_application WHERE client_id = 'your-client-id';

# Register client if missing
pnpm seed:clients

# Enable disabled client
UPDATE oauth_application SET disabled = false WHERE client_id = 'your-client-id';
```

### "invalid_redirect_uri" Error

**Causes:**
- Redirect URI doesn't match registered URIs
- Missing trailing slash
- HTTP vs HTTPS mismatch

**Solutions:**
```sql
-- Check registered redirect URIs
SELECT client_id, redirect_urls FROM oauth_application WHERE client_id = 'your-client-id';

-- Update redirect URIs (comma-separated)
UPDATE oauth_application
SET redirect_urls = 'http://localhost:3000/callback,https://myapp.com/callback'
WHERE client_id = 'your-client-id';
```

### "invalid_grant" - Code Expired or Used

**Causes:**
1. Authorization code expired (10 minute lifetime)
2. Code already exchanged (one-time use)
3. PKCE verifier doesn't match challenge

**Solutions:**
- Start a new authorization flow
- Ensure code_verifier is stored before redirect
- Use sessionStorage, not localStorage for PKCE verifier

### PKCE Challenge Mismatch

**Causes:**
- Code verifier was regenerated
- Wrong verifier sent to token endpoint

**Solutions:**
```javascript
// Ensure verifier is stored BEFORE redirect
const verifier = generateCodeVerifier();
sessionStorage.setItem('pkce_verifier', verifier);  // Store first!
const challenge = generateCodeChallenge(verifier);
// Then redirect...

// In callback, retrieve same verifier
const storedVerifier = sessionStorage.getItem('pkce_verifier');
// Use storedVerifier for token exchange
```

### "No redirect" After Authorization

**Causes:**
- User not logged in
- Consent not granted
- Session cookie not sent

**Solutions:**
```bash
# Check if session exists
curl -v http://localhost:3001/api/auth/oauth2/authorize?... \
  -H "Cookie: your-session-cookie"

# Better Auth returns JSON instead of HTTP redirect
# Parse the response body:
{"redirect": true, "url": "http://localhost:3000/callback?code=..."}
```

## Token Issues

### "Token expired"

**Causes:**
- Access token expired (6 hours default)
- ID token expired

**Solutions:**
```javascript
// Check token expiration before use
const payload = JSON.parse(atob(token.split('.')[1]));
if (payload.exp * 1000 < Date.now()) {
  // Token expired, refresh or re-authenticate
}
```

### "Unable to find matching key" (JWKS)

**Causes:**
1. JWKS cache stale
2. Key rotated
3. Wrong issuer URL

**Solutions:**
```bash
# Verify JWKS endpoint
curl http://localhost:3001/api/auth/jwks | jq '.keys[].kid'

# Check token kid matches JWKS
node -e "console.log(JSON.parse(Buffer.from('TOKEN'.split('.')[0], 'base64url')).kid)"

# Clear JWKS cache in your application
```

### "Invalid signature"

**Causes:**
1. Token tampered with
2. Wrong public key
3. Algorithm mismatch

**Solutions:**
- Ensure using RS256 algorithm
- Verify JWKS URL is correct
- Check token wasn't modified

## Database Issues

### "relation does not exist"

**Causes:**
- Schema not pushed to database
- Wrong database URL

**Solutions:**
```bash
# Push schema
pnpm db:push

# Verify connection
echo $DATABASE_URL

# Check tables
psql $DATABASE_URL -c "\dt"
```

### "unique constraint violation"

**Causes:**
- Duplicate email/client_id
- Re-running seed script

**Solutions:**
- Seed scripts handle duplicates with ON CONFLICT
- Check existing records before insert

### Connection Errors

**Causes:**
- Wrong DATABASE_URL
- Network issues
- SSL certificate issues

**Solutions:**
```bash
# Test connection
psql $DATABASE_URL -c "SELECT 1"

# Ensure SSL mode for Neon
# DATABASE_URL should include ?sslmode=require
```

## Email Issues

### Emails Not Sending

**Causes:**
1. Email provider not configured
2. Wrong API key
3. From address not verified

**Solutions:**
```bash
# Check email configuration
# In auth.ts, look for:
# [Auth] Email enabled via: SMTP from: your@email.com
# Or:
# [Auth] Email disabled - missing provider or EMAIL_FROM

# For Resend:
RESEND_API_KEY=re_xxx
RESEND_FROM_EMAIL=onboarding@resend.dev  # Use verified domain in production

# For SMTP (Gmail):
SMTP_HOST=smtp.gmail.com
SMTP_PORT=587
SMTP_USER=your@gmail.com
SMTP_PASS=your-app-password  # Not regular password!
EMAIL_FROM=your@gmail.com
```

### Gmail App Password

Gmail requires an "App Password" for SMTP:

1. Enable 2-factor authentication
2. Go to Google Account > Security > App passwords
3. Generate password for "Mail" on "Other"
4. Use that password as SMTP_PASS

## CORS Issues

### "CORS policy: No 'Access-Control-Allow-Origin'"

**Causes:**
- Origin not in trustedOrigins
- Preflight request failing

**Solutions:**
```bash
# Add origin to env
ALLOWED_ORIGINS=http://localhost:3000,https://myapp.com

# Or in development, the default allows localhost:3000
```

## Server Issues

### "EADDRINUSE: address already in use :::3001"

**Causes:**
- Another process on port 3001

**Solutions:**
```bash
# Find and kill process
lsof -ti :3001 | xargs kill -9

# Or use different port
PORT=3002 pnpm dev
```

### Server Not Starting

**Causes:**
1. Missing environment variables
2. Database connection failed
3. Syntax error in code

**Solutions:**
```bash
# Check required env vars
cat .env.local

# Required:
DATABASE_URL=...
BETTER_AUTH_SECRET=...
BETTER_AUTH_URL=...
```

## Testing Issues

### Test User Not Found

```bash
# Create test user via UI at http://localhost:3001/auth/sign-up

# Or create via API:
curl -X POST http://localhost:3001/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com", "password": "TestPass123", "name": "Test User"}'
```

### Test Client Not Found

```bash
# Seed test clients
pnpm seed:clients

# Verify
SELECT client_id FROM oauth_application;
```

## Debug Mode

### Enable Debug Logging

```typescript
// In auth.ts, add:
export const auth = betterAuth({
  // ...
  advanced: {
    // ...
    debugMode: true,  // Enables verbose logging
  },
});
```

### Inspect Network Requests

```bash
# Use curl with verbose output
curl -v http://localhost:3001/api/auth/... 2>&1 | grep -E "^[<>]"

# Or use browser DevTools Network tab
```

### Check Server Logs

```bash
# Server logs show auth events
pnpm dev
# Watch for [Auth] prefixed messages
```

## Getting Help

1. **Run tests first**: `pnpm test-auth`
2. **Check server logs**: Look for error messages
3. **Verify configuration**: Compare with .env.example
4. **Check Better Auth docs**: https://better-auth.com/docs

Still stuck? Open an issue with:
- Error message
- Steps to reproduce
- Environment (Node version, OS)
- Relevant configuration (sanitized)
