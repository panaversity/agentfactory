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
