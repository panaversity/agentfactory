# Email Verification & Password Reset Setup Guide

This guide will help you set up and test email verification and password reset functionality in your SSO platform.

## 📋 What's Been Configured

✅ **Resend Email Service** - Installed and integrated
✅ **Email Sending Utility** - Created at `packages/auth-config/email.ts`
✅ **Email Verification** - Configured in BetterAuth
✅ **Password Reset** - Configured in BetterAuth
✅ **Beautiful Email Templates** - HTML templates with your branding

## 🔑 Step 1: Get Your Resend API Key

### Option A: For Testing (Free)

1. Go to [https://resend.com](https://resend.com)
2. Sign up for a free account
3. Go to [API Keys](https://resend.com/api-keys)
4. Click "Create API Key"
5. Name it "SSO Platform - Development"
6. Copy the API key (starts with `re_...`)

### Option B: For Production

1. After signing up, verify your domain in Resend
2. Add DNS records to your domain
3. Create an API key with your verified domain

## 🛠️ Step 2: Add API Key to Environment

Open `.env` and add your Resend API key:

```bash
# Email Configuration
RESEND_API_KEY="re_xxxxxxxxxxxxxxxxxxxxxxxxx"  # ← Paste your key here
EMAIL_FROM="onboarding@resend.dev"  # ← For testing, use resend.dev
APP_NAME="SSO Platform"
```

**For testing:** Use `onboarding@resend.dev` as the FROM address
**For production:** Use your own domain like `noreply@yourdomain.com`

## 🚀 Step 3: Restart Your Server

The configuration changes require a server restart:

```bash
# Stop the current server (Ctrl+C if running)
cd apps/sso-server
pnpm run dev
```

## 🧪 Step 4: Test Email Verification

### Test 1: Sign Up a New User

```bash
curl -X POST http://localhost:3000/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{
    "email": "your-real-email@gmail.com",
    "password": "TestPass123!",
    "name": "Test User"
  }'
```

**Expected behavior:**
1. User created in database
2. Verification email sent to your email
3. Check your inbox for a beautiful verification email
4. Click "Verify Email Address" button
5. You'll be redirected and logged in

### Test 2: Using the Client

```typescript
// In your sso-client or sso-admin app
import { authClient } from "@repo/auth-config/client";

// Sign up
const result = await authClient.signUp.email({
  email: "your-email@gmail.com",
  password: "SecurePass123!",
  name: "John Doe"
});

console.log(result);
// User receives verification email automatically!
```

### Test 3: Manually Trigger Verification Email

```typescript
// If user didn't receive email, they can request another one
await authClient.sendVerificationEmail({
  email: "user@example.com",
  callbackURL: "/dashboard"
});
```

## 🔐 Step 5: Test Password Reset

### Test 1: Request Password Reset

```bash
curl -X POST http://localhost:3000/api/auth/forget-password \
  -H "Content-Type: application/json" \
  -d '{
    "email": "your-email@gmail.com",
    "redirectTo": "http://localhost:3001/reset-password"
  }'
```

**Expected behavior:**
1. Password reset email sent
2. Check your inbox
3. Click "Reset Password" button
4. You'll be redirected to `/reset-password?token=xxx`

### Test 2: Using the Client

```typescript
// Request password reset
await authClient.forgetPassword({
  email: "user@example.com",
  redirectTo: "http://localhost:3001/reset-password"
});

// User receives password reset email with token
```

### Test 3: Complete Password Reset

```typescript
// On your reset password page (e.g., /reset-password)
import { useSearchParams } from "next/navigation";

const searchParams = useSearchParams();
const token = searchParams.get('token');

// Reset password
await authClient.resetPassword({
  newPassword: "NewSecurePass123!",
  token: token!
});

// User can now login with new password
```

## 📧 Email Templates

Your emails will look professional with:
- ✅ Gradient headers with your app name
- ✅ Clean, modern design
- ✅ Clear call-to-action buttons
- ✅ Copy-paste links for accessibility
- ✅ Expiry time information
- ✅ Mobile-responsive design

## 🎛️ Configuration Options

### Current Settings

Located in `packages/auth-config/index.ts`:

```typescript
emailAndPassword: {
  requireEmailVerification: false,  // Set to true to enforce verification
  autoSignIn: true,                 // Users auto-login after sign up
  minPasswordLength: 8,
  maxPasswordLength: 128,
  resetPasswordTokenExpiresIn: 3600,  // 1 hour
}

emailVerification: {
  sendOnSignUp: true,                      // Auto-send on sign up
  autoSignInAfterVerification: true,       // Auto-login after verify
  expiresIn: 60 * 60 * 24,                // 24 hours
}
```

### To Enforce Email Verification

Change `requireEmailVerification` to `true`:

```typescript
emailAndPassword: {
  requireEmailVerification: true,  // ← Users MUST verify before login
}
```

This will:
- Block unverified users from signing in
- Show "Please verify your email" error
- Require email verification before access

## 🐛 Troubleshooting

### Email Not Sending

**Problem:** Emails not arriving
**Solution:**
1. Check your RESEND_API_KEY is correct
2. Check server logs for errors
3. Verify you're using `onboarding@resend.dev` for testing
4. Check your spam folder

### Invalid Token Error

**Problem:** "Invalid or expired token"
**Solution:**
1. Token expired (1 hour for password reset, 24 hours for verification)
2. Request a new reset/verification email
3. Check the token in the URL is complete

### Email Verification Not Working

**Problem:** Verification link doesn't work
**Solution:**
1. Check BETTER_AUTH_URL is set correctly
2. Ensure server is running on that URL
3. Check browser console for errors
4. Verify the callback URL is accessible

## 📊 Available Endpoints

After configuration, these endpoints are now fully functional:

```
POST /api/auth/sign-up/email              # Creates user + sends verification
POST /api/auth/sign-in/email              # Login (blocked if unverified)
POST /api/auth/send-verification-email    # Manually send verification
GET  /api/auth/verify-email               # Verify email with token
POST /api/auth/forget-password            # Request password reset
POST /api/auth/reset-password             # Reset password with token
```

## 🎯 Next Steps

1. ✅ Get Resend API key
2. ✅ Add to `.env`
3. ✅ Restart server
4. ✅ Test sign up flow
5. ✅ Test password reset
6. Create UI pages for:
   - Password reset form
   - Email verification success
   - Resend verification email

## 📚 Additional Resources

- [Resend Documentation](https://resend.com/docs)
- [BetterAuth Email Docs](https://www.better-auth.com/docs/authentication/email-password)
- [BetterAuth API Reference](https://www.better-auth.com/docs/reference/api)

## 💡 Pro Tips

1. **Testing:** Use your real email for testing to see actual emails
2. **Development:** Resend free tier includes 100 emails/day
3. **Production:** Verify your domain in Resend for higher limits
4. **Debugging:** Check server console for email sending logs
5. **Security:** Never commit your RESEND_API_KEY to git (it's in .gitignore)

---

**Need help?** Check the server logs or open an issue in your repository!
