# Environment Variables Reference

## 📋 Complete List

### Core Configuration

| Variable | Purpose | Example | Required |
|----------|---------|---------|----------|
| `DATABASE_URL` | PostgreSQL connection string | `postgresql://user:pass@host.neon.tech/db?sslmode=require` | ✅ Yes |
| `BETTER_AUTH_SECRET` | Secret key for JWT signing (min 32 chars) | `openssl rand -base64 32` | ✅ Yes |
| `BETTER_AUTH_URL` | Auth server URL (backend) | `http://localhost:3001` | ✅ Yes |
| `NEXT_PUBLIC_BETTER_AUTH_URL` | Auth server URL (client-side) | `http://localhost:3001` | ✅ Yes |
| `NODE_ENV` | Environment | `development` or `production` | ✅ Yes |

### CORS Configuration

| Variable | Purpose | Example | Required |
|----------|---------|---------|----------|
| `ALLOWED_ORIGINS` | Comma-separated allowed origins | `http://localhost:3000,https://panaversity.org` | ✅ Yes |

### Branding (White-Label Support)

| Variable | Purpose | Example | Default |
|----------|---------|---------|---------|
| `NEXT_PUBLIC_APP_NAME` | Application name shown in UI | `Panaversity SSO` | `Panaversity SSO` |
| `NEXT_PUBLIC_APP_DESCRIPTION` | App description in header | `Secure Single Sign-On` | `Secure Single Sign-On` |
| `NEXT_PUBLIC_ORG_NAME` | Organization name in emails/footer | `Panaversity` | `Panaversity` |

### Continue URL Configuration

| Variable | Purpose | Example | Default |
|----------|---------|---------|---------|
| `NEXT_PUBLIC_CONTINUE_URL` | Where users go after authentication | `http://localhost:3000` | - |

**What is `NEXT_PUBLIC_CONTINUE_URL`?**
- Used for the "Continue to Application" button on the post-auth page
- Manual fallback link if user is not being redirected via OAuth
- Points to your main application (not the auth server itself)
- **NOT** used for OAuth redirect URLs (those are in `src/lib/trusted-clients.ts`)

### Email Configuration (Optional)

Choose **one** email provider:

#### Option 1: SMTP (Gmail, Custom SMTP)

| Variable | Purpose | Example |
|----------|---------|---------|
| `SMTP_HOST` | SMTP server hostname | `smtp.gmail.com` |
| `SMTP_PORT` | SMTP port | `587` |
| `SMTP_USER` | SMTP username | `your@gmail.com` |
| `SMTP_PASS` | SMTP password/app password | `your-16-char-app-password` |
| `SMTP_SECURE` | Use TLS? | `false` (use STARTTLS on port 587) |
| `EMAIL_FROM` | From address for emails | `your@gmail.com` |

#### Option 2: Resend API

| Variable | Purpose | Example |
|----------|---------|---------|
| `RESEND_API_KEY` | Resend API key | `re_xxxxxxxxx` |
| `RESEND_FROM_EMAIL` | From address | `onboarding@resend.dev` |

**Priority:** If both are configured, SMTP is used first.

### OAuth Client Configuration

**✅ No environment variables needed!**

All OAuth client redirect URLs are now hardcoded in `src/lib/trusted-clients.ts` with automatic environment-based filtering:

- **Development**: Localhost URLs included
- **Production**: Localhost URLs automatically removed

This provides better security and consistency without requiring environment variables.

---

## 🔍 Common Confusion Explained

### ❓ What's the difference between `NEXT_PUBLIC_CONTINUE_URL` and OAuth redirect URLs?

**`NEXT_PUBLIC_CONTINUE_URL`:**
- UI/UX feature for the "Continue to Application" button
- Manual fallback link if OAuth redirect doesn't happen
- Where users go when they land on the auth server homepage
- **NOT** used for OAuth security flows

**OAuth Redirect URLs:**
- Security feature for OAuth 2.0 authorization flow
- Where users are **automatically** redirected after successful authorization
- Configured per OAuth client in `src/lib/trusted-clients.ts`
- **Validated during OAuth flow** (must match exactly)

**Example:**
```
NEXT_PUBLIC_CONTINUE_URL=https://robolearn.panaversity.org
  ↑ Used for "Continue to Application" button (manual link)

OAuth Client Redirect URLs (in trusted-clients.ts):
  ✅ https://robolearn.panaversity.org/auth/callback
  ✅ http://localhost:3000/auth/callback (filtered in production)
  ↑ Used for OAuth authorization code flow (automatic redirect)
```

**Why both?**
- OAuth flow: Secure, validated, automatic redirect to callback URL
- Continue button: Convenience link if user lands on auth server directly

### ❓ Why is `PANAVERSITY_SSO_REDIRECT_URLS` not implemented?

**Previous approach (inflexible):**
- Hardcode redirect URLs in seed script
- Use environment variable to override
- Requires re-seeding to change URLs

**Current approach (flexible):**
- Seed with default URLs via `pnpm run seed:prod`
- **Edit redirect URLs dynamically via Admin UI** at `/admin/clients`
- No re-seeding needed!
- Can add/remove localhost URLs instantly for testing

**Workflow:**
1. **Production Setup:** Run `pnpm run seed:prod` once
2. **Testing:** Admin UI → Edit client → Add `http://localhost:3000/callback` → Save
3. **After Testing:** Admin UI → Edit client → Remove localhost URL → Save

---

## 🧪 Testing Environment Variables

For running tests, you may need:

| Variable | Purpose | How to Get |
|----------|---------|------------|
| `ADMIN_SESSION_COOKIE` | Admin session for API tests | 1. Sign in as admin<br>2. DevTools > Cookies<br>3. Copy cookie string |

---

## 📝 Example `.env.local`

```env
# Core
DATABASE_URL=postgresql://user:pass@host.neon.tech/db?sslmode=require
BETTER_AUTH_SECRET=your-32-character-secret-key-here
BETTER_AUTH_URL=http://localhost:3001
NEXT_PUBLIC_BETTER_AUTH_URL=http://localhost:3001
NODE_ENV=development

# CORS
ALLOWED_ORIGINS=http://localhost:3000,https://panaversity.org

# Branding
NEXT_PUBLIC_APP_NAME=Panaversity SSO
NEXT_PUBLIC_APP_DESCRIPTION=Secure Single Sign-On
NEXT_PUBLIC_ORG_NAME=Panaversity

# Continue URL
NEXT_PUBLIC_CONTINUE_URL=http://localhost:3000

# Email (SMTP)
SMTP_HOST=smtp.gmail.com
SMTP_PORT=587
SMTP_USER=your@gmail.com
SMTP_PASS=your-app-password
SMTP_SECURE=false
EMAIL_FROM=your@gmail.com
```

---

## 🔒 Security Best Practices

1. **Never commit `.env.local` to git** (it's in `.gitignore`)
2. **Use different secrets for dev/prod**
3. **Rotate `BETTER_AUTH_SECRET` periodically**
4. **Use app passwords for Gmail SMTP** (not your main password)
5. **Manage redirect URLs via Admin UI**, not environment variables (better security)

---

## 🆘 Troubleshooting

### Issue: "OAuth redirect URL not allowed"
**Cause:** Redirect URL not registered in OAuth client configuration

**Solution:**
1. Go to `/admin/clients` as admin
2. Click "Edit" on the client
3. Add the redirect URL
4. Save

### Issue: "Cannot edit pre-configured trusted client"
**Cause:** Trying to edit `robolearn-public-client` via API/UI

**Solution:**
- Trusted clients are configured in `src/lib/auth.ts`
- Edit the code directly, not via UI
- This prevents accidental modification of critical clients

### Issue: Emails not sending
**Cause:** Email configuration missing or incorrect

**Solution:**
1. Check if `SMTP_*` or `RESEND_*` variables are set
2. Test SMTP credentials with a separate SMTP tool
3. Check dev server logs for email errors
