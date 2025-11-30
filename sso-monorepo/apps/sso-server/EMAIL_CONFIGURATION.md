# Email Configuration Guide

The SSO server supports two email providers with automatic selection based on available environment variables.

---

## 🔄 Automatic Provider Selection

The system automatically chooses the email provider based on which credentials are configured:

1. **Priority 1: Resend** - If `RESEND_API_KEY` is set
2. **Priority 2: SMTP** - If `SMTP_HOST`, `SMTP_USER`, and `SMTP_PASS` are set
3. **No Provider** - Logs warning if neither is configured

---

## 📧 Option 1: Resend (Easiest Setup)

### Pros
- ✅ Easy to set up
- ✅ Reliable delivery
- ✅ Good for development

### Cons
- ⚠️ **100 emails/day limit** on free plan
- ⚠️ Requires domain verification for production

### Setup Steps

1. **Sign up at [resend.com](https://resend.com)**

2. **Get your API key** from the dashboard

3. **Add to `.env.local`:**
```env
RESEND_API_KEY=re_123456789abcdef
EMAIL_FROM=onboarding@resend.dev  # For testing
# EMAIL_FROM=noreply@yourdomain.com  # For production (requires domain verification)
APP_NAME=My SSO Platform
```

4. **Test it:**
```bash
# Start the server and try signing up
pnpm dev
```

### Production Considerations

For production with Resend:
- Verify your domain in Resend dashboard
- Update `EMAIL_FROM` to use your domain
- Consider upgrading plan if you need more than 100 emails/day

---

## 📧 Option 2: Google SMTP (Recommended for Production)

### Pros
- ✅ **Unlimited emails** with Gmail account
- ✅ No daily limits
- ✅ Free forever
- ✅ Reliable Google infrastructure

### Cons
- ⚠️ Requires 2FA and App Password setup
- ⚠️ Gmail may flag high-volume sending

### Setup Steps

#### 1. Enable 2-Factor Authentication

1. Go to [Google Account Security](https://myaccount.google.com/security)
2. Enable **2-Step Verification**
3. Wait for confirmation

#### 2. Generate App Password

1. Go to [App Passwords](https://myaccount.google.com/apppasswords)
   - (Or Google Account → Security → 2-Step Verification → App passwords)

2. **Create new app password:**
   - Select app: **Mail**
   - Select device: **Other (Custom name)**
   - Enter name: **SSO Server**
   - Click **Generate**

3. **Copy the 16-character password** (looks like `abcd efgh ijkl mnop`)

#### 3. Configure Environment Variables

Add to `.env.local`:

```env
# Google SMTP Configuration
SMTP_HOST=smtp.gmail.com
SMTP_PORT=587
SMTP_SECURE=false
SMTP_USER=your-email@gmail.com
SMTP_PASS=abcd efgh ijkl mnop  # The 16-character App Password
EMAIL_FROM=your-email@gmail.com
APP_NAME=My SSO Platform
```

**Important:**
- Use the **App Password**, NOT your regular Gmail password
- Remove spaces from the App Password (or keep them, both work)
- Set `SMTP_SECURE=false` for port 587 (TLS)
- Set `SMTP_SECURE=true` for port 465 (SSL)

#### 4. Test the Configuration

```bash
# Start the server
pnpm dev

# Try signing up with a test email
# Check the email arrives in inbox/spam
```

---

## 🔧 Using Other SMTP Providers

The SSO server works with any SMTP provider. Here are common configurations:

### Outlook/Office 365

```env
SMTP_HOST=smtp.office365.com
SMTP_PORT=587
SMTP_SECURE=false
SMTP_USER=your-email@outlook.com
SMTP_PASS=your-password
EMAIL_FROM=your-email@outlook.com
```

### Yahoo Mail

```env
SMTP_HOST=smtp.mail.yahoo.com
SMTP_PORT=587
SMTP_SECURE=false
SMTP_USER=your-email@yahoo.com
SMTP_PASS=your-app-password  # Requires App Password
EMAIL_FROM=your-email@yahoo.com
```

### SendGrid

```env
SMTP_HOST=smtp.sendgrid.net
SMTP_PORT=587
SMTP_SECURE=false
SMTP_USER=apikey
SMTP_PASS=your-sendgrid-api-key
EMAIL_FROM=noreply@yourdomain.com
```

### Mailgun

```env
SMTP_HOST=smtp.mailgun.org
SMTP_PORT=587
SMTP_SECURE=false
SMTP_USER=your-mailgun-smtp-user
SMTP_PASS=your-mailgun-smtp-password
EMAIL_FROM=noreply@yourdomain.com
```

### Amazon SES

```env
SMTP_HOST=email-smtp.us-east-1.amazonaws.com
SMTP_PORT=587
SMTP_SECURE=false
SMTP_USER=your-ses-smtp-user
SMTP_PASS=your-ses-smtp-password
EMAIL_FROM=noreply@yourdomain.com
```

---

## 🧪 Testing Email Configuration

### Method 1: Check Server Logs

When the server starts, it will log which email provider is being used:

```
✅ Email sent successfully via Resend: { id: '...' }
# OR
✅ Email sent successfully via SMTP: message-id-here
# OR
⚠️ No email provider configured. Email not sent
```

### Method 2: Test Sign-Up Flow

1. Start the server:
```bash
pnpm dev
```

2. Navigate to `http://localhost:3000/signup`

3. Create a test account with a real email you can access

4. Check your email (including spam folder)

5. Click the verification link

### Method 3: Test Password Reset

1. Go to `http://localhost:3000/forgot-password`

2. Enter your email

3. Check for password reset email

---

## 🔍 Debugging Email Issues

### Issue: "No email provider configured"

**Problem:** Neither Resend nor SMTP credentials are set

**Solution:**
```bash
# Check your .env.local file
# Make sure you have EITHER:

# Resend:
RESEND_API_KEY=re_...

# OR SMTP:
SMTP_HOST=smtp.gmail.com
SMTP_USER=...
SMTP_PASS=...
```

### Issue: "SMTP authentication failed"

**Problem:** Wrong credentials or not using App Password

**Solutions:**
1. Verify you're using **App Password**, not regular password
2. Check 2FA is enabled on Google account
3. Regenerate App Password if needed
4. Remove any spaces from the password
5. Make sure `SMTP_USER` matches the Gmail account that generated the App Password

### Issue: Emails go to spam

**Solutions:**
1. Add sender email to contacts
2. For Gmail SMTP: Mark first email as "Not Spam"
3. For production: Set up SPF, DKIM, and DMARC records
4. Use a verified domain with Resend

### Issue: "Connection timeout"

**Problem:** Firewall blocking SMTP port or wrong host

**Solutions:**
1. Check `SMTP_HOST` is correct
2. Verify port `587` or `465` is not blocked by firewall
3. Try `SMTP_PORT=465` with `SMTP_SECURE=true`
4. Test connection:
```bash
telnet smtp.gmail.com 587
# Should connect successfully
```

### Issue: Rate limiting

**Problem:** Too many emails sent

**Solutions:**
- **Resend Free:** 100 emails/day limit
- **Gmail:** ~500 emails/day for regular accounts, 2000/day for Google Workspace
- Switch to dedicated email service for high volume (SendGrid, Mailgun, SES)

---

## 🎯 Recommendations by Use Case

### Development/Testing
**Use:** Resend (free tier)
```env
RESEND_API_KEY=re_...
EMAIL_FROM=onboarding@resend.dev
```

### Small Production (<100 emails/day)
**Use:** Resend (free tier)
```env
RESEND_API_KEY=re_...
EMAIL_FROM=noreply@yourdomain.com  # Requires domain verification
```

### Medium Production (100-500 emails/day)
**Use:** Google SMTP
```env
SMTP_HOST=smtp.gmail.com
SMTP_PORT=587
SMTP_SECURE=false
SMTP_USER=your-email@gmail.com
SMTP_PASS=app-password
EMAIL_FROM=your-email@gmail.com
```

### Large Production (>500 emails/day)
**Use:** Dedicated service (SendGrid, Mailgun, Amazon SES)
```env
SMTP_HOST=smtp.sendgrid.net
SMTP_PORT=587
SMTP_SECURE=false
SMTP_USER=apikey
SMTP_PASS=your-api-key
EMAIL_FROM=noreply@yourdomain.com
```

---

## 📊 Cost Comparison

| Provider | Free Tier | Paid Plans |
|----------|-----------|------------|
| **Resend** | 100 emails/day | $20/month for 50k emails |
| **Gmail** | ~500 emails/day | Free forever |
| **Google Workspace** | ~2000 emails/day | $6/user/month |
| **SendGrid** | 100 emails/day | $19.95/month for 50k emails |
| **Mailgun** | 5,000 emails/month | $35/month for 50k emails |
| **Amazon SES** | 62,000 emails/month (AWS free tier) | $0.10 per 1000 emails |

---

## 🔒 Security Best Practices

### 1. Never Commit Credentials

```bash
# ❌ BAD
git add .env.local

# ✅ GOOD
# Add to .gitignore:
.env.local
.env*.local
```

### 2. Use App Passwords (Not Regular Passwords)

```env
# ❌ BAD
SMTP_PASS=my-gmail-password

# ✅ GOOD
SMTP_PASS=abcd efgh ijkl mnop  # App Password
```

### 3. Rotate Credentials Regularly

- Regenerate App Passwords every 90 days
- Rotate API keys quarterly
- Revoke unused credentials

### 4. Use Environment-Specific Credentials

```bash
# Development
.env.local

# Staging
.env.staging

# Production
# Use environment variables in hosting platform
# (Vercel, AWS, Railway, etc.)
```

---

## 📝 Email Templates

The SSO server sends two types of emails:

### 1. Email Verification

- Sent when user signs up
- Contains verification link
- Expires in 24 hours

### 2. Password Reset

- Sent when user requests password reset
- Contains reset link
- Expires in 1 hour

### Customization

To customize email templates, edit:
```
packages/auth-config/email.ts
```

You can modify:
- HTML styling
- Email content
- Subject lines
- Email from name

---

## 🧰 Troubleshooting Commands

### Test SMTP Connection

```bash
# Test if port is reachable
telnet smtp.gmail.com 587

# Or use nc (netcat)
nc -zv smtp.gmail.com 587
```

### Check Environment Variables

```bash
# Print (be careful - contains secrets!)
printenv | grep SMTP
printenv | grep EMAIL
printenv | grep RESEND
```

### View Server Logs

```bash
# Run server with full logs
pnpm dev

# Watch for email-related logs:
# "✅ Email sent successfully via Resend"
# "✅ Email sent successfully via SMTP"
# "⚠️ No email provider configured"
```

---

## 📖 Additional Resources

### Resend
- [Resend Documentation](https://resend.com/docs)
- [Resend Pricing](https://resend.com/pricing)
- [Domain Verification Guide](https://resend.com/docs/dashboard/domains/introduction)

### Gmail SMTP
- [Google App Passwords Guide](https://support.google.com/accounts/answer/185833)
- [Gmail SMTP Settings](https://support.google.com/mail/answer/7126229)
- [Gmail Sending Limits](https://support.google.com/a/answer/166852)

### Email Best Practices
- [SPF Records](https://dmarcian.com/spf-syntax-table/)
- [DKIM Setup](https://dmarcian.com/dkim-inspector/)
- [DMARC Policy](https://dmarcian.com/dmarc-inspector/)

---

## ✅ Quick Checklist

Before deploying to production:

- [ ] Email provider configured (Resend or SMTP)
- [ ] `EMAIL_FROM` set to your domain
- [ ] Test email verification flow
- [ ] Test password reset flow
- [ ] Check emails not going to spam
- [ ] Domain verified (if using Resend)
- [ ] Credentials stored securely
- [ ] Monitoring/logging set up

---

## 💬 Support

If you encounter issues with email configuration:

1. Check server logs for error messages
2. Review this guide's troubleshooting section
3. Test with a different email provider
4. Contact SSO server administrator

---

## License

Proprietary - Panaversity SSO Platform
