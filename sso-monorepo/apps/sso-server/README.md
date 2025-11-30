# SSO Server - Unified Authentication Platform

Modern Single Sign-On (SSO) server with authentication UI built with Next.js 16, React 19, Better Auth, and TypeScript. This is a **unified application** containing both the OIDC provider backend and authentication UI pages.

## Features

### Authentication Features
- ✅ **Email/Password Authentication**: Sign up and sign in with email
- ✅ **Social Login**: GitHub and Google OAuth integration
- ✅ **Password Reset**: Forgot password flow with email verification
- ✅ **Email Verification**: Verify email addresses with token-based links
- ✅ **Session Management**: Secure httpOnly cookies with Better Auth

### OIDC Provider Features
- ✅ **OpenID Connect Provider**: Full OIDC Authorization Code Flow
- ✅ **Dynamic Client Registration**: Register OIDC clients via API
- ✅ **JWT Token Signing**: RS256 asymmetric signing with JWKS
- ✅ **Discovery Document**: Standard `.well-known/openid-configuration`
- ✅ **User Consent**: OAuth consent screen for third-party applications

### UI/UX Features
- ✅ **Responsive Design**: Mobile-first design with Tailwind CSS
- ✅ **Accessibility**: WCAG compliant with proper ARIA labels
- ✅ **Form Validation**: Client-side validation with Zod schemas
- ✅ **Error Handling**: Comprehensive error messages and timeout handling
- ✅ **Dark Mode**: System-aware theme support

## Tech Stack

- **Framework**: Next.js 16 (App Router with Turbopack)
- **UI**: React 19, Tailwind CSS, shadcn/ui
- **Authentication**: Better Auth with OIDC Provider plugin
- **Forms**: React Hook Form with Zod validation
- **Database**: PostgreSQL with Drizzle ORM
- **Icons**: Lucide React
- **Email**: Resend for transactional emails

## Getting Started

### Prerequisites

- Node.js 18+ or Bun
- pnpm (recommended) or npm
- PostgreSQL database (local or remote)
- (Optional) OAuth app credentials for GitHub/Google

### Environment Variables

Create a `.env.local` file in the `apps/sso-server` directory:

```env
# Database
DATABASE_URL=postgresql://user:password@localhost:5432/sso_db

# Better Auth
BETTER_AUTH_SECRET=your-secret-key-here
BETTER_AUTH_URL=http://localhost:3000

# Email Configuration (Choose ONE option)

# Option 1: Resend (100 emails/day on free plan)
RESEND_API_KEY=your-resend-api-key
EMAIL_FROM=noreply@yourdomain.com

# Option 2: Google SMTP (Unlimited with Gmail account)
# If RESEND_API_KEY is not set, will use SMTP automatically
SMTP_HOST=smtp.gmail.com
SMTP_PORT=587
SMTP_SECURE=false
SMTP_USER=your-email@gmail.com
SMTP_PASS=your-app-password  # Use App Password, not regular password
EMAIL_FROM=your-email@gmail.com

# Application Name (for emails)
APP_NAME=SSO Platform

# Optional: OAuth Providers
GITHUB_CLIENT_ID=your-github-client-id
GITHUB_CLIENT_SECRET=your-github-client-secret

GOOGLE_CLIENT_ID=your-google-client-id
GOOGLE_CLIENT_SECRET=your-google-client-secret

# Optional: CAPTCHA
NEXT_PUBLIC_HCAPTCHA_SITE_KEY=your-hcaptcha-site-key
```

### Installation

From the monorepo root:

```bash
# Install dependencies
pnpm install

# Run database migrations
cd packages/database
pnpm db:push

# Start development server
cd ../../apps/sso-server
pnpm dev
```

The server will be available at `http://localhost:3000`

## Available Routes

### Authentication Pages

| Route | Description |
|-------|-------------|
| `/signup` | User registration with email and password |
| `/signin` | User authentication |
| `/forgot-password` | Request password reset email |
| `/reset-password?token=...` | Reset password with token |
| `/verify-email` | Email verification success message |
| `/verify-email/verify?token=...` | Email verification token handler |
| `/login?client_id=...` | OIDC login page (for external apps) |
| `/consent?client_id=...` | OIDC consent page (for external apps) |
| `/dashboard` | User dashboard (redirect target after auth) |

### API Endpoints

#### Better Auth API
| Endpoint | Description |
|----------|-------------|
| `/api/auth/sign-in/email` | Email/password sign in |
| `/api/auth/sign-up/email` | Email/password sign up |
| `/api/auth/sign-in/github` | GitHub OAuth |
| `/api/auth/sign-in/google` | Google OAuth |
| `/api/auth/forget-password` | Request password reset |
| `/api/auth/reset-password` | Reset password with token |
| `/api/auth/verify-email` | Verify email with token |

#### OIDC Provider Endpoints
| Endpoint | Description |
|----------|-------------|
| `/api/auth/.well-known/openid-configuration` | OIDC discovery document |
| `/api/auth/.well-known/jwks.json` | Public keys for token verification |
| `/api/auth/oauth2/authorize` | Authorization endpoint |
| `/api/auth/oauth2/token` | Token endpoint |
| `/api/auth/oauth2/userinfo` | UserInfo endpoint |
| `/api/auth/oauth2/register` | Dynamic client registration |

## Authentication Flows

### Sign Up Flow

1. User navigates to `/signup`
2. Fills in name, email, and password
3. Form validates with Zod schema
4. Calls `signUpAction` server action
5. Better Auth creates user account in database
6. User redirected to `/verify-email` with success message
7. User clicks verification link in email
8. Email verified, redirected to `/signin` to log in

### Sign In Flow

1. User navigates to `/signin`
2. Enters email and password
3. Form validates credentials
4. Calls `signInAction` server action
5. Better Auth authenticates and sets session cookie
6. User redirected to dashboard or callback URL

### Social Login Flow

1. User clicks "Sign in with GitHub" or "Sign in with Google"
2. Redirected to `/api/auth/sign-in/{provider}?callbackURL=...`
3. Better Auth handles OAuth flow with provider
4. Better Auth sets session cookie
5. User redirected back to callback URL with active session

### OIDC Authorization Flow (External Apps)

1. External app redirects user to `/api/auth/oauth2/authorize?client_id=...&redirect_uri=...&response_type=code&scope=openid+profile+email&state=...`
2. SSO server checks if user is authenticated:
   - If not → redirects to `/login?client_id=...` (user signs in)
   - If yes → redirects to `/consent?client_id=...` (user grants permissions)
3. After consent, SSO server issues authorization code
4. Redirects back to external app with code
5. External app exchanges code for tokens at `/api/auth/oauth2/token`
6. External app can fetch user info from `/api/auth/oauth2/userinfo`

### Password Reset Flow

1. User clicks "Forgot Password?" on sign-in page
2. Enters email on `/forgot-password`
3. Generic success message displayed (prevents user enumeration)
4. User receives reset email with token link
5. Clicks link → redirected to `/reset-password?token=...`
6. Enters new password
7. Form validates and calls `resetPasswordAction`
8. Success message shown, auto-redirects to `/signin`

## Architecture

### Unified Application Design

This application was intentionally designed as a **unified SSO server** containing both:
- **Backend**: OIDC provider, authentication API, session management
- **Frontend**: Authentication UI pages, user dashboard

**Benefits of Unified Architecture:**
- ✅ Simpler deployment (one app instead of two)
- ✅ Faster API calls (no network overhead)
- ✅ No CORS issues (same origin)
- ✅ Better security (session cookies don't cross domains)
- ✅ Easier development and testing

### Directory Structure

```
apps/sso-server/
├── app/
│   ├── (auth)/              # Authentication pages
│   │   ├── layout.tsx       # Centered auth layout
│   │   ├── signup/          # Sign up page + form
│   │   ├── signin/          # Sign in page + form
│   │   ├── forgot-password/ # Password reset request
│   │   ├── reset-password/  # Password reset with token
│   │   ├── verify-email/    # Email verification
│   │   ├── login/           # OIDC login (external apps)
│   │   └── consent/         # OIDC consent (external apps)
│   ├── api/
│   │   └── auth/
│   │       └── [...all]/
│   │           └── route.ts # Better Auth API handler
│   ├── dashboard/           # User dashboard
│   ├── server/
│   │   └── auth.ts          # Server actions for auth
│   ├── layout.tsx           # Root layout
│   ├── page.tsx             # Home page
│   └── globals.css          # Global styles
├── components/
│   └── auth/
│       ├── captcha.tsx           # HCaptcha component
│       ├── form-error.tsx        # Error display
│       ├── password-input.tsx    # Password with show/hide
│       └── social-login-buttons.tsx  # OAuth buttons
├── lib/
│   ├── constants.ts         # Error messages, routes
│   ├── hooks/               # Custom React hooks
│   │   ├── use-captcha.ts
│   │   └── use-auth-redirect.txt
│   ├── schemas/
│   │   └── auth.ts          # Zod validation schemas
│   └── utils/
│       ├── api.ts           # Error handling, timeouts
│       ├── redirect.ts      # Redirect utilities
│       └── failed-attempts.ts  # Login attempt tracking
├── public/                  # Static assets
├── .env.local              # Environment variables
├── next.config.js          # Next.js configuration
├── tailwind.config.ts      # Tailwind configuration
└── package.json
```

### Server Actions

All authentication calls use Next.js Server Actions located in `app/server/auth.ts`:

- `signUpAction` - User registration
- `signInAction` - User authentication
- `forgetPasswordAction` - Request password reset
- `resetPasswordAction` - Reset password with token
- `verifyEmailAction` - Verify email with token
- `sendVerificationEmailAction` - Resend verification email

**Important**: Server actions use relative API paths (`''`) since the UI and backend are in the same application.

### Form Validation

Forms use Zod schemas for client-side validation (`lib/schemas/auth.ts`):

- `signUpSchema` - Name, email, password with complexity requirements
- `signInSchema` - Email and password
- `forgotPasswordSchema` - Email only
- `resetPasswordSchema` - New password and confirmation

### Error Handling

- **Timeout Protection**: 30-second timeout on all API calls
- **User-Friendly Messages**: Generic error messages prevent user enumeration
- **Field-Level Errors**: Inline validation errors for each form field
- **Network Errors**: Graceful handling of connection issues

### OIDC Provider Configuration

OIDC provider is configured in `packages/auth-config/index.ts`:

```typescript
oidcProvider({
  loginPage: '/login',      // UI page for user login
  consentPage: '/consent',  // UI page for user consent
  useJWTPlugin: true,       // Enable JWT signing
  allowDynamicClientRegistration: true,
})
```

## Security Features

- ✅ **CSRF Protection**: Via Better Auth
- ✅ **Secure Cookies**: httpOnly, sameSite, secure in production
- ✅ **Password Complexity**: Minimum requirements enforced
- ✅ **User Enumeration Prevention**: Generic error messages
- ✅ **Token Expiration**: Reset/verification links expire
- ✅ **JWT Signing**: RS256 asymmetric keys with JWKS
- ✅ **OAuth State Parameter**: CSRF protection for OAuth flows
- ✅ **CAPTCHA Support**: Optional, after failed login attempts

## Development

### Running Tests

```bash
# Type checking
pnpm type-check

# Linting
pnpm lint

# Format code
pnpm format
```

### Building for Production

```bash
pnpm build
pnpm start
```

### Testing OIDC Flow

See `API_TESTING_GUIDE.md` for detailed instructions on testing:
- User authentication
- OIDC client registration
- Authorization Code Flow
- Token exchange
- UserInfo endpoint

## Troubleshooting

### Database Connection Issues

If you see database errors:
1. Verify `DATABASE_URL` is correct in `.env.local`
2. Ensure PostgreSQL is running
3. Run `pnpm db:push` to create/update tables
4. Check database permissions

### Email Verification Not Working

If emails aren't being sent:
1. Check that `RESEND_API_KEY` is configured
2. Verify domain in Resend dashboard for production
3. For testing, only the Resend account email receives emails
4. Set `sendOnSignUp: false` in auth config to disable email verification during development

### Social Login Fails

1. Ensure GitHub/Google OAuth apps are configured
2. Check callback URLs match: `http://localhost:3000/api/auth/callback/{provider}`
3. Verify client IDs and secrets in `.env.local`
4. Check OAuth app is enabled in Better Auth config

### Session Not Persisting

1. Check that cookies are enabled in browser
2. Verify `better-auth.session_token` cookie is being set
3. Ensure `BETTER_AUTH_URL` matches your domain
4. Check for HTTPS issues (cookies with `secure` flag)

### OIDC Flow Returns 404

1. Verify all endpoints start with `/api/auth/` prefix
2. Check Better Auth is properly configured with OIDC plugin
3. Ensure `next.config.js` has proper rewrites for `.well-known` endpoints
4. Test discovery document: `curl http://localhost:3000/api/auth/.well-known/openid-configuration`

### Styling Issues

If UI components look unstyled:
1. Check `app/globals.css` imports from `@repo/ui/globals.css`
2. Verify `tailwind.config.ts` includes shadcn UI theme configuration
3. Ensure `@repo/ui` package is installed
4. Restart dev server to clear Turbopack cache

## API Testing

Use the provided testing guide:

```bash
# View OIDC endpoints
curl http://localhost:3000/api/auth/.well-known/openid-configuration

# Register a test OIDC client
curl -X POST http://localhost:3000/api/auth/oauth2/register \
  -H "Content-Type: application/json" \
  -d '{
    "client_name": "Test Client",
    "redirect_uris": ["http://localhost:3001/auth/callback"]
  }'
```

See `API_TESTING_GUIDE.md` and `OIDC_TESTING_STEPS.md` for complete workflows.

## Contributing

This is part of the SSO monorepo. See main README for contribution guidelines.

## License

Proprietary - Panaversity SSO Platform
