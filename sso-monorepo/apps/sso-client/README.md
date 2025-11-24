# SSO Client - Authentication Pages

Modern authentication UI for the SSO platform built with Next.js 14, React 19, and TypeScript.

## Features

- ✅ **Email/Password Authentication**: Sign up and sign in with email
- ✅ **Social Login**: GitHub and Google OAuth integration
- ✅ **Password Reset**: Forgot password flow with email verification
- ✅ **Email Verification**: Verify email addresses with token-based links
- ✅ **OIDC Support**: OpenID Connect integration for external applications
- ✅ **Responsive Design**: Mobile-first design with Tailwind CSS
- ✅ **Accessibility**: WCAG compliant with proper ARIA labels
- ✅ **Form Validation**: Client-side validation with Zod schemas
- ✅ **Error Handling**: Comprehensive error messages and timeout handling

## Tech Stack

- **Framework**: Next.js 14 (App Router)
- **UI**: React 19, Tailwind CSS, shadcn/ui
- **Forms**: React Hook Form with Zod validation
- **Icons**: Lucide React
- **Authentication**: Better Auth (server-side)

## Getting Started

### Prerequisites

- Node.js 18+ or Bun
- pnpm (recommended) or npm
- Running SSO server at `http://localhost:3000`

### Environment Variables

Create a `.env.local` file in the `apps/sso-client` directory:

```env
# SSO Server URL
NEXT_PUBLIC_SSO_SERVER_URL=http://localhost:3000

# Optional: HCaptcha for CAPTCHA protection
NEXT_PUBLIC_HCAPTCHA_SITE_KEY=your_site_key_here
```

### Installation

From the monorepo root:

```bash
# Install dependencies
pnpm install

# Start development server
pnpm dev
```

The client will be available at `http://localhost:3001`

## Available Pages

### Authentication Pages

| Route | Description |
|-------|-------------|
| `/signup` | User registration with email and password |
| `/signin` | User authentication |
| `/forgot-password` | Request password reset email |
| `/reset-password?token=...` | Reset password with token |
| `/verify-email` | Email verification success message |
| `/verify-email/verify?token=...` | Email verification token handler |
| `/login?client_id=...` | OIDC login page |
| `/consent?client_id=...` | OIDC consent page |

### Application Pages

| Route | Description |
|-------|-------------|
| `/dashboard` | User dashboard (redirect target after auth) |

## Authentication Flow

### Sign Up Flow

1. User navigates to `/signup`
2. Fills in name, email, and password
3. Form validates with Zod schema
4. Calls `signUpAction` server action
5. Backend creates user account
6. User redirected to `/verify-email` with success message
7. User clicks verification link in email
8. Redirected to `/signin` to log in

### Sign In Flow

1. User navigates to `/signin`
2. Enters email and password
3. Form validates credentials
4. Calls `signInAction` server action
5. Backend authenticates and sets session cookie
6. User redirected to dashboard or callback URL

### Social Login Flow

1. User clicks "Sign in with GitHub" or "Sign in with Google"
2. Redirected to `${SSO_SERVER_URL}/api/auth/sign-in/{provider}?callbackURL=...`
3. Backend handles OAuth flow with provider
4. Backend sets session cookie
5. User redirected back to callback URL with active session

**Note**: OAuth callback is handled entirely by the backend. The client only initiates the flow.

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

### Directory Structure

```
apps/sso-client/
├── app/
│   ├── (auth)/              # Authentication pages
│   │   ├── layout.tsx       # Centered auth layout
│   │   ├── signup/          # Sign up page
│   │   ├── signin/          # Sign in page
│   │   ├── forgot-password/ # Password reset request
│   │   ├── reset-password/  # Password reset with token
│   │   ├── verify-email/    # Email verification
│   │   ├── login/           # OIDC login
│   │   └── consent/         # OIDC consent
│   ├── dashboard/           # User dashboard
│   ├── server/
│   │   └── auth.ts          # Server actions for auth
│   ├── layout.tsx
│   └── page.tsx
├── components/
│   └── auth/
│       ├── form-error.tsx   # Error display component
│       └── social-login-buttons.tsx  # OAuth buttons
├── lib/
│   ├── constants.ts         # Error messages, routes
│   ├── hooks/               # Custom React hooks
│   ├── schemas/
│   │   └── auth.ts          # Zod validation schemas
│   └── utils/
│       ├── api.ts           # Error handling, timeouts
│       ├── redirect.ts      # Redirect utilities
│       └── failed-attempts.ts  # Login attempt tracking
└── public/
```

### Server Actions

All authentication calls use Next.js Server Actions located in `app/server/auth.ts`:

- `signUpAction` - User registration
- `signInAction` - User authentication
- `forgetPasswordAction` - Request password reset
- `resetPasswordAction` - Reset password with token
- `verifyEmailAction` - Verify email with token
- `sendVerificationEmailAction` - Resend verification email

### Form Validation

Forms use Zod schemas for client-side validation:

- `signUpSchema` - Name, email, password with complexity requirements
- `signInSchema` - Email and password
- `forgotPasswordSchema` - Email only
- `resetPasswordSchema` - New password and confirmation

### Error Handling

- **Timeout Protection**: 30-second timeout on all API calls
- **User-Friendly Messages**: Generic error messages prevent user enumeration
- **Field-Level Errors**: Inline validation errors for each form field
- **Network Errors**: Graceful handling of connection issues

## Security Features

- ✅ CSRF protection via Better Auth
- ✅ Secure session cookies (httpOnly, sameSite)
- ✅ Password complexity requirements
- ✅ User enumeration prevention
- ✅ Token expiration for reset/verification links
- ✅ CAPTCHA support (optional, after failed login attempts)

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
```

### Environment-Specific Configuration

Development and production environments automatically use different configurations:

- **Development**: `autoSignIn: true`, `requireEmailVerification: false`
- **Production**: Configure in backend `packages/auth-config/index.ts`

## Troubleshooting

### Email Verification Not Working

If emails aren't being sent:
1. Check that Resend API key is configured in backend
2. Verify domain in Resend dashboard for production
3. For testing, only the Resend account email receives emails
4. Set `sendOnSignUp: false` in backend to disable email verification

### Social Login Fails

1. Ensure GitHub/Google OAuth apps are configured in backend
2. Check callback URLs match your domain
3. Verify `NEXT_PUBLIC_SSO_SERVER_URL` is correct

### Session Not Persisting

1. Check that cookies are enabled in browser
2. Ensure backend and client are on same domain (or proper CORS configured)
3. Verify `better-auth.session_token` cookie is being set

## Contributing

This is part of the SSO monorepo. See main README for contribution guidelines.

## License

Proprietary - Panaversity SSO Platform
