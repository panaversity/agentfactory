# Quickstart Guide: SSO Client Authentication Pages

**Feature**: 001-sso-client-auth-pages  
**Last Updated**: 2025-11-23

This guide helps developers set up and start working on the SSO client authentication UI pages.

---

## Prerequisites

Ensure you have completed the monorepo setup:

1. **Node.js**: v18.18.0+ (recommended: v20.x)
2. **pnpm**: v8.0.0+ (`npm install -g pnpm`)
3. **Git**: Latest version
4. **PostgreSQL**: Access to Neon database (or local Postgres for dev)

---

## Initial Setup

### 1. Clone and Install

```powershell
# Clone repository (if not already done)
git clone <repository-url>
cd sso

# Install all dependencies (monorepo-wide)
pnpm install
```

### 2. Environment Variables

Create `.env.local` in `apps/sso-client/`:

```env
# SSO Server URL (backend)
NEXT_PUBLIC_SSO_SERVER_URL=http://localhost:3000

# hCaptcha (for brute-force protection)
NEXT_PUBLIC_HCAPTCHA_SITE_KEY=your-hcaptcha-site-key

# Optional: Analytics, monitoring
NEXT_PUBLIC_ANALYTICS_ID=
```

**Get hCaptcha keys**:
1. Sign up at https://www.hcaptcha.com/
2. Create a new site
3. Copy Site Key to `.env.local`
4. Copy Secret Key to `apps/sso-server/.env.local` (backend needs it for verification)

### 3. Install shadcn/ui Components

The following components need to be added to `packages/ui`:

```powershell
# Navigate to ui package
cd packages/ui

# Add required components
pnpm dlx shadcn@latest add input
pnpm dlx shadcn@latest add form
pnpm dlx shadcn@latest add label
pnpm dlx shadcn@latest add card
pnpm dlx shadcn@latest add alert
pnpm dlx shadcn@latest add button  # Should already exist

# Return to monorepo root
cd ../..
```

### 4. Install Additional Dependencies

```powershell
# Navigate to sso-client app
cd apps/sso-client

# Install form and validation libraries
pnpm add react-hook-form zod @hookform/resolvers

# Install hCaptcha React component
pnpm add @hcaptcha/react-hcaptcha

# Install type definitions
pnpm add -D @types/react-hook-form

# Return to monorepo root
cd ../..
```

---

## Development Workflow

### Start Development Servers

```powershell
# Option 1: Run all apps (from monorepo root)
pnpm dev

# Option 2: Run only sso-client (from monorepo root)
pnpm --filter sso-client dev

# Option 3: Run sso-server + sso-client (recommended for this feature)
pnpm --filter sso-server --filter sso-client dev
```

**Ports**:
- SSO Server (backend): http://localhost:3000
- SSO Client (frontend): http://localhost:3001
- SSO Admin: http://localhost:3002

### Access Authentication Pages

| Page | URL | Purpose |
|------|-----|---------|
| Sign Up | http://localhost:3001/signup | New user registration |
| Sign In | http://localhost:3001/signin | Existing user login |
| Forgot Password | http://localhost:3001/forgot-password | Request password reset |
| Reset Password | http://localhost:3001/reset-password?token=... | Set new password |
| Verify Email | http://localhost:3001/verify-email?token=... | Confirm email address |
| OIDC Login | http://localhost:3001/login?client_id=... | Third-party app login |
| Consent | http://localhost:3001/consent?client_id=... | Grant permissions |

---

## Project Structure

```
apps/sso-client/
├── app/
│   ├── (auth)/                    # Auth route group (shared layout)
│   │   ├── layout.tsx             # Centered layout for auth pages
│   │   ├── signin/
│   │   │   ├── page.tsx           # Sign in page
│   │   │   └── signin-form.tsx    # Sign in form component
│   │   ├── signup/
│   │   │   ├── page.tsx
│   │   │   └── signup-form.tsx
│   │   ├── forgot-password/
│   │   │   ├── page.tsx
│   │   │   └── forgot-password-form.tsx
│   │   ├── reset-password/
│   │   │   ├── page.tsx
│   │   │   └── reset-password-form.tsx
│   │   ├── verify-email/
│   │   │   └── page.tsx
│   │   ├── login/
│   │   │   └── page.tsx           # OIDC login page
│   │   └── consent/
│   │       ├── page.tsx
│   │       └── consent-form.tsx
│   ├── lib/
│   │   ├── schemas/               # Zod validation schemas
│   │   │   ├── auth.ts
│   │   │   └── index.ts
│   │   ├── hooks/                 # Custom React hooks
│   │   │   ├── useAuthRedirect.ts # Redirect logged-in users
│   │   │   └── useCaptcha.ts      # hCaptcha state management
│   │   ├── utils/
│   │   │   ├── api.ts             # API helpers (withTimeout, handleError)
│   │   │   └── redirect.ts        # Redirect URL logic
│   │   └── constants.ts           # Error messages, routes
│   └── __tests__/                 # Jest/React Testing Library tests
│       ├── signin.test.tsx
│       ├── signup.test.tsx
│       └── ...
└── package.json
```

---

## Key Files to Work With

### 1. Zod Schemas (`app/lib/schemas/auth.ts`)

```typescript
import { z } from 'zod';

export const signInSchema = z.object({
  email: z.string().email('Invalid email format'),
  password: z.string().min(1, 'Password is required'),
  rememberMe: z.boolean().optional().default(false),
});

export type SignInFormData = z.infer<typeof signInSchema>;
```

### 2. Form Component (`app/(auth)/signin/signin-form.tsx`)

```typescript
'use client';

import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { signInSchema, type SignInFormData } from '@/lib/schemas/auth';
import { authClient } from '@repo/auth-config/client';

export function SignInForm() {
  const { register, handleSubmit, formState: { errors, isSubmitting } } = useForm<SignInFormData>({
    resolver: zodResolver(signInSchema),
  });

  const onSubmit = async (data: SignInFormData) => {
    const { data: result, error } = await authClient.signIn.email(data);
    if (error) {
      // Handle error
    } else {
      // Redirect to dashboard or callbackUrl
    }
  };

  return (
    <form onSubmit={handleSubmit(onSubmit)}>
      {/* Form fields */}
    </form>
  );
}
```

### 3. Custom Hook (`app/lib/hooks/useAuthRedirect.ts`)

```typescript
'use client';

import { useEffect } from 'react';
import { useRouter } from 'next/navigation';
import { authClient } from '@repo/auth-config/client';

export function useAuthRedirect() {
  const router = useRouter();

  useEffect(() => {
    async function checkSession() {
      const { data } = await authClient.getSession();
      if (data?.session) {
        router.replace('/dashboard');
      }
    }
    checkSession();
  }, [router]);
}
```

---

## Testing

### Run Tests

```powershell
# Run all tests in sso-client
cd apps/sso-client
pnpm test

# Run tests in watch mode
pnpm test:watch

# Run tests with coverage
pnpm test:coverage
```

### Test Structure

```typescript
// __tests__/signin.test.tsx
import { render, screen, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { SignInForm } from '@/app/(auth)/signin/signin-form';

describe('SignInForm', () => {
  it('validates email format', async () => {
    render(<SignInForm />);
    const emailInput = screen.getByLabelText(/email/i);
    await userEvent.type(emailInput, 'invalid-email');
    await userEvent.tab(); // Trigger blur
    
    expect(await screen.findByText('Invalid email format')).toBeInTheDocument();
  });
});
```

---

## Common Tasks

### Add a New Form Field

1. Update Zod schema in `app/lib/schemas/auth.ts`
2. Add field to form component
3. Update TypeScript types (auto-inferred from Zod)
4. Add validation tests

### Customize Error Messages

Edit `app/lib/constants.ts`:

```typescript
export const ERROR_MESSAGES = {
  INVALID_CREDENTIALS: 'Invalid email or password',
  EMAIL_EXISTS: 'An account with this email already exists',
  // ... add custom messages
} as const;
```

### Style Components

Uses Tailwind CSS from `app/globals.css`. Follow existing patterns:

```tsx
<Button 
  type="submit" 
  disabled={isSubmitting}
  className="w-full"
>
  {isSubmitting ? 'Signing in...' : 'Sign In'}
</Button>
```

---

## API Testing

### Test Backend Endpoints

See `sso-monorepo/API_TESTING_GUIDE.md` for full details.

Quick test with PowerShell:

```powershell
# Sign up
Invoke-RestMethod -Uri "http://localhost:3000/api/auth/sign-up/email" `
  -Method POST `
  -Headers @{"Content-Type"="application/json"} `
  -Body '{"email":"test@example.com","password":"Test123!@#","name":"Test User"}'

# Sign in
Invoke-RestMethod -Uri "http://localhost:3000/api/auth/sign-in/email" `
  -Method POST `
  -Headers @{"Content-Type"="application/json"} `
  -Body '{"email":"test@example.com","password":"Test123!@#"}'
```

---

## Debugging

### View Session Data

```typescript
const { data } = await authClient.getSession();
console.log('Current session:', data);
```

### Check Network Requests

1. Open browser DevTools (F12)
2. Go to Network tab
3. Submit form
4. Inspect request/response to `http://localhost:3000/api/auth/*`

### Common Issues

| Issue | Solution |
|-------|----------|
| "Cannot find module '@repo/auth-config'" | Run `pnpm install` in monorepo root |
| CORS errors | Ensure `NEXT_PUBLIC_SSO_SERVER_URL` is correct in `.env.local` |
| Session not persisting | Check that cookies are enabled, server is setting httpOnly cookies |
| hCaptcha not loading | Verify `NEXT_PUBLIC_HCAPTCHA_SITE_KEY` in `.env.local` |

---

## Next Steps

1. **Read Specification**: `specs/001-sso-client-auth-pages/spec.md`
2. **Review Plan**: `specs/001-sso-client-auth-pages/plan.md`
3. **Check Data Models**: `specs/001-sso-client-auth-pages/data-model.md`
4. **Read API Contracts**: `specs/001-sso-client-auth-pages/contracts/auth-api.ts`
5. **Start Coding**: Begin with `/signin` page (highest priority)
6. **Write Tests**: Follow TDD - write test first, then implementation
7. **Run Tasks**: Use `/sp.tasks` to generate detailed task breakdown

---

## Resources

- **BetterAuth Docs**: https://www.better-auth.com/docs
- **shadcn/ui**: https://ui.shadcn.com/
- **React Hook Form**: https://react-hook-form.com/
- **Zod**: https://zod.dev/
- **hCaptcha**: https://docs.hcaptcha.com/
- **Next.js 15 Docs**: https://nextjs.org/docs

---

## Support

For questions or issues:
1. Check `specs/001-sso-client-auth-pages/plan.md` for architecture decisions
2. Review `sso-monorepo/README.md` for monorepo setup
3. Consult `sso-monorepo/API_TESTING_GUIDE.md` for backend testing
4. Ask on team Slack channel
