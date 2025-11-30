# Research: SSO Client Authentication Pages

**Feature**: 001-sso-client-auth-pages  
**Date**: 2025-11-23  
**Status**: Complete

## Research Questions

### 1. CAPTCHA Library Selection

**Question**: Which CAPTCHA solution should be used for brute-force protection after 5 failed login attempts?

**Decision**: **hCaptcha**

**Rationale**:
- Better privacy compliance (GDPR-friendly, doesn't track users like reCAPTCHA)
- Free tier supports up to 1M requests/month
- React component available (@hcaptcha/react-hcaptcha)
- Accessibility features built-in
- No Google dependency (important for some jurisdictions)

**Alternatives Considered**:
- **reCAPTCHA v3**: Rejected - invisible challenges but privacy concerns, Google dependency
- **Cloudflare Turnstile**: Good alternative but requires Cloudflare account/setup
- **reCAPTCHA v2**: Rejected - older UX, more intrusive checkbox

**Implementation Notes**:
- Add `@hcaptcha/react-hcaptcha` to sso-client dependencies
- Create reusable `<Captcha />` wrapper component
- Store hCaptcha site key in environment variable (`NEXT_PUBLIC_HCAPTCHA_SITE_KEY`)
- Backend validation required (verify token server-side)

---

### 2. Form State Management Pattern

**Question**: What's the best approach for managing form state, validation, and submission?

**Decision**: **react-hook-form + zod**

**Rationale**:
- Industry standard for React forms
- Minimal re-renders (performance)
- Built-in TypeScript support
- Seamless zod integration for schema validation
- Handles async validation and API errors elegantly
- Works well with shadcn/ui form components

**Alternatives Considered**:
- **Formik**: Rejected - more boilerplate, slower performance
- **Native React state**: Rejected - too much manual validation logic
- **Server Actions only**: Rejected - need client-side validation for UX

**Implementation Notes**:
- Create zod schemas in `lib/validations/auth.ts`
- Use `useForm` hook with zod resolver
- Validation rules: email (RFC 5322), password (min 8 chars, complexity)
- Handle both client-side and server-side errors

---

### 3. API Client Error Handling Pattern

**Question**: How should API errors and timeouts be handled across all auth forms?

**Decision**: **Centralized error handler with typed error responses**

**Rationale**:
- Consistent error messages across all auth pages
- Type-safe error handling
- Easy to display field-specific vs general errors
- Supports timeout after 30 seconds (per spec)

**Pattern**:
```typescript
// lib/utils/auth-helpers.ts
export async function handleAuthRequest<T>(
  promise: Promise<T>,
  options = { timeout: 30000 }
): Promise<{ data?: T; error?: string; fieldErrors?: Record<string, string> }> {
  try {
    const timeoutPromise = new Promise((_, reject) =>
      setTimeout(() => reject(new Error('Request timeout')), options.timeout)
    );
    const data = await Promise.race([promise, timeoutPromise]) as T;
    return { data };
  } catch (error) {
    if (error.message === 'Request timeout') {
      return { error: 'Request timed out. Please try again.' };
    }
    // Parse BetterAuth error responses
    return parseAuthError(error);
  }
}
```

---

### 4. Session Detection and Redirect Logic

**Question**: How to detect logged-in users and redirect them from auth pages?

**Decision**: **Client-side session check with middleware-like hook**

**Rationale**:
- authClient.getSession() provides session state
- Use hook pattern for reusability: `useAuthRedirect()`
- Check on component mount, redirect if authenticated
- Respects callback URL from query params

**Implementation**:
```typescript
// lib/hooks/use-redirect.ts
export function useAuthRedirect() {
  const router = useRouter();
  const searchParams = useSearchParams();
  
  useEffect(() => {
    authClient.getSession().then(({ data }) => {
      if (data?.session) {
        const callbackUrl = searchParams.get('callbackUrl');
        router.push(callbackUrl || '/dashboard');
      }
    });
  }, []);
}
```

---

### 5. Shadcn/UI Components to Add

**Question**: Which shadcn/ui components need to be installed in @repo/ui package?

**Decision**: Install the following components via shadcn CLI

**Required Components**:
1. **input** - Text inputs for email, password, name
2. **form** - Form wrapper with context (react-hook-form integration)
3. **label** - Accessible labels for form fields
4. **card** - Container for auth forms
5. **alert** - Error/success message display
6. **button** - Already exists, verify variants match design

**Installation Command**:
```bash
cd packages/ui
npx shadcn@latest add input form label card alert
```

**Implementation Notes**:
- Components auto-configured for Tailwind CSS
- Export from @repo/ui/components/* for tree-shaking
- Maintain consistent styling across sso-client and sso-admin

---

### 6. Email Validation Approach

**Question**: How should email validation be implemented (client + server)?

**Decision**: **Zod with RFC 5322 regex on client, BetterAuth handles server**

**Client-Side Validation**:
```typescript
// lib/validations/auth.ts
import { z } from 'zod';

export const emailSchema = z.string()
  .min(1, 'Email is required')
  .email('Invalid email format')
  .max(255, 'Email too long');

export const signUpSchema = z.object({
  email: emailSchema,
  password: z.string()
    .min(8, 'Password must be at least 8 characters')
    .regex(/[A-Z]/, 'Password must contain uppercase letter')
    .regex(/[a-z]/, 'Password must contain lowercase letter')
    .regex(/[0-9]/, 'Password must contain number')
    .regex(/[^A-Za-z0-9]/, 'Password must contain special character'),
  name: z.string().min(1, 'Name is required').max(100),
});
```

**Server-Side**: BetterAuth already validates per FR-002 (RFC 5322 standard)

---

### 7. Loading State Management

**Question**: How to show loading indicators during API calls?

**Decision**: **Local state with isSubmitting from react-hook-form**

**Pattern**:
```typescript
const { handleSubmit, formState: { isSubmitting } } = useForm();

// In component:
<Button type="submit" disabled={isSubmitting}>
  {isSubmitting ? 'Signing in...' : 'Sign in'}
</Button>
```

**Visual Feedback**:
- Disable form inputs during submission
- Show loading spinner on button
- Prevent double-submission
- Clear visual feedback within 100ms (per SC-013)

---

### 8. OAuth Redirect Pattern

**Question**: How should social login redirects be handled?

**Decision**: **Direct window.location redirect to backend OAuth endpoints**

**Rationale**:
- BetterAuth handles OAuth flow server-side
- Client just needs to redirect to `/api/auth/sign-in/github` or `/google`
- Backend sets session cookie and redirects back to client

**Implementation**:
```typescript
// components/auth/social-login-buttons.tsx
export function SocialLoginButtons() {
  const handleSocialLogin = (provider: 'github' | 'google') => {
    const callbackUrl = new URLSearchParams(window.location.search).get('callbackUrl');
    const redirectUrl = callbackUrl 
      ? `http://localhost:3000/api/auth/sign-in/${provider}?callbackUrl=${encodeURIComponent(callbackUrl)}`
      : `http://localhost:3000/api/auth/sign-in/${provider}`;
    
    window.location.href = redirectUrl;
  };

  return (
    <>
      <Button onClick={() => handleSocialLogin('github')}>
        Sign in with GitHub
      </Button>
      <Button onClick={() => handleSocialLogin('google')}>
        Sign in with Google
      </Button>
    </>
  );
}
```

---

## Technology Stack Summary

| Category | Technology | Version | Purpose |
|----------|-----------|---------|---------|
| Framework | Next.js | 16.0.3 | App Router, file-based routing |
| UI Library | React | 19.2.0 | Component framework |
| Language | TypeScript | 5.x | Type safety |
| Forms | react-hook-form | Latest | Form state management |
| Validation | zod | Latest | Schema validation |
| UI Components | shadcn/ui | Latest | Accessible components |
| Styling | Tailwind CSS | 3.4.x | Utility-first CSS |
| CAPTCHA | hCaptcha | Latest | Brute-force protection |
| API Client | @repo/auth-config/client | Workspace | BetterAuth client |
| Testing (Unit) | Jest + RTL | Latest | Component tests |
| Testing (E2E) | Playwright | Latest | Integration tests |

---

## Environment Variables Required

**sso-client (.env.local)**:
```env
NEXT_PUBLIC_API_URL=http://localhost:3000
NEXT_PUBLIC_HCAPTCHA_SITE_KEY=your_hcaptcha_site_key
```

**sso-server (.env)**:
```env
HCAPTCHA_SECRET_KEY=your_hcaptcha_secret_key
```

---

## Dependencies to Install

**sso-client**:
```bash
pnpm add react-hook-form zod @hookform/resolvers @hcaptcha/react-hcaptcha
pnpm add -D @testing-library/react @testing-library/jest-dom @playwright/test
```

**packages/ui**:
```bash
# Already has Button, add remaining components
cd packages/ui
npx shadcn@latest add input form label card alert
```

---

## Next Steps

1. ✅ Research complete - all NEEDS CLARIFICATION items resolved
2. → Proceed to Phase 1: Create data-model.md and contracts
3. → Generate quickstart.md for development setup
4. → Update agent context with technology choices
