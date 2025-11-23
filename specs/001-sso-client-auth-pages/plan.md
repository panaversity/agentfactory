# Implementation Plan: SSO Client Authentication Pages

**Branch**: `001-sso-client-auth-pages` | **Date**: 2025-11-23 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-sso-client-auth-pages/spec.md`

## Summary

Build authentication UI pages for the SSO client application using Next.js 16, React 19, and shadcn/ui components. The pages will connect to existing BetterAuth backend endpoints (running on localhost:3000) via the authClient from @repo/auth-config/client package. This feature implements 5 core authentication pages (Sign Up, Sign In, Forgot Password, Reset Password, Email Verification) plus 2 OIDC pages (Login, Consent) for external application integration. All pages must be responsive, accessible, and handle loading/error states with 30-second API timeouts.

## Technical Context

**Language/Version**: TypeScript 5.x with Next.js 16.0.3 App Router  
**Primary Dependencies**: 
  - React 19.2.0
  - Next.js 16.0.3
  - @repo/ui (shadcn/ui components: Button, Input, Form, Card, Label, Alert)
  - @repo/auth-config/client (authClient for API calls)
  - react-hook-form (form state management)
  - zod (form validation schemas)
  - CAPTCHA library (NEEDS CLARIFICATION: hCaptcha vs reCAPTCHA vs Cloudflare Turnstile)

**Storage**: N/A (client-side only, session managed by backend cookies)  
**Testing**: Jest + React Testing Library for component tests, Playwright for E2E tests  
**Target Platform**: Web browsers (Chrome 90+, Firefox 88+, Safari 14+, Edge 90+)  
**Project Type**: Web frontend (Next.js App Router application)  
**Performance Goals**: 
  - Pages load and become interactive in <2s on 4G connections
  - Form validation feedback within 200ms
  - Visual feedback within 100ms of user interaction
  
**Constraints**: 
  - API calls timeout after 30 seconds
  - Must work with JavaScript enabled (no SSR fallback for auth forms)
  - Session cookies set by backend (httpOnly, secure in production)
  
**Scale/Scope**: 
  - 7 pages total (5 auth + 2 OIDC)
  - ~15 reusable form components
  - Integration with existing SSO server on port 3000

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Since no project-specific constitution file exists, applying general best practices:

- ✅ **Test-First Development**: Component tests will be written before implementation
- ✅ **Accessibility**: All forms keyboard-navigable, screen-reader friendly (WCAG 2.1 AA)
- ✅ **Security**: Input sanitization, HTTPS in production, CAPTCHA for brute-force protection
- ✅ **Performance**: Measured success criteria (<2s load, <200ms validation)
- ✅ **Simplicity**: Reusable components, clear separation of concerns (UI/logic/API)

**No violations detected** - proceeding with standard Next.js App Router patterns and shadcn/ui components.

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
sso-monorepo/
├── apps/
│   └── sso-client/              # This feature's primary location
│       ├── app/
│       │   ├── auth/
│       │   │   ├── signin/
│       │   │   │   └── page.tsx        # Sign In page
│       │   │   ├── signup/
│       │   │   │   └── page.tsx        # Sign Up page
│       │   │   ├── forgot-password/
│       │   │   │   └── page.tsx        # Forgot Password page
│       │   │   ├── reset-password/
│       │   │   │   └── page.tsx        # Reset Password page
│       │   │   ├── verify-email/
│       │   │   │   └── page.tsx        # Email Verification page
│       │   │   ├── login/
│       │   │   │   └── page.tsx        # OIDC Login page
│       │   │   └── consent/
│       │   │       └── page.tsx        # OIDC Consent page
│       │   ├── dashboard/
│       │   │   └── page.tsx            # Default redirect target
│       │   ├── layout.tsx
│       │   ├── page.tsx
│       │   └── globals.css
│       ├── components/
│       │   └── auth/
│       │       ├── signin-form.tsx      # Sign In form component
│       │       ├── signup-form.tsx      # Sign Up form component
│       │       ├── forgot-password-form.tsx
│       │       ├── reset-password-form.tsx
│       │       ├── social-login-buttons.tsx
│       │       ├── captcha.tsx          # CAPTCHA wrapper component
│       │       ├── auth-redirect.tsx    # Redirect handler for logged-in users
│       │       └── form-error.tsx       # Error display component
│       ├── lib/
│       │   ├── validations/
│       │   │   └── auth.ts             # Zod schemas for form validation
│       │   ├── hooks/
│       │   │   ├── use-auth.ts         # Auth state hook
│       │   │   └── use-redirect.ts     # Redirect logic hook
│       │   └── utils/
│       │       └── auth-helpers.ts     # Helper functions
│       └── __tests__/
│           ├── components/
│           │   └── auth/               # Component unit tests
│           └── e2e/
│               └── auth-flow.spec.ts   # E2E tests (Playwright)
│
├── packages/
│   ├── ui/                             # Existing shared UI components
│   │   └── components/
│       │       ├── button.tsx          # EXISTS
│       │       ├── input.tsx           # TO ADD
│       │       ├── form.tsx            # TO ADD
│       │       ├── card.tsx            # TO ADD
│       │       ├── label.tsx           # TO ADD
│       │       └── alert.tsx           # TO ADD
│   └── auth-config/                    # Existing auth configuration
│       └── client.ts                   # authClient - EXISTS
```

**Structure Decision**: Using Next.js 16 App Router with file-based routing. All authentication pages are under `/app/auth/` with dedicated route folders. Form components are extracted into `/components/auth/` for reusability and testability. Shadcn/ui components will be added to the shared `@repo/ui` package as they're needed across both sso-client and potentially sso-admin applications.

## Complexity Tracking

No constitution violations detected. Standard Next.js App Router patterns with shadcn/ui components align with simplicity and reusability principles.

---

## Phase 0: Research Complete ✓

**Research Decisions Documented**: `specs/001-sso-client-auth-pages/research.md`

**Key Decisions**:
1. **CAPTCHA**: hCaptcha (privacy-friendly, 1M free requests/month)
2. **Form Management**: react-hook-form + zod for validation
3. **Error Handling**: Centralized handler with 30s timeout using Promise.race
4. **Session Detection**: useAuthRedirect() hook to redirect logged-in users
5. **Shadcn Components**: input, form, label, card, alert to be added
6. **Email Validation**: Zod schemas with RFC 5322 regex
7. **Loading States**: isSubmitting from react-hook-form
8. **OAuth Redirects**: Direct window.location to backend endpoints

**Technology Stack Summary**:
- Frontend: Next.js 16 + React 19 + TypeScript 5.x
- UI: shadcn/ui + Tailwind CSS
- Forms: react-hook-form + zod + @hookform/resolvers
- CAPTCHA: @hcaptcha/react-hcaptcha
- Backend Integration: @repo/auth-config/client (authClient)

---

## Phase 1: Design Documents ✓

### Data Model

**Document**: `specs/001-sso-client-auth-pages/data-model.md`

**Key Entities**:
- SignUpFormData (email, password, name)
- SignInFormData (email, password, rememberMe)
- ForgotPasswordFormData (email)
- ResetPasswordFormData (token, newPassword, confirmPassword)
- CaptchaState (required, token, verified)
- AuthResponse<T> (generic API response shape)
- RedirectParams (callbackUrl, defaultRedirect)
- ApiCallState<T> (loading, data, error, timestamp)
- FailedAttempts (count, lastAttempt, resetAt)

**Validation Rules**:
- Email: RFC 5322 format, 1-255 chars, lowercase normalized
- Password (signup): 8-128 chars, 1 upper, 1 lower, 1 number, 1 special
- Password (login): minimum 1 char (no complexity check on login)
- Name: 1-100 chars, UTF-8 allowed
- Token: Server-side validation only

### API Contracts

**Document**: `specs/001-sso-client-auth-pages/contracts/auth-api.ts`

**Endpoints**:
- `POST /api/auth/sign-up/email` → SignUpResponse
- `POST /api/auth/sign-in/email` → SignInResponse
- `POST /api/auth/forget-password` → ForgotPasswordResponse
- `POST /api/auth/reset-password` → ResetPasswordResponse
- `GET /api/auth/verify-email` → VerifyEmailResponse
- `GET /api/auth/session` → GetSessionResponse
- `POST /api/auth/sign-out` → SignOutResponse
- OAuth redirects (window.location, not fetch)
- OIDC authorize + consent flows

**Error Codes**:
- EMAIL_ALREADY_EXISTS, INVALID_EMAIL, WEAK_PASSWORD
- INVALID_CREDENTIALS, EMAIL_NOT_VERIFIED, ACCOUNT_LOCKED
- INVALID_TOKEN, TOKEN_EXPIRED
- NETWORK_ERROR, TIMEOUT, UNKNOWN_ERROR

**Timeout Handling**: `withTimeout<T>(promise, 30000)` wrapper

### Quickstart Guide

**Document**: `specs/001-sso-client-auth-pages/quickstart.md`

**Setup Steps**:
1. Install dependencies: `pnpm install`
2. Configure `.env.local` with `NEXT_PUBLIC_SSO_SERVER_URL` and `NEXT_PUBLIC_HCAPTCHA_SITE_KEY`
3. Add shadcn/ui components: input, form, label, card, alert
4. Install packages: react-hook-form, zod, @hookform/resolvers, @hcaptcha/react-hcaptcha
5. Start dev servers: `pnpm --filter sso-server --filter sso-client dev`

**Development Workflow**:
- Access pages at http://localhost:3001/signin, /signup, etc.
- Test backend with PowerShell `Invoke-RestMethod`
- Run tests: `pnpm test`, `pnpm test:watch`
- Debug with browser DevTools Network tab

### Agent Context Update

**Status**: ✓ Completed

Updated `.github/copilot-instructions.md` with:
- Language: TypeScript 5.x with Next.js 16.0.3 App Router
- Database: N/A (client-side only, session managed by backend cookies)
- Project Type: Web frontend (Next.js App Router application)

---

## Next Steps

### For Implementation (Run `/sp.tasks`)

The task generation command will create `tasks.md` with:
- Detailed implementation tasks for each page
- Test cases (unit, integration, E2E)
- Acceptance criteria per task
- Task dependencies and order

### Quick Task Preview (Generated by `/sp.tasks`)

1. **Setup & Dependencies**
   - Add shadcn/ui components to @repo/ui
   - Install npm packages (react-hook-form, zod, hCaptcha)
   - Configure environment variables

2. **Shared Components & Utilities**
   - Create Zod validation schemas (`lib/validations/auth.ts`)
   - Build reusable hooks (useAuthRedirect, useCaptcha)
   - Create shared components (FormError, AuthRedirect, CaptchaWrapper)

3. **Sign In Page** (Priority: HIGH)
   - Page component with routing
   - Form component with validation
   - CAPTCHA integration after 5 failed attempts
   - Tests (unit + E2E)

4. **Sign Up Page** (Priority: HIGH)
   - Page component with routing
   - Form component with password complexity validation
   - Duplicate email handling
   - Tests (unit + E2E)

5. **Forgot Password Page** (Priority: MEDIUM)
   - Page component with routing
   - Form component with success message (no user enumeration)
   - Tests (unit + E2E)

6. **Reset Password Page** (Priority: MEDIUM)
   - Page component with token validation
   - Form component with password confirmation
   - Expired token handling
   - Tests (unit + E2E)

7. **Email Verification Page** (Priority: MEDIUM)
   - Page component with token validation
   - Success/error states
   - Auto-redirect after verification
   - Tests (unit + E2E)

8. **OIDC Login Page** (Priority: LOW)
   - Page component with client_id validation
   - Redirect to consent if not logged in
   - Auto-approve for trusted clients
   - Tests (unit + E2E)

9. **Consent Page** (Priority: LOW)
   - Page component with scope display
   - Grant/deny buttons
   - Authorization code redirect
   - Tests (unit + E2E)

10. **Final Integration & Polish**
    - Cross-browser testing
    - Accessibility audit (WCAG 2.1 AA)
    - Performance testing (Lighthouse)
    - Documentation updates

---

## Design Principles Applied

1. **Separation of Concerns**: Pages contain routing logic; form components handle UI and validation; utilities manage API calls and errors.
2. **Type Safety**: Zod schemas provide both runtime validation and compile-time TypeScript types (via `z.infer<>`).
3. **Reusability**: Shared components in @repo/ui package; form logic extracted into hooks.
4. **Error Handling**: Centralized error handler with timeout wrapper; consistent error message display.
5. **Accessibility**: Semantic HTML, ARIA labels, keyboard navigation, screen reader support.
6. **Security**: CAPTCHA for brute-force protection, input sanitization, HTTPS in production, httpOnly cookies.
7. **Performance**: Code splitting via Next.js App Router, lazy loading for CAPTCHA, optimized images/fonts.

---

## Planning Complete

**Generated Artifacts**:
- ✓ `specs/001-sso-client-auth-pages/plan.md` (this file)
- ✓ `specs/001-sso-client-auth-pages/research.md`
- ✓ `specs/001-sso-client-auth-pages/data-model.md`
- ✓ `specs/001-sso-client-auth-pages/contracts/auth-api.ts`
- ✓ `specs/001-sso-client-auth-pages/contracts/README.md`
- ✓ `specs/001-sso-client-auth-pages/quickstart.md`
- ✓ Updated `.github/copilot-instructions.md` with technology stack

**Run Next Command**:
```
/sp.tasks
```

This will generate `tasks.md` with detailed implementation tasks, acceptance criteria, and test cases for each authentication page.
