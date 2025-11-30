---
id: 0004
title: Create Technical Implementation Plan for SSO Client Auth Pages
stage: plan
date: 2025-11-23
surface: agent
model: Claude Sonnet 4.5
feature: 001-sso-client-auth-pages
branch: 001-sso-client-auth-pages
user: ZohaibCodez
command: /sp.plan
labels: ["planning", "architecture", "research", "data-model", "contracts", "quickstart"]
links:
  spec: specs/001-sso-client-auth-pages/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-sso-client-auth-pages/plan.md
  - specs/001-sso-client-auth-pages/research.md
  - specs/001-sso-client-auth-pages/data-model.md
  - specs/001-sso-client-auth-pages/contracts/auth-api.ts
  - specs/001-sso-client-auth-pages/contracts/README.md
  - specs/001-sso-client-auth-pages/quickstart.md
  - .github/copilot-instructions.md
tests:
  - None (planning phase, no code implementation yet)
---

## Prompt

/sp.plan

## Response snapshot

Created comprehensive technical implementation plan with the following deliverables:

### Phase 0: Research (research.md)
- **CAPTCHA Solution**: Selected hCaptcha over reCAPTCHA and Cloudflare Turnstile
  - Privacy-friendly (GDPR compliant)
  - 1M free requests/month
  - No Google dependency
- **Form Management**: react-hook-form + zod validation
- **Error Handling**: Centralized handler with 30s timeout using Promise.race pattern
- **Session Detection**: useAuthRedirect() hook for redirecting logged-in users
- **Shadcn Components**: Identified 6 components to add (input, form, label, card, alert, button)
- **Email Validation**: Zod schemas with RFC 5322 regex
- **Loading States**: isSubmitting from react-hook-form
- **OAuth Redirects**: Direct window.location to backend endpoints

### Phase 1: Design Documents

#### data-model.md
Defined client-side data structures:
- **Form Models**: SignUpFormData, SignInFormData, ForgotPasswordFormData, ResetPasswordFormData
- **State Models**: CaptchaState, AuthResponse<T>, RedirectParams, ApiCallState<T>, FailedAttempts
- **Validation Rules**: 
  - Email: RFC 5322, 1-255 chars
  - Password (signup): 8-128 chars, 1 upper, 1 lower, 1 number, 1 special
  - Password (login): min 1 char (no complexity check)
  - Name: 1-100 chars, UTF-8
- **Error Handling**: Field-level and form-level error interfaces
- **State Transitions**: Flow diagrams for registration, login, password reset
- **Failed Attempts Tracking**: localStorage-based with 15-minute reset window, CAPTCHA trigger after 5 failures

#### contracts/auth-api.ts
Complete TypeScript API contracts:
- **Sign Up**: SignUpRequest → SignUpResponse
- **Sign In**: SignInRequest → SignInResponse
- **Forgot Password**: ForgotPasswordRequest → ForgotPasswordResponse
- **Reset Password**: ResetPasswordRequest → ResetPasswordResponse
- **Email Verification**: VerifyEmailRequest → VerifyEmailResponse
- **Session Management**: GetSessionResponse, SignOutResponse
- **OAuth**: Redirect-based flow (window.location)
- **OIDC**: OIDCAuthorizeQuery, ConsentRequest
- **Error Codes**: 13 standardized codes (EMAIL_ALREADY_EXISTS, INVALID_CREDENTIALS, TOKEN_EXPIRED, etc.)
- **Utilities**: handleAuthError(), withTimeout<T>() wrapper

#### quickstart.md
Developer onboarding guide:
- **Prerequisites**: Node.js 18.18+, pnpm 8+, PostgreSQL/Neon access
- **Setup Steps**: 
  1. pnpm install
  2. Configure .env.local (NEXT_PUBLIC_SSO_SERVER_URL, NEXT_PUBLIC_HCAPTCHA_SITE_KEY)
  3. Add shadcn/ui components (6 components)
  4. Install dependencies (react-hook-form, zod, @hcaptcha/react-hcaptcha)
- **Development Workflow**: Start servers, access pages, test API
- **Project Structure**: Detailed file tree for apps/sso-client
- **Common Tasks**: Add form field, customize error messages, style components
- **Debugging**: Session inspection, network requests, common issues table
- **Resources**: Links to BetterAuth, shadcn/ui, react-hook-form, Zod, hCaptcha docs

#### plan.md
Complete implementation plan:
- **Summary**: Build 7 auth pages (5 core + 2 OIDC) using Next.js 16 + React 19 + shadcn/ui
- **Technical Context**: Tech stack, dependencies, performance goals (<2s load, <200ms validation, 30s timeout)
- **Constitution Check**: No violations, follows best practices (test-first, accessibility, security, performance, simplicity)
- **Project Structure**: File tree for apps/sso-client with pages, components, lib, tests
- **Complexity Tracking**: No violations detected
- **Phase 0/1 Complete**: Research and design documents generated
- **Next Steps**: Run /sp.tasks to generate detailed task breakdown

### Agent Context Update
Updated .github/copilot-instructions.md with:
- Language: TypeScript 5.x with Next.js 16.0.3 App Router
- Database: N/A (client-side only)
- Project Type: Web frontend

## Outcome

- ✅ Impact: Complete technical plan with 6 design documents generated. All "NEEDS CLARIFICATION" items resolved. Ready for task breakdown phase.
- 🧪 Tests: No tests (planning phase)
- 📁 Files: 7 files created/updated (plan.md, research.md, data-model.md, 2 contracts files, quickstart.md, copilot-instructions.md)
- 🔁 Next prompts: `/sp.tasks` to generate detailed implementation tasks with acceptance criteria and test cases
- 🧠 Reflection: Planning phase successfully decomposed complex feature into clear technical decisions. Research phase resolved CAPTCHA library choice (hCaptcha), form management pattern (react-hook-form + zod), and error handling strategy (centralized with timeout). Data model provides type-safe form schemas with Zod. API contracts ensure compile-time safety. Quickstart guide enables rapid developer onboarding.

## Evaluation notes (flywheel)

- Failure modes observed: None - all Phase 0 and Phase 1 deliverables completed successfully
- Graders run and results (PASS/FAIL): PASS - All documents generated, agent context updated, no placeholder/NEEDS CLARIFICATION items remaining
- Prompt variant (if applicable): Standard /sp.plan workflow (setup → research → design)
- Next experiment (smallest change to try): Test /sp.tasks command to validate task generation from completed plan

