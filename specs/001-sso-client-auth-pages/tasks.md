# Tasks: SSO Client Authentication Pages

**Input**: Design documents from `/specs/001-sso-client-auth-pages/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Tests are NOT required for this feature (not specified in requirements)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4, US5)
- Include exact file paths in descriptions

## Path Conventions

All paths are relative to monorepo: `apps/sso-client/` for client code, `packages/ui/` for shared components

---

## Phase 1: Setup (Shared Infrastructure) ✅

**Purpose**: Project initialization and dependency installation

- [X] T001 Install npm dependencies in apps/sso-client: `pnpm add react-hook-form zod @hookform/resolvers @hcaptcha/react-hcaptcha`
- [X] T002 [P] Add shadcn/ui input component to packages/ui: `pnpm dlx shadcn@latest add input`
- [X] T003 [P] Add shadcn/ui form component to packages/ui: `pnpm dlx shadcn@latest add form`
- [X] T004 [P] Add shadcn/ui label component to packages/ui: `pnpm dlx shadcn@latest add label`
- [X] T005 [P] Add shadcn/ui card component to packages/ui: `pnpm dlx shadcn@latest add card`
- [X] T006 [P] Add shadcn/ui alert component to packages/ui: `pnpm dlx shadcn@latest add alert`
- [X] T007 Configure environment variables in apps/sso-client/.env.local with NEXT_PUBLIC_SSO_SERVER_URL and NEXT_PUBLIC_HCAPTCHA_SITE_KEY
- [X] T008 Create apps/sso-client/app/(auth)/layout.tsx for centered authentication page layout

**Checkpoint**: ✅ Dependencies installed, shadcn components available, environment configured

---

## Phase 2: Foundational (Blocking Prerequisites) ✅

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T009 Create Zod validation schemas in apps/sso-client/lib/schemas/auth.ts (signUpSchema, signInSchema, forgotPasswordSchema, resetPasswordSchema)
- [X] T010 [P] Create error handling utilities in apps/sso-client/lib/utils/api.ts (withTimeout function, handleAuthError function, AUTH_ERROR_CODES constant)
- [X] T011 [P] Create redirect utilities in apps/sso-client/lib/utils/redirect.ts (getRedirectUrl function to handle callbackUrl query parameter)
- [X] T012 [P] Create failed attempts tracking utilities in apps/sso-client/lib/utils/failed-attempts.ts (getFailedAttempts, incrementFailedAttempts, clearFailedAttempts, shouldShowCaptcha functions using localStorage)
- [X] T013 [P] Create FormError component in apps/sso-client/components/auth/form-error.tsx (displays form-level errors using Alert component from @repo/ui)
- [X] T014 [P] Create useAuthRedirect hook in apps/sso-client/lib/hooks/use-auth-redirect.ts (checks session with authClient.getSession, redirects logged-in users to dashboard or callbackUrl)
- [X] T015 [P] Create useCaptcha hook in apps/sso-client/lib/hooks/use-captcha.ts (manages CAPTCHA state: required, token, verified)
- [X] T016 [P] Create Captcha wrapper component in apps/sso-client/components/auth/captcha.tsx (wraps @hcaptcha/react-hcaptcha with consistent styling)
- [X] T017 Create constants file in apps/sso-client/lib/constants.ts (ERROR_MESSAGES object with all error message strings, ROUTES object with page paths)

**Checkpoint**: ✅ Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - User Registration with Email and Password (Priority: P1) 🎯 MVP ✅

**Goal**: Enable new users to create accounts using email, password, and name, with proper validation and redirect to dashboard or callback URL

**Independent Test**: Navigate to http://localhost:3001/signup, fill in valid email/password/name, submit form, verify account creation and redirect to dashboard

### Implementation for User Story 1

- [X] T018 [P] [US1] Create signup page component in apps/sso-client/app/(auth)/signup/page.tsx (imports SignUpForm, uses useAuthRedirect hook)
- [X] T019 [US1] Create SignUpForm component in apps/sso-client/app/(auth)/signup/signup-form.tsx (uses react-hook-form with signUpSchema, calls authClient.signUp.email, handles loading/error states, redirects on success)
- [X] T020 [US1] Implement email already exists error handling in SignUpForm (display fieldErrors from API response, show EMAIL_ALREADY_EXISTS message)
- [X] T021 [US1] Implement weak password validation feedback in SignUpForm (client-side validation with Zod, display inline errors for each password requirement)
- [X] T022 [US1] Implement invalid email format validation in SignUpForm (Zod email validation, display inline error below email field)
- [X] T023 [US1] Add loading spinner to submit button in SignUpForm (use isSubmitting from react-hook-form, disable button and show loading text)
- [X] T024 [US1] Add link to sign-in page in apps/sso-client/app/(auth)/signup/page.tsx (below form: "Already have an account? Sign in")
- [X] T025 [US1] Implement redirect logic in SignUpForm (use getRedirectUrl utility, redirect to callbackUrl or /dashboard after successful registration)

**Checkpoint**: ✅ User Story 1 complete - users can register with email/password and are redirected appropriately

---

## Phase 4: User Story 2 - User Sign In with Email and Password (Priority: P1) 🎯 MVP

**Goal**: Enable returning users to authenticate using email and password, with CAPTCHA after 5 failed attempts, remember me option, and redirect to dashboard or callback URL

**Independent Test**: Navigate to http://localhost:3001/signin, enter valid credentials, verify authentication and redirect to dashboard

### Implementation for User Story 2

- [X] T026 [P] [US2] Create signin page component in apps/sso-client/app/(auth)/signin/page.tsx (imports SignInForm, uses useAuthRedirect hook)
- [X] T027 [US2] Create SignInForm component in apps/sso-client/app/(auth)/signin/signin-form.tsx (uses react-hook-form with signInSchema, calls authClient.signIn.email, handles loading/error states, redirects on success)
- [X] T028 [US2] Implement invalid credentials error handling in SignInForm (display INVALID_CREDENTIALS message without user enumeration, increment failed attempts counter)
- [ ] T029 [US2] Implement remember me checkbox in SignInForm (add rememberMe field to form, pass to authClient.signIn.email)
- [ ] T030 [US2] Implement CAPTCHA challenge after 5 failed attempts in SignInForm (use useCaptcha hook, check shouldShowCaptcha utility, display Captcha component, pass token to API)
- [ ] T031 [US2] Implement failed attempt counter reset on successful login in SignInForm (call clearFailedAttempts from failed-attempts.ts)
- [X] T032 [US2] Add "Forgot Password?" link in apps/sso-client/app/(auth)/signin/page.tsx (link to /auth/forgot-password below password field)
- [X] T033 [US2] Add link to sign-up page in apps/sso-client/app/(auth)/signin/page.tsx (below form: "Don't have an account? Sign up")
- [X] T034 [US2] Implement redirect logic in SignInForm (use getRedirectUrl utility, redirect to callbackUrl or /dashboard after successful sign-in)
- [X] T035 [US2] Implement API timeout handling in SignInForm (use withTimeout wrapper with 30s timeout, display TIMEOUT error message with retry option)

**Checkpoint**: User Story 2 complete - users can sign in with email/password, CAPTCHA protection active, redirect working

---

## Phase 5: User Story 3 - Password Reset Flow (Priority: P2)

**Goal**: Enable users who forgot their password to reset it via email, with time-limited tokens and proper error handling for expired links

**Independent Test**: Navigate to http://localhost:3001/forgot-password, enter email, verify success message, click reset link in email, set new password, verify redirect and ability to sign in with new password

### Implementation for User Story 3

#### Forgot Password Page

- [ ] T036 [P] [US3] Create forgot-password page component in apps/sso-client/app/(auth)/forgot-password/page.tsx (imports ForgotPasswordForm, uses useAuthRedirect hook)
- [ ] T037 [US3] Create ForgotPasswordForm component in apps/sso-client/app/(auth)/forgot-password/forgot-password-form.tsx (uses react-hook-form with forgotPasswordSchema, calls authClient.forgetPassword)
- [ ] T038 [US3] Implement success message display in ForgotPasswordForm (show generic message regardless of email existence to prevent user enumeration, keep user on page)
- [ ] T039 [US3] Implement error handling in ForgotPasswordForm (display network errors or API timeouts, allow retry)
- [ ] T040 [US3] Add link to sign-in page in apps/sso-client/app/(auth)/forgot-password/page.tsx (below form: "Remember your password? Sign in")

#### Reset Password Page

- [ ] T041 [P] [US3] Create reset-password page component in apps/sso-client/app/(auth)/reset-password/page.tsx (extracts token from query params, imports ResetPasswordForm, validates token existence)
- [ ] T042 [US3] Create ResetPasswordForm component in apps/sso-client/app/(auth)/reset-password/reset-password-form.tsx (uses react-hook-form with resetPasswordSchema, calls authClient.resetPassword)
- [ ] T043 [US3] Implement password confirmation validation in ResetPasswordForm (Zod refine to check newPassword === confirmPassword, display error on confirmPassword field)
- [ ] T044 [US3] Implement token expiration error handling in ResetPasswordForm (detect TOKEN_EXPIRED error code, display user-friendly message with link to forgot-password page)
- [ ] T045 [US3] Implement invalid token error handling in ResetPasswordForm (detect INVALID_TOKEN error code, display error message with link to forgot-password page)
- [ ] T046 [US3] Implement success redirect in ResetPasswordForm (redirect to /auth/signin with success message after password reset)
- [ ] T047 [US3] Implement missing token error state in apps/sso-client/app/(auth)/reset-password/page.tsx (display error if no token query parameter, link to forgot-password page)

**Checkpoint**: User Story 3 complete - users can request password reset and complete reset flow with proper token validation

---

## Phase 6: User Story 4 - Social Login (OAuth) (Priority: P3)

**Goal**: Enable users to sign in or sign up using GitHub or Google accounts, with account linking for matching emails

**Independent Test**: Navigate to http://localhost:3001/signin, click "Sign in with GitHub", complete OAuth flow, verify successful authentication and redirect to dashboard

### Implementation for User Story 4

- [ ] T048 [P] [US4] Create SocialLoginButtons component in apps/sso-client/components/auth/social-login-buttons.tsx (displays GitHub and Google buttons with provider logos)
- [ ] T049 [US4] Implement GitHub OAuth redirect in SocialLoginButtons (window.location.href to ${SSO_SERVER_URL}/api/auth/sign-in/github with callbackURL parameter)
- [ ] T050 [US4] Implement Google OAuth redirect in SocialLoginButtons (window.location.href to ${SSO_SERVER_URL}/api/auth/sign-in/google with callbackURL parameter)
- [ ] T051 [US4] Add SocialLoginButtons to SignInForm in apps/sso-client/app/(auth)/signin/signin-form.tsx (below password field, above "Forgot Password?" link, with divider)
- [ ] T052 [US4] Add SocialLoginButtons to SignUpForm in apps/sso-client/app/(auth)/signup/signup-form.tsx (below name field, above submit button, with divider)
- [ ] T053 [US4] Create OAuth callback handling documentation in apps/sso-client/README.md (explain that backend handles OAuth callback and redirects back with session cookie)

**Checkpoint**: User Story 4 complete - users can authenticate with GitHub/Google, account linking handled by backend

---

## Phase 7: User Story 5 - OIDC Integration for External Applications (Priority: P2)

**Goal**: Enable external applications to integrate with SSO using OIDC protocol, with login and consent pages

**Independent Test**: From test application, initiate OIDC flow with valid client_id, verify redirect to login page, sign in, grant consent, verify redirect back with authorization code

### Implementation for User Story 5

#### OIDC Login Page

- [ ] T054 [P] [US5] Create login page component in apps/sso-client/app/(auth)/login/page.tsx (extracts OIDC params from query, uses useAuthRedirect hook with OIDC context)
- [ ] T055 [US5] Implement OIDC parameter validation in login page (validate client_id, redirect_uri, response_type, scope, display error if invalid)
- [ ] T056 [US5] Implement automatic redirect for authenticated users in login page (if session exists, redirect to consent page or callback with authorization code)
- [ ] T057 [US5] Display client application information in login page (show requesting app name, logo if available, requested scopes)
- [ ] T058 [US5] Integrate SignInForm with OIDC context in login page (pass OIDC parameters through authentication flow, redirect to consent after sign-in)

#### OIDC Consent Page

- [ ] T059 [P] [US5] Create consent page component in apps/sso-client/app/(auth)/consent/page.tsx (extracts OIDC params from query, checks session, imports ConsentForm)
- [ ] T060 [US5] Create ConsentForm component in apps/sso-client/app/(auth)/consent/consent-form.tsx (displays requested permissions, grant/deny buttons)
- [ ] T061 [US5] Implement scope display in ConsentForm (parse scope parameter, display human-readable permission descriptions for openid, profile, email scopes)
- [ ] T062 [US5] Implement grant consent action in ConsentForm (POST to ${SSO_SERVER_URL}/api/auth/consent with client_id, redirect_uri, scope, state, approved: true)
- [ ] T063 [US5] Implement deny consent action in ConsentForm (POST to ${SSO_SERVER_URL}/api/auth/consent with approved: false, redirect to redirect_uri with error)
- [ ] T064 [US5] Implement redirect after consent in ConsentForm (backend redirects to redirect_uri with authorization code or error)
- [ ] T065 [US5] Implement missing session error in consent page (redirect to login page with OIDC parameters if no session)

**Checkpoint**: User Story 5 complete - external applications can authenticate users via OIDC with login and consent flows

---

## Phase 8: Email Verification Page (Supporting Feature)

**Goal**: Provide email verification confirmation page for users who register with email

**Independent Test**: Register new account, click verification link in email, verify success message and redirect to dashboard

- [ ] T066 [P] Create verify-email page component in apps/sso-client/app/(auth)/verify-email/page.tsx (extracts token from query params, calls authClient.verifyEmail)
- [ ] T067 Implement success state in verify-email page (display success message with checkmark icon, show "Continue to Dashboard" button)
- [ ] T068 Implement error state in verify-email page (detect INVALID_TOKEN or TOKEN_EXPIRED errors, display appropriate error messages)
- [ ] T069 Implement loading state in verify-email page (show loading spinner while verification API call is in progress)
- [ ] T070 Implement automatic redirect in verify-email page (redirect to dashboard 3 seconds after successful verification)
- [ ] T071 Implement missing token error in verify-email page (display error if no token query parameter, link to sign-in page)

**Checkpoint**: Email verification page complete - users can verify their email addresses

---

## Phase 9: Dashboard Page (Redirect Target)

**Goal**: Provide default redirect destination after successful authentication

- [ ] T072 Create dashboard page in apps/sso-client/app/dashboard/page.tsx (simple welcome page displaying user name and email from session)
- [ ] T073 Implement session check in dashboard page (use authClient.getSession, redirect to /auth/signin if no session)
- [ ] T074 Add sign-out button in dashboard page (calls authClient.signOut, redirects to /auth/signin)
- [ ] T075 Display user information in dashboard (show user.name, user.email, user.emailVerified status from session)

**Checkpoint**: Dashboard page complete - users have a destination after authentication

---

## Phase 10: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements, accessibility, responsiveness, and documentation

- [ ] T076 [P] Implement responsive design for all auth pages (test on mobile, tablet, desktop, ensure forms are centered and readable)
- [ ] T077 [P] Add keyboard navigation support (test tab order, ensure all interactive elements are keyboard-accessible)
- [ ] T078 [P] Add ARIA labels and roles to form components (screen reader testing, ensure proper announcements for errors and loading states)
- [ ] T079 [P] Implement focus management (auto-focus first form field on page load, focus error fields after validation)
- [ ] T080 [P] Add loading state animations (smooth transitions for loading spinners, skeleton screens where appropriate)
- [ ] T081 [P] Standardize error message styling (consistent Alert component usage, proper color contrast for accessibility)
- [ ] T082 [P] Add form field icons (email icon for email fields, lock icon for password fields, eye icon for password visibility toggle)
- [ ] T083 Implement password visibility toggle in SignUpForm and ResetPasswordForm (eye icon button to show/hide password)
- [ ] T084 Add input autocomplete attributes (autocomplete="email" for email fields, autocomplete="current-password" for signin, autocomplete="new-password" for signup/reset)
- [ ] T085 Test all pages with browser DevTools Network throttling (verify 30s timeout works, ensure good UX on slow connections)
- [ ] T086 Verify all error scenarios (test each error code from AUTH_ERROR_CODES, ensure proper user-facing messages)
- [ ] T087 Create README.md in apps/sso-client with development instructions (how to start dev server, environment variables, page routes)
- [ ] T088 Add TypeScript strict mode validation (ensure no 'any' types, all props properly typed)
- [ ] T089 Implement consistent spacing and typography (verify Tailwind classes match design system, consistent padding/margins)
- [ ] T090 Add page metadata and SEO (title tags, meta descriptions for each auth page)

**Checkpoint**: All polish tasks complete - production-ready authentication UI

---

## Dependencies & Execution Strategy

### User Story Dependencies

```
Phase 1 (Setup) & Phase 2 (Foundational)
    ↓
Phase 3 (US1: Sign Up) ─────┐
                            ├──→ Can be implemented in parallel
Phase 4 (US2: Sign In) ─────┤
                            │
Phase 5 (US3: Password Reset) ─┤
                            │
Phase 6 (US4: Social Login) ────┤
                            │
Phase 7 (US5: OIDC) ───────────┘
    ↓
Phase 8 (Email Verification) ← Supporting feature
    ↓
Phase 9 (Dashboard) ← Redirect target
    ↓
Phase 10 (Polish)
```

### Parallel Execution Opportunities

**After Foundational Phase (T009-T017) is complete:**

- **US1 (T018-T025)** can be implemented independently
- **US2 (T026-T035)** can be implemented independently
- **US3 (T036-T047)** can be implemented independently
- **US4 (T048-T053)** can be implemented independently (depends on SignInForm/SignUpForm existing but can add SocialLoginButtons component in parallel)
- **US5 (T054-T065)** can be implemented independently

**Within each User Story:**
- Tasks marked [P] can run in parallel (different files, no dependencies)
- Non-[P] tasks within a story should be sequential

**Polish Phase (T076-T090):**
- Most polish tasks marked [P] can run in parallel as they touch different concerns

### Implementation Strategy

**MVP Scope (Recommended First Release):**
- Phase 1: Setup (T001-T008)
- Phase 2: Foundational (T009-T017)
- Phase 3: User Story 1 - Sign Up (T018-T025)
- Phase 4: User Story 2 - Sign In (T026-T035)
- Phase 9: Dashboard (T072-T075)
- Selected Polish: T076-T081, T087-T088

**Second Release:**
- Phase 5: User Story 3 - Password Reset (T036-T047)
- Phase 8: Email Verification (T066-T071)

**Third Release:**
- Phase 6: User Story 4 - Social Login (T048-T053)

**Fourth Release:**
- Phase 7: User Story 5 - OIDC (T054-T065)

**Final Polish:**
- Phase 10: Remaining polish tasks (T082-T086, T089-T090)

### Testing Strategy

**Manual Testing Checklist (No automated tests required per spec):**

1. **Sign Up Flow**: Navigate to /signup → Enter valid data → Verify redirect to dashboard
2. **Sign In Flow**: Navigate to /signin → Enter credentials → Test CAPTCHA after 5 failures → Verify redirect
3. **Password Reset**: Click "Forgot Password" → Enter email → Check email for link → Click link → Reset password → Sign in with new password
4. **Social Login**: Click GitHub/Google button → Complete OAuth → Verify account creation/linking
5. **OIDC Flow**: Initiate from test app → Sign in → Grant consent → Verify authorization code redirect
6. **Email Verification**: Register → Click verification link in email → Verify success message
7. **Edge Cases**: Test expired tokens, invalid tokens, already-logged-in users accessing auth pages, API timeouts

---

## Task Summary

**Total Tasks**: 90

**Breakdown by Phase**:
- Phase 1 (Setup): 8 tasks
- Phase 2 (Foundational): 9 tasks (BLOCKING)
- Phase 3 (US1 - Sign Up): 8 tasks
- Phase 4 (US2 - Sign In): 10 tasks
- Phase 5 (US3 - Password Reset): 12 tasks
- Phase 6 (US4 - Social Login): 6 tasks
- Phase 7 (US5 - OIDC): 12 tasks
- Phase 8 (Email Verification): 6 tasks
- Phase 9 (Dashboard): 4 tasks
- Phase 10 (Polish): 15 tasks

**Parallelizable Tasks**: 45 tasks marked [P]

**User Story Distribution**:
- US1 (Sign Up): 8 tasks
- US2 (Sign In): 10 tasks
- US3 (Password Reset): 12 tasks
- US4 (Social Login): 6 tasks
- US5 (OIDC): 12 tasks
- Supporting: 10 tasks (Email Verification + Dashboard)
- Infrastructure: 17 tasks (Setup + Foundational)
- Polish: 15 tasks

**Estimated MVP Scope**: 35 tasks (Setup + Foundational + US1 + US2 + Dashboard + Core Polish)

**Independent Test Criteria Met**: ✅ Each user story has clear acceptance scenarios and can be tested independently

**Format Validation**: ✅ All tasks follow checklist format with Task ID, [P] marker where applicable, [Story] label for user story tasks, and specific file paths
