# Tasks: Auth Server

**Feature**: 003-auth-server
**Generated**: 2025-11-29
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

## Task Summary

| Phase | Description | Tasks | Completed |
|-------|-------------|-------|-----------|
| 1 | Setup | 6 | 6 |
| 2 | Foundational (Better Auth) | 6 | 6 |
| 3 | US1 - Registration | 6 | 6 |
| 4 | US2 - Login | 4 | 4 |
| 5 | US3 - Logout | 3 | 3 |
| 6 | US4 - Profile API | 4 | 4 |
| 7 | US5 - Update Profile | 2 | 2 |
| 8 | Polish & Integration | 10 | 8 |
| 9 | OAuth/OIDC Provider | 7 | 7 |
| 10 | Email Verification | 4 | 4 |
| **Total** | | **52** | **50** |

---

## Phase 1: Setup

**Goal**: Working Next.js project with Drizzle ORM connected to Neon Postgres.

**Exit Criteria**: `npm run db:push` succeeds, tables visible in Neon console.

### Tasks

- [x] T001 Create Next.js project with TypeScript in `auth-server/`
- [x] T002 Install dependencies: better-auth, drizzle-orm, @neondatabase/serverless, dotenv in `auth-server/package.json`
- [x] T003 [P] Create environment template with DATABASE_URL, BETTER_AUTH_SECRET, BETTER_AUTH_URL, ALLOWED_ORIGINS in `auth-server/.env.example`
- [x] T004 [P] Configure Drizzle with Neon serverless driver in `auth-server/drizzle.config.ts`
- [x] T005 Create Drizzle database client in `auth-server/src/lib/db/index.ts`
- [x] T006 Define database schema (user, session, account, user_profile tables) in `auth-server/src/lib/db/schema.ts`

---

## Phase 2: Foundational (Better Auth Core)

**Goal**: Better Auth initialized with email/password provider, API routes working.

**Exit Criteria**: POST to `/api/auth/sign-up` creates user in database.

**Dependencies**: Phase 1 must be complete.

### Tasks

- [x] T007 Configure Better Auth with Drizzle adapter and email/password provider in `auth-server/src/lib/auth.ts`
- [x] T008 Configure session settings (7-day duration, HTTP-only cookies) in `auth-server/src/lib/auth.ts`
- [x] T009 [P] Create Better Auth catch-all API route in `auth-server/src/app/api/auth/[...all]/route.ts`
- [x] T010 [P] Create client-side auth helpers (signIn, signUp, signOut, useSession) in `auth-server/src/lib/auth-client.ts`
- [x] T011 Configure CORS headers for robolearn-interface domain in `auth-server/next.config.ts`
- [x] T012 Run database migration to create Better Auth tables via `npm run db:push`

---

## Phase 3: US1 - New User Registration (P1)

**User Story**: A visitor wants to create an account with email, password, and software background level.

**Independent Test**: Complete registration flow end-to-end, verify user record exists with profile data.

**Acceptance Criteria**:
- Valid email, password (8+ chars), background selection → account created, redirected as logged-in
- Duplicate email → clear error with sign-in link
- Short password → immediate validation feedback

### Tasks

- [x] T013 [P] [US1] Create software background selector component (beginner/intermediate/advanced) in `auth-server/src/components/background-select.tsx`
- [x] T014 [P] [US1] Create sign-up form component with email, password, confirm password fields in `auth-server/src/components/sign-up-form.tsx`
- [x] T015 [P] [US1] Create auth layout with centered card styling in `auth-server/src/app/auth/layout.tsx`
- [x] T016 [US1] Add client-side validation (email format, password 8+ chars, passwords match) in `auth-server/src/components/sign-up-form.tsx`
- [x] T017 [US1] Create sign-up page integrating form and background selector in `auth-server/src/app/auth/sign-up/page.tsx`
- [x] T018 [US1] Add Better Auth hook to create user_profile record after signup in `auth-server/src/lib/auth.ts`

---

## Phase 4: US2 - Existing User Login (P1)

**User Story**: A returning user wants to sign in to continue learning.

**Independent Test**: Sign in with known credentials, verify session created and redirect works.

**Acceptance Criteria**:
- Correct credentials → session created, redirected to book interface
- Incorrect credentials → generic "Invalid credentials" message
- Session persists across browser close for 7 days

### Tasks

- [x] T019 [P] [US2] Create sign-in form component with email and password fields in `auth-server/src/components/sign-in-form.tsx`
- [x] T020 [P] [US2] Create sign-in page in `auth-server/src/app/auth/sign-in/page.tsx`
- [x] T021 [US2] Add error handling with generic "Invalid credentials" message in `auth-server/src/components/sign-in-form.tsx`
- [x] T022 [US2] Create root page that redirects to sign-in or dashboard based on session in `auth-server/src/app/page.tsx`

---

## Phase 5: US3 - User Logout (P2)

**User Story**: A logged-in user wants to sign out of their account.

**Independent Test**: Log in, click logout, verify session invalidated and redirected.

**Acceptance Criteria**:
- Logout button → session invalidated, redirected to login
- Browser back button after logout → cannot access protected content

### Tasks

- [x] T023 [P] [US3] Create logout button component that calls signOut in `auth-server/src/components/logout-button.tsx`
- [x] T024 [US3] Add logout button to authenticated pages/layout in `auth-server/src/app/auth/layout.tsx`
- [x] T025 [US3] Verify session invalidation prevents back-button access (test manually)

---

## Phase 6: US4 - Profile Data Access (P2)

**User Story**: robolearn-interface needs to fetch user profile data for personalization.

**Independent Test**: Authenticated API request returns user's software background level.

**Acceptance Criteria**:
- Authenticated GET /api/profile → returns user info + software background
- Unauthenticated request → 401 Unauthorized

### Tasks

- [x] T026 [P] [US4] Create GET /api/profile endpoint returning user + profile data in `auth-server/src/app/api/profile/route.ts`
- [x] T027 [P] [US4] Add session validation middleware to profile endpoint in `auth-server/src/app/api/profile/route.ts`
- [x] T028 [US4] Return 401 for unauthenticated requests in `auth-server/src/app/api/profile/route.ts`
- [x] T029 [US4] Test profile endpoint with curl/Postman (authenticated and unauthenticated)

---

## Phase 7: US5 - Update Profile (P3)

**User Story**: A user wants to update their software background level.

**Independent Test**: Update background field, verify change persists.

**Acceptance Criteria**:
- Change background level → saved and reflected in subsequent fetches

### Tasks

- [x] T030 [P] [US5] Add PUT /api/profile endpoint for updating software background in `auth-server/src/app/api/profile/route.ts`
- [x] T031 [US5] Validate softwareBackground enum value (beginner/intermediate/advanced) in `auth-server/src/app/api/profile/route.ts`

---

## Phase 8: Polish & Deploy

**Goal**: Production-ready auth server with rate limiting and deployment.

**Exit Criteria**: Live auth server accessible from robolearn-interface.

### Tasks

- [x] T032 Configure rate limiting (5 attempts/min/IP) in Better Auth config in `auth-server/src/lib/auth.ts`
- [x] T033 [P] Add user-friendly error messages to all forms (network errors, validation) in `auth-server/src/components/`
- [x] T034 [P] Create README with setup instructions and environment variable docs in `auth-server/README.md`
- [x] T035 Add Better Auth client to robolearn-interface in `robolearn-interface/src/lib/auth-client.ts`
- [x] T036 Create AuthContext for session state in `robolearn-interface/src/contexts/AuthContext.tsx`
- [x] T037 Update navbar with auth state (Get Started / Sign In / User Name) in `robolearn-interface/src/components/NavbarAuth/`
- [x] T038 Wire up Hero and CTA buttons to auth-server redirects
- [x] T039 Test all API endpoints (sign-up, sign-in, sign-out, profile CRUD)
- [ ] T040 Deploy auth-server to Vercel (or Cloud Run)
- [ ] T041 Configure production environment variables in deployment platform

---

## Phase 9: OAuth/OIDC Provider (COMPLETED)

**Goal**: Auth server acts as OAuth 2.1 / OIDC Provider with PKCE and JWKS support.

**Exit Criteria**: OAuth authorization flow works with PKCE, JWKS endpoint returns valid keys.

**Dependencies**: Phase 2 must be complete.

### Tasks

- [x] T042 [P] Add JWT plugin with JWKS configuration (RS256 asymmetric signing) in `auth-server/src/lib/auth.ts`
- [x] T043 [P] Configure OIDC Provider plugin with public client support and useJWTPlugin in `auth-server/src/lib/auth.ts`
- [x] T044 [P] Define OAuth tables in schema (oauth_application, oauth_access_token, oauth_consent, jwks) in `auth-server/src/lib/db/schema.ts`
- [x] T045 Create TypeScript seed script for trusted public client in `auth-server/scripts/seed-public-client.ts`
- [x] T046 Create SQL seed file for direct database seeding in `auth-server/scripts/seed-public-client.sql`
- [x] T047 [P] Add admin-only client registration endpoint (custom, not built-in) in `auth-server/src/app/api/admin/clients/register/route.ts`
- [x] T048 Test OAuth flow: authorize → token exchange → userinfo with PKCE

---

## Phase 10: Email Verification & Password Reset (COMPLETED)

**Goal**: Email verification on signup and password reset functionality.

**Exit Criteria**: Verification emails sent on signup, password reset flow works end-to-end.

**Dependencies**: Phase 2 must be complete.

### Tasks

- [x] T049 [P] Configure Resend and SMTP email providers with fallback logic in `auth-server/src/lib/auth.ts`
- [x] T050 [P] Add email verification config (sendOnSignUp, autoSignInAfterVerification) in `auth-server/src/lib/auth.ts`
- [x] T051 [P] Add password reset email templates and sendResetPassword callback in `auth-server/src/lib/auth.ts`
- [x] T052 Update .env.example with email provider variables (RESEND_API_KEY, SMTP_*, EMAIL_FROM) in `auth-server/.env.example`

---

## Dependencies Graph

```
Phase 1 (Setup)
    │
    ▼
Phase 2 (Better Auth Core)
    │
    ├────────────────┬────────────────┐
    ▼                ▼                ▼
Phase 3 (US1)    Phase 4 (US2)    Phase 6 (US4)
Registration       Login          Profile API
    │                │                │
    └────────┬───────┘                │
             ▼                        │
        Phase 5 (US3)                 │
          Logout                      │
             │                        │
             └────────┬───────────────┘
                      ▼
               Phase 7 (US5)
              Update Profile
                      │
                      ▼
               Phase 8 (Polish)
```

## Parallel Execution Opportunities

### Within Phase 1 (Setup)
```
T001 → T002 → [T003 ∥ T004] → T005 → T006
```

### Within Phase 2 (Foundational)
```
T007 → T008 → [T009 ∥ T010] → T011 → T012
```

### Within Phase 3 (US1 - Registration)
```
[T013 ∥ T014 ∥ T015] → T016 → T017 → T018
```

### Within Phase 4 (US2 - Login)
```
[T019 ∥ T020] → T021 → T022
```

### Across Phases (after Phase 2)
```
[Phase 3 (US1) ∥ Phase 4 (US2) ∥ Phase 6 (US4)]
```
Note: US1, US2, and US4 can be developed in parallel after Phase 2 completes.

---

## Implementation Strategy

### MVP Scope (Minimum for hackathon points)
- **Phase 1**: Setup ✓
- **Phase 2**: Better Auth Core ✓
- **Phase 3**: US1 - Registration ✓
- **Phase 4**: US2 - Login ✓
- **Phase 8**: Deploy (minimal) ✓

**MVP delivers**: Users can register and sign in. Earns auth points.

### Extended Scope (Full 50 bonus points)
- Add Phase 5: US3 - Logout
- Add Phase 6: US4 - Profile API (required for personalization)
- Add Phase 7: US5 - Update Profile
- Complete Phase 8: Full polish

**Extended delivers**: Complete auth + profile for personalization feature.

---

## Validation Checklist

After all tasks complete, verify:

### Email/Password Auth (Basic)
- [x] User can register with email, password, background (<60 seconds)
- [x] User can sign in with credentials (<10 seconds)
- [x] 100% of users have software background captured
- [x] Session persists 7 days
- [x] Logout invalidates session
- [x] GET /api/profile returns software background (authenticated)
- [x] GET /api/profile returns 401 (unauthenticated)
- [x] PUT /api/profile updates background level
- [x] Rate limiting blocks rapid login attempts
- [x] CORS allows robolearn-interface requests
- [x] No plaintext passwords in database

### OAuth/OIDC Provider
- [x] GET /.well-known/openid-configuration returns OIDC discovery document
- [x] GET /api/auth/jwks returns valid RSA public keys
- [x] OAuth authorization flow with PKCE succeeds (<5 seconds)
- [x] Token exchange with code_verifier (no client_secret) succeeds
- [x] GET /api/auth/oauth2/userinfo returns user data with custom claims
- [x] Admin can register new OAuth clients via POST /api/admin/clients/register
- [x] Public clients (type: "public") have clientSecret = null in database

### Email Verification
- [x] Verification email sent on signup (Resend or SMTP)
- [x] Email verification link completes and auto-signs in user
- [x] Password reset email sent when requested
- [x] Password reset flow completes successfully

### Deployment (Pending)
- [ ] Deploy auth-server to Vercel (or Cloud Run)
- [ ] Configure production environment variables in deployment platform
