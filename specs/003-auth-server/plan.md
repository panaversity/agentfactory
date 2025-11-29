# Implementation Plan: Auth Server

**Branch**: `003-auth-server` | **Date**: 2025-11-29 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-auth-server/spec.md`

## Summary

Build a standalone authentication server for the RoboLearn platform using Next.js and Better Auth. The server handles email/password registration, login, session management, and user profile storage (including software background level for personalization). Architecture is SSO-ready for future multi-client support but MVP focuses on serving robolearn-interface as the single client.

## Technical Context

**Language/Version**: TypeScript 5.x, Node.js 20+
**Framework**: Next.js 14+ (App Router)
**Primary Dependencies**: Better Auth, Drizzle ORM, Neon Serverless Driver
**Storage**: Neon Postgres (serverless)
**Testing**: Vitest (unit), Playwright (e2e)
**Target Platform**: Vercel / Cloud Run (serverless)
**Project Type**: Web application (standalone auth server)
**Performance Goals**: <100ms session validation, 100 concurrent users
**Constraints**: HTTP-only cookies, CORS for robolearn-interface domain
**Scale/Scope**: Single client (robolearn-interface), ~1000 users initially

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| **Specification Primacy** | ✅ Pass | Spec created before implementation plan |
| **Factual Accuracy** | ✅ Pass | Using Better Auth official docs via MCP |
| **Formal Verification** | ✅ Pass | Security-critical domain - invariants verified in spec |
| **Hardware Tier Awareness** | N/A | Auth is tier-agnostic |
| **Safety-Critical Content** | ✅ Pass | Password hashing, rate limiting, HTTP-only cookies specified |

## Project Structure

### Documentation (this feature)

```text
specs/003-auth-server/
├── spec.md              # Feature specification ✅
├── plan.md              # This file ✅
├── checklists/
│   └── requirements.md  # Validation checklist ✅
└── tasks.md             # Phase 3 output (next step)
```

### Source Code (repository root)

```text
auth-server/                      # New Next.js project
├── src/
│   ├── app/
│   │   ├── api/
│   │   │   └── auth/
│   │   │       └── [...all]/
│   │   │           └── route.ts  # Better Auth API handler
│   │   ├── auth/
│   │   │   ├── sign-in/
│   │   │   │   └── page.tsx      # Login page
│   │   │   ├── sign-up/
│   │   │   │   └── page.tsx      # Registration page + onboarding
│   │   │   └── layout.tsx        # Auth layout
│   │   ├── layout.tsx            # Root layout
│   │   └── page.tsx              # Redirect to sign-in or dashboard
│   ├── lib/
│   │   ├── auth.ts               # Better Auth configuration
│   │   ├── auth-client.ts        # Client-side auth helpers
│   │   └── db/
│   │       ├── index.ts          # Drizzle client
│   │       └── schema.ts         # User, Session, UserProfile tables
│   └── components/
│       ├── sign-in-form.tsx      # Login form component
│       ├── sign-up-form.tsx      # Registration form component
│       └── background-select.tsx # Software background selector
├── drizzle/
│   └── migrations/               # Database migrations
├── drizzle.config.ts
├── package.json
├── tsconfig.json
├── next.config.ts
├── .env.example
└── .env.local                    # Local environment (gitignored)
```

**Structure Decision**: Standalone Next.js application separate from robolearn-interface. This enables independent deployment, clear separation of concerns, and future SSO expansion.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    AUTH SERVER (Next.js)                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Pages (App Router)              API Routes                     │
│  ┌────────────────────┐         ┌────────────────────────────┐ │
│  │ /auth/sign-in      │         │ /api/auth/[...all]         │ │
│  │ /auth/sign-up      │         │ - POST sign-up             │ │
│  │ /                  │────────►│ - POST sign-in             │ │
│  └────────────────────┘         │ - POST sign-out            │ │
│                                 │ - GET session              │ │
│                                 │ - GET/PUT profile          │ │
│  Components                     └─────────────┬──────────────┘ │
│  ┌────────────────────┐                       │                │
│  │ SignInForm         │                       │                │
│  │ SignUpForm         │                       ▼                │
│  │ BackgroundSelect   │         ┌────────────────────────────┐ │
│  └────────────────────┘         │ Better Auth Core           │ │
│                                 │ - Email/Password provider  │ │
│                                 │ - Session management       │ │
│                                 │ - Rate limiting            │ │
│                                 └─────────────┬──────────────┘ │
│                                               │                │
│                                               ▼                │
│                                 ┌────────────────────────────┐ │
│                                 │ Drizzle ORM                │ │
│                                 │ - User table               │ │
│                                 │ - Session table            │ │
│                                 │ - Account table            │ │
│                                 │ - UserProfile table        │ │
│                                 └─────────────┬──────────────┘ │
│                                               │                │
└───────────────────────────────────────────────┼────────────────┘
                                                │
                                                ▼
                                 ┌────────────────────────────┐
                                 │ Neon Postgres              │
                                 │ (Serverless)               │
                                 └────────────────────────────┘
```

## Database Schema

```typescript
// Better Auth managed tables
user {
  id: text PK
  email: text UNIQUE NOT NULL
  name: text
  emailVerified: boolean DEFAULT false
  image: text
  createdAt: timestamp DEFAULT now()
  updatedAt: timestamp DEFAULT now()
}

session {
  id: text PK
  userId: text FK -> user.id
  token: text UNIQUE NOT NULL
  expiresAt: timestamp NOT NULL
  ipAddress: text
  userAgent: text
  createdAt: timestamp DEFAULT now()
  updatedAt: timestamp DEFAULT now()
}

account {
  id: text PK
  userId: text FK -> user.id
  accountId: text NOT NULL
  providerId: text NOT NULL  // "credential" for email/password
  accessToken: text
  refreshToken: text
  accessTokenExpiresAt: timestamp
  refreshTokenExpiresAt: timestamp
  scope: text
  password: text  // hashed, only for credential provider
  createdAt: timestamp DEFAULT now()
  updatedAt: timestamp DEFAULT now()
}

// Custom RoboLearn extension
user_profile {
  id: text PK
  userId: text FK -> user.id UNIQUE
  softwareBackground: enum('beginner', 'intermediate', 'advanced') NOT NULL
  createdAt: timestamp DEFAULT now()
  updatedAt: timestamp DEFAULT now()
}
```

## Implementation Phases

### Phase 1: Project Setup & Database

**Goal**: Working Next.js project connected to Neon Postgres with schema deployed.

| Task | Description | Files |
|------|-------------|-------|
| 1.1 | Create Next.js project with TypeScript | `auth-server/package.json`, `tsconfig.json` |
| 1.2 | Install dependencies (better-auth, drizzle-orm, @neondatabase/serverless) | `package.json` |
| 1.3 | Configure Drizzle with Neon driver | `drizzle.config.ts`, `src/lib/db/index.ts` |
| 1.4 | Define database schema | `src/lib/db/schema.ts` |
| 1.5 | Run initial migration | `drizzle/migrations/` |
| 1.6 | Create .env.example with required variables | `.env.example` |

**Exit Criteria**: `npm run db:push` succeeds, tables visible in Neon console.

### Phase 2: Better Auth Configuration

**Goal**: Better Auth initialized with email/password provider, sessions working.

| Task | Description | Files |
|------|-------------|-------|
| 2.1 | Configure Better Auth with Drizzle adapter | `src/lib/auth.ts` |
| 2.2 | Set up email/password credential provider | `src/lib/auth.ts` |
| 2.3 | Configure session settings (7-day duration, HTTP-only cookies) | `src/lib/auth.ts` |
| 2.4 | Create catch-all API route | `src/app/api/auth/[...all]/route.ts` |
| 2.5 | Create client-side auth helpers | `src/lib/auth-client.ts` |
| 2.6 | Configure CORS for robolearn-interface | `next.config.ts` |

**Exit Criteria**: POST to `/api/auth/sign-up` creates user, `/api/auth/session` returns session.

### Phase 3: Auth UI Components

**Goal**: Functional sign-in and sign-up pages with forms.

| Task | Description | Files |
|------|-------------|-------|
| 3.1 | Create auth layout with shared styles | `src/app/auth/layout.tsx` |
| 3.2 | Build sign-in form component | `src/components/sign-in-form.tsx` |
| 3.3 | Build sign-up form component | `src/components/sign-up-form.tsx` |
| 3.4 | Build software background selector | `src/components/background-select.tsx` |
| 3.5 | Create sign-in page | `src/app/auth/sign-in/page.tsx` |
| 3.6 | Create sign-up page with onboarding | `src/app/auth/sign-up/page.tsx` |
| 3.7 | Add form validation (client-side) | Components |
| 3.8 | Style with Tailwind (minimal, clean) | All components |

**Exit Criteria**: User can register with email/password/background, then sign in.

### Phase 4: User Profile API

**Goal**: Profile data accessible via authenticated API endpoint.

| Task | Description | Files |
|------|-------------|-------|
| 4.1 | Create profile creation hook (after signup) | `src/lib/auth.ts` hooks |
| 4.2 | Add GET /api/profile endpoint | `src/app/api/profile/route.ts` |
| 4.3 | Add PUT /api/profile endpoint (update background) | `src/app/api/profile/route.ts` |
| 4.4 | Add authentication middleware check | API routes |

**Exit Criteria**: Authenticated GET `/api/profile` returns user's software background.

### Phase 5: Security & Polish

**Goal**: Rate limiting, error handling, and production readiness.

| Task | Description | Files |
|------|-------------|-------|
| 5.1 | Configure rate limiting (5 attempts/min/IP) | `src/lib/auth.ts` |
| 5.2 | Add user-friendly error messages | Components |
| 5.3 | Implement logout functionality | UI + API |
| 5.4 | Add session persistence across browser close | Cookie config |
| 5.5 | Test CORS with robolearn-interface | Integration test |

**Exit Criteria**: Rate limiting blocks rapid attempts, errors are user-friendly.

### Phase 6: Integration & Deployment

**Goal**: Auth server deployed and working with robolearn-interface.

| Task | Description | Files |
|------|-------------|-------|
| 6.1 | Deploy to Vercel (or Cloud Run) | Deployment config |
| 6.2 | Configure production environment variables | Vercel dashboard |
| 6.3 | Test end-to-end flow from robolearn-interface | Manual testing |
| 6.4 | Document integration steps for robolearn-interface | README |

**Exit Criteria**: Live auth server, robolearn-interface can authenticate users.

## Dependencies Between Phases

```
Phase 1 (Setup) ──► Phase 2 (Better Auth) ──► Phase 3 (UI)
                                          │
                                          └──► Phase 4 (Profile API)
                                                      │
                                                      ▼
                   Phase 5 (Security) ◄───────────────┘
                          │
                          ▼
                   Phase 6 (Deploy)
```

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Better Auth configuration issues | Use official MCP server for guidance |
| Neon connection problems | Test locally with connection pooling |
| CORS misconfiguration | Test with actual robolearn-interface domain early |
| Session not persisting | Verify cookie settings (secure, sameSite, domain) |

## Cross-Book Intelligence Assessment

**Patterns that apply to future books**:
- Better Auth + Next.js setup pattern
- Drizzle + Neon configuration
- Auth UI component patterns

**Patterns specific to THIS platform**:
- UserProfile with softwareBackground field
- Personalization API endpoints

**Skills to create (platform-level)**:
- `better-auth-setup` skill (future - after this implementation validates the pattern)

**Knowledge files needed**:
- None new - existing stack.md covers technology choices

## Environment Variables Required

```env
# Database
DATABASE_URL=postgresql://user:pass@host/db?sslmode=require

# Better Auth
BETTER_AUTH_SECRET=your-secret-key-min-32-chars
BETTER_AUTH_URL=http://localhost:3001  # or production URL

# CORS
ALLOWED_ORIGINS=http://localhost:3000,https://robolearn.example.com
```

## Success Validation

After implementation, verify:

- [ ] User can register with email, password, and background selection (<60s)
- [ ] User can sign in with credentials (<10s)
- [ ] Session persists for 7 days
- [ ] Logout invalidates session
- [ ] Profile endpoint returns software background
- [ ] Rate limiting blocks rapid login attempts
- [ ] CORS allows robolearn-interface requests
- [ ] No plaintext passwords in database
