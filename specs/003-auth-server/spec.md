# Feature Specification: Auth Server

**Feature Branch**: `003-auth-server`
**Created**: 2025-11-29
**Status**: Draft
**Input**: User description: "Build production-ready authentication system using Better Auth with Next.js. Email/password only (no OAuth for now). Simple onboarding with 1 question about software background. Connects to robolearn-interface. SSO-ready architecture for future expansion."

## Executive Summary

A standalone authentication server for the RoboLearn platform that handles user registration, login, session management, and basic user profiling. The auth server will be built with Next.js + Better Auth, connected to Neon Postgres, and designed with SSO-ready architecture for future multi-client support.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - New User Registration (Priority: P1)

A visitor to the RoboLearn platform wants to create an account to access personalized content. They provide their email and password, then answer a simple question about their software background level to enable content personalization.

**Why this priority**: Core functionality - without registration, no users can access the platform. This is the entry point for all user engagement.

**Independent Test**: Can be fully tested by completing the registration flow end-to-end and verifying the user record exists in the database with their profile data.

**Acceptance Scenarios**:

1. **Given** a visitor on the registration page, **When** they enter a valid email, password (min 8 chars), and select their software background level, **Then** an account is created and they are redirected to the book interface as a logged-in user.

2. **Given** a visitor attempting to register, **When** they enter an email that already exists, **Then** they see a clear error message indicating the email is taken and are offered a link to sign in instead.

3. **Given** a visitor on registration, **When** they enter a password shorter than 8 characters, **Then** they see immediate validation feedback before form submission.

---

### User Story 2 - Existing User Login (Priority: P1)

A returning user wants to sign in to their existing account to continue learning where they left off.

**Why this priority**: Equal to registration - users must be able to return to their accounts. Together with P1-registration, these form the minimum viable auth system.

**Independent Test**: Can be tested by signing in with known credentials and verifying session creation and redirect to the book interface.

**Acceptance Scenarios**:

1. **Given** a registered user on the login page, **When** they enter correct email and password, **Then** a session is created and they are redirected to the book interface.

2. **Given** a user attempting login, **When** they enter incorrect credentials, **Then** they see a generic "Invalid credentials" message (not revealing which field is wrong for security).

3. **Given** a logged-in user, **When** they close the browser and return within 7 days, **Then** their session is still valid and they remain logged in.

---

### User Story 3 - User Logout (Priority: P2)

A logged-in user wants to sign out of their account, especially important on shared devices.

**Why this priority**: Essential security feature but secondary to core login/register functionality.

**Independent Test**: Can be tested by logging in, clicking logout, and verifying session is invalidated and user is redirected appropriately.

**Acceptance Scenarios**:

1. **Given** a logged-in user, **When** they click the logout button, **Then** their session is invalidated and they are redirected to the login page.

2. **Given** a user who just logged out, **When** they use the browser back button, **Then** they cannot access protected content and are redirected to login.

---

### User Story 4 - Profile Data Access for Personalization (Priority: P2)

The robolearn-interface needs to fetch the current user's profile data (including software background level) to personalize content display.

**Why this priority**: Enables the personalization feature (50 bonus points) but requires auth to be working first.

**Independent Test**: Can be tested by making an authenticated API request to the profile endpoint and verifying correct user data is returned.

**Acceptance Scenarios**:

1. **Given** a logged-in user on robolearn-interface, **When** the interface requests user profile data, **Then** the auth server returns the user's software background level and basic profile info.

2. **Given** an unauthenticated request to the profile endpoint, **When** no valid session exists, **Then** the server returns a 401 Unauthorized response.

---

### User Story 5 - Update Profile (Priority: P3)

A user wants to update their software background level as they progress in their learning journey.

**Why this priority**: Nice-to-have enhancement; initial profile is set at registration.

**Independent Test**: Can be tested by updating the software background field and verifying the change persists.

**Acceptance Scenarios**:

1. **Given** a logged-in user, **When** they change their software background level from "beginner" to "intermediate", **Then** the change is saved and reflected in subsequent profile fetches.

---

### Edge Cases

- What happens when a user's session expires mid-activity? → Graceful redirect to login with return URL preserved.
- How does the system handle rapid repeated login attempts? → Rate limiting: 5 attempts per minute per IP, then 15-minute lockout.
- What if the database is temporarily unavailable? → User-friendly error page, no sensitive info exposed.
- What happens if a user tries to access auth server directly without going through robolearn-interface? → Auth pages are accessible directly; API endpoints require proper CORS.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST allow users to create accounts with email and password.
- **FR-002**: System MUST validate email format before account creation.
- **FR-003**: System MUST enforce minimum password length of 8 characters.
- **FR-004**: System MUST hash passwords securely before storage (never store plaintext).
- **FR-005**: System MUST create a session upon successful login lasting 7 days.
- **FR-006**: System MUST invalidate sessions on logout.
- **FR-007**: System MUST capture software background level during registration (beginner/intermediate/advanced).
- **FR-008**: System MUST expose an authenticated endpoint to retrieve user profile data.
- **FR-009**: System MUST implement rate limiting on authentication endpoints.
- **FR-010**: System MUST support CORS for requests from robolearn-interface domain.
- **FR-011**: System MUST provide clear, user-friendly error messages without exposing system details.
- **FR-012**: System MUST prevent duplicate email registrations.
- **FR-013**: System MUST act as OAuth 2.1 / OIDC Provider with PKCE support for public clients.
- **FR-014**: System MUST provide JWKS endpoint for asymmetric token verification (RS256).
- **FR-015**: System MUST support dynamic client registration via admin-only endpoint.
- **FR-016**: System MUST send email verification on signup (Resend or SMTP).
- **FR-017**: System MUST support password reset via email.
- **FR-018**: System MUST distinguish between public clients (PKCE, no secret) and confidential clients (with secret).

### Non-Functional Requirements

- **NFR-001**: Auth server MUST be deployable independently from robolearn-interface.
- **NFR-002**: Architecture MUST support future addition of OAuth providers (Google, GitHub) without major refactoring.
- **NFR-003**: Architecture MUST support future SSO/OIDC provider capabilities for multi-client scenarios.
- **NFR-004**: System MUST use secure HTTP-only cookies for session management.

### Key Entities

- **User**: Core identity record containing id, email (unique), name (optional), emailVerified status, createdAt, updatedAt, role (user/admin). Managed by Better Auth.

- **Session**: Active login session containing id, userId, token, expiresAt, ipAddress, userAgent. Managed by Better Auth.

- **UserProfile**: Extended user data containing userId (FK), softwareBackground (enum: beginner/intermediate/advanced), createdAt, updatedAt. Custom extension for RoboLearn personalization.

- **OAuth Application**: Registered OAuth clients containing clientId, clientSecret (null for public clients), redirectURLs, type (public/confidential), metadata. Used for OIDC Provider functionality.

- **OAuth Access Token**: Issued tokens for OAuth clients containing accessToken, refreshToken, expiresAt, clientId, userId, scopes.

- **JWKS**: JSON Web Key Set storing RSA key pairs for asymmetric token signing (RS256).

## Constraints

- **C-001**: Must use Better Auth library for core authentication functionality.
- **C-002**: Must use Neon Postgres as the database.
- **C-003**: Must use Next.js as the server framework.
- **C-004**: Must use Drizzle ORM for database operations.
- **C-005**: No OAuth providers in initial implementation (email/password only).
- **C-006**: Onboarding limited to single question (software background level).

## Non-Goals

- **NG-001**: Social login (Google, GitHub) - deferred to future iteration.
- **NG-002**: Full hardware survey - simplified to single question for hackathon.
- **NG-003**: ~~Password reset/forgot password flow~~ - ✅ COMPLETED (FR-017)
- **NG-004**: ~~Email verification~~ - ✅ COMPLETED (FR-016)
- **NG-005**: Two-factor authentication - future enhancement.
- **NG-006**: Admin dashboard for user management - future enhancement.
- **NG-007**: ~~Acting as full SSO provider~~ - ✅ COMPLETED (FR-013, FR-014)
- **NG-008**: ~~JWKS support for offline token verification~~ - ✅ COMPLETED (FR-014)
- **NG-009**: ~~Scalable architecture for multiple apps~~ - ✅ COMPLETED (FR-013, FR-014, FR-015)

## Assumptions

- **A-001**: robolearn-interface will be the only client initially, but architecture should support multiple clients.
- **A-002**: Users will access from modern browsers (Chrome, Firefox, Safari, Edge - last 2 versions).
- **A-003**: Neon Postgres connection string will be provided via environment variable.
- **A-004**: Auth server will be deployed to a separate domain/subdomain from the book interface.
- **A-005**: Session duration of 7 days is acceptable for the learning platform use case.
- **A-006**: Rate limiting of 5 login attempts per minute is sufficient for MVP.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can complete registration (email, password, background selection) in under 60 seconds.
- **SC-002**: Users can sign in within 10 seconds of entering credentials.
- **SC-003**: 100% of registered users have a software background level captured.
- **SC-004**: Session validation requests return within 100ms under normal load.
- **SC-005**: Zero plaintext passwords stored in database (verified by security audit).
- **SC-006**: robolearn-interface can successfully authenticate users and fetch profile data.
- **SC-007**: Auth server handles 100 concurrent users without performance degradation.
- **SC-008**: OAuth authorization flow completes with PKCE in under 5 seconds.
- **SC-009**: JWKS endpoint returns valid RSA public keys for token verification.
- **SC-010**: Email verification emails delivered within 30 seconds of signup.
- **SC-011**: Admin can register new OAuth clients via custom endpoint.
- **SC-012**: JWKS endpoint returns valid RSA public keys (RS256) for token verification.
- **SC-013**: Client-side token verification works offline (no server calls per request).
- **SC-014**: Access tokens expire after 6 hours, refresh tokens after 7 days.
- **SC-015**: System supports 10+ apps with 10,000+ users each via SSO architecture.

## Dependencies

- **D-001**: Neon Postgres database instance must be provisioned.
- **D-002**: Domain/subdomain for auth server deployment must be available.
- **D-003**: robolearn-interface must implement auth client integration (separate task).

## Open Questions

None - all requirements are sufficiently clear for implementation. OAuth and extended features explicitly deferred to non-goals.
