# Feature Specification: SSO Client Authentication Pages

**Feature Branch**: `001-sso-client-auth-pages`
**Created**: 2025-11-21
**Updated**: 2025-11-23
**Status**: Draft
**Input**: User description: "Build authentication UI pages that connect to the existing backend. Create Sign Up, Sign In, Forgot Password, Reset Password, and Email Verification pages using shadcn UI components from @repo/ui package. Pages should use authClient from @repo/auth-config/client to call existing SSO server endpoints running on localhost:3000."

## Clarifications

### Session 2025-11-23

- Q: When users complete registration or sign-in, what should the redirect targets be? → A: Redirect to callback URL if provided in query parameters, otherwise redirect to application dashboard (no intermediate success page)
- Q: What mechanism should be used after 5 failed login attempts? → A: Show CAPTCHA challenge after 5 failed attempts (user can continue trying with CAPTCHA verification)
- Q: What should happen when already logged-in users access auth pages? → A: Automatically redirect to dashboard (or callback URL if provided in query parameters)
- Q: What should be the API timeout duration before showing an error? → A: 30 seconds timeout (standard web application timeout)
- Q: What should happen when registering with an unverified existing email? → A: Allow registration and resend verification email (overwrite pending unverified account)

## User Scenarios & Testing *(mandatory)*

### User Story 1 - User Registration with Email and Password (Priority: P1)

As a new user, I need to create an account using my email address and password so that I can access SSO-protected applications.

**Why this priority**: This is the foundational authentication method that enables users to join the system. Without registration, no users can be onboarded, making this the most critical user journey.

**Independent Test**: Can be fully tested by navigating to the registration page, submitting a valid email and password, and verifying that a user account is created in the database. Delivers immediate value as the primary onboarding mechanism.

**Acceptance Scenarios**:

1. **Given** I am on the registration page, **When** I enter a valid email address, a secure password, and my name, **Then** my account is created and I am redirected to the callback URL if provided, otherwise to the application dashboard
2. **Given** I am on the registration page, **When** I enter an email that already exists, **Then** I see an error message indicating the email is already registered
3. **Given** I am on the registration page, **When** I enter a weak password (less than 8 characters), **Then** I see an error message indicating password requirements
4. **Given** I am on the registration page, **When** I enter an invalid email format, **Then** I see an error message indicating the email format is invalid
5. **Given** I successfully register, **When** I check my email inbox, **Then** I receive a verification email (if email verification is enabled)

---

### User Story 2 - User Sign In with Email and Password (Priority: P1)

As a registered user, I need to sign in using my email and password so that I can authenticate and access protected applications through the SSO system.

**Why this priority**: Sign-in is equally critical to registration as it enables returning users to access the system. Both form the core authentication flow.

**Independent Test**: Can be fully tested by navigating to the sign-in page, entering valid credentials, and verifying successful authentication with a valid session token. Delivers value by allowing existing users to authenticate.

**Acceptance Scenarios**:

1. **Given** I am on the sign-in page, **When** I enter my correct email and password, **Then** I am authenticated and redirected to the application dashboard or callback URL
2. **Given** I am on the sign-in page, **When** I enter an incorrect password, **Then** I see an error message indicating invalid credentials
3. **Given** I am on the sign-in page, **When** I enter an email that doesn't exist, **Then** I see an error message indicating invalid credentials (no user enumeration)
4. **Given** I am signed in, **When** I close the browser and return later, **Then** my session persists based on "remember me" preference
5. **Given** I have failed to sign in 5 times consecutively, **Then** I am required to complete a CAPTCHA challenge before attempting further login attempts

---

### User Story 3 - Password Reset Flow (Priority: P2)

As a user who forgot my password, I need to reset it using my email address so that I can regain access to my account.

**Why this priority**: While not required for initial MVP launch, password reset is essential for production readiness and user retention. Users will inevitably forget passwords, and without this flow, they'd be permanently locked out.

**Independent Test**: Can be fully tested by clicking "Forgot Password", submitting an email, receiving a reset link, and successfully setting a new password. Delivers value by preventing account abandonment.

**Acceptance Scenarios**:

1. **Given** I am on the sign-in page, **When** I click "Forgot Password" and enter my email, **Then** I receive a password reset link via email
2. **Given** I received a password reset email, **When** I click the reset link within 1 hour, **Then** I am taken to a page where I can set a new password
3. **Given** I am on the password reset page, **When** I enter a new secure password and confirm it, **Then** my password is updated and I can sign in with the new password
4. **Given** I received a password reset email, **When** I click the reset link after 1 hour, **Then** I see an error message that the link has expired
5. **Given** I enter an email that doesn't exist, **When** I submit the forgot password form, **Then** I see a generic success message (no user enumeration)

---

### User Story 4 - Social Login (OAuth) (Priority: P3)

As a user, I want to sign in using my GitHub or Google account so that I can authenticate quickly without creating a new password.

**Why this priority**: Social login is a convenience feature that improves user experience but is not required for core functionality. The system already supports email/password authentication.

**Independent Test**: Can be fully tested by clicking "Sign in with GitHub/Google", completing OAuth flow, and verifying successful authentication. Delivers value by reducing friction for users who prefer social login.

**Acceptance Scenarios**:

1. **Given** I am on the sign-in page, **When** I click "Sign in with GitHub", **Then** I am redirected to GitHub's authorization page
2. **Given** I authorize the application on GitHub, **When** I am redirected back, **Then** I am signed in and my profile information is linked to my account
3. **Given** I previously signed in with email/password, **When** I sign in with GitHub using the same email, **Then** my accounts are linked and I can use either method
4. **Given** I am on the registration page, **When** I click "Sign up with Google", **Then** my account is created using Google profile information

---

### User Story 5 - OIDC Integration for External Applications (Priority: P2) [OPTIONAL FOR NOW]

As an external application developer, I need to integrate with the SSO client using OIDC protocol so that my users can authenticate through the central SSO system.

**Why this priority**: OIDC integration is essential for the SSO system's purpose - enabling single sign-on across multiple applications. This is a core requirement but secondary to basic user authentication.

**Independent Test**: Can be fully tested by registering an OAuth application, initiating an OIDC flow from a test application, and verifying successful authentication callback with tokens. Delivers value by enabling the primary SSO use case.

**Acceptance Scenarios**:

1. **Given** an external application initiates an OIDC authorization request, **When** I am not signed in, **Then** I am redirected to the sign-in page with the client application context
2. **Given** I sign in successfully during an OIDC flow, **When** the application is trusted, **Then** I am redirected back to the application with an authorization code
3. **Given** I sign in successfully during an OIDC flow, **When** the application requires consent, **Then** I see a consent page listing the requested permissions
4. **Given** I grant consent, **When** I approve permissions, **Then** I am redirected to the application callback URL with tokens
5. **Given** I am already signed in, **When** an application initiates an OIDC flow, **Then** I am automatically redirected back without re-authentication (SSO)

---

### Edge Cases

- What happens when a user tries to register with an email that already exists but isn't verified?
- How does the system handle concurrent login attempts from different devices?
- What happens if the OAuth provider (GitHub/Google) returns an error during authentication?
- How does the system handle expired OIDC authorization requests?
- What happens when a user's session expires while they're in the middle of an OIDC flow?
- How does the system handle special characters in email addresses or names?
- What happens if the user closes the browser during the OAuth redirect flow?
- How does the system handle database connection failures during authentication?
- What happens when the SSO server (backend) is unavailable or returns an error?
- How does the UI handle slow network connections or API timeouts?
- What happens when a user clicks the password reset link multiple times?
- How does the system handle users who navigate directly to the reset password page without a token?
- What happens when a user tries to use an already-used password reset token?
- How does the UI handle validation errors from the server (e.g., password too common)?
- What happens when a user refreshes the page during form submission?

### Edge Case Resolutions

- **Already logged-in users visiting auth pages**: Users who are already authenticated are automatically redirected to the dashboard (or callback URL if provided) without displaying auth forms
- **API timeout handling**: All authentication API calls timeout after 30 seconds, displaying a user-friendly error message prompting retry
- **Registration with unverified existing email**: System allows re-registration with an unverified email address, overwrites the pending account, and resends the verification email

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a registration page where users can create accounts using email and password
- **FR-002**: System MUST validate email addresses using RFC 5322 standard format
- **FR-003**: System MUST enforce password requirements (minimum 8 characters, at least one uppercase, one lowercase, one number, one special character)
- **FR-004**: System MUST provide a sign-in page where users can authenticate using email and password
- **FR-005**: System MUST display clear error messages for failed authentication attempts without revealing whether the email exists (prevent user enumeration)
- **FR-006**: System MUST support "remember me" functionality to persist user sessions across browser restarts
- **FR-007**: System MUST provide a password reset flow accessible from the sign-in page
- **FR-008**: System MUST send password reset emails with time-limited tokens (1-hour expiration)
- **FR-009**: System MUST provide social login options for GitHub and Google OAuth providers
- **FR-010**: System MUST link social accounts to existing accounts when email addresses match
- **FR-011**: System MUST support OIDC authorization flows for external applications
- **FR-012**: System MUST display a consent page for untrusted OIDC applications requesting access
- **FR-013**: System MUST skip consent for trusted applications registered in the system
- **FR-014**: System MUST maintain session state across OIDC flows to enable true single sign-on
- **FR-015**: System MUST redirect users back to the requesting application after successful authentication
- **FR-016**: System MUST log all authentication events (sign-in, sign-up, password reset, OAuth events)
- **FR-017**: System MUST display a CAPTCHA challenge after 5 failed login attempts within a 15-minute period to prevent brute force attacks
- **FR-018**: System MUST provide responsive UI that works on mobile, tablet, and desktop devices
- **FR-019**: System MUST use HTTPS for all authentication endpoints in production
- **FR-020**: System MUST sanitize all user inputs to prevent XSS and injection attacks
- **FR-021**: System MUST provide a dedicated Sign Up page at `/auth/signup` that collects email, password, and name
- **FR-022**: System MUST provide a dedicated Sign In page at `/auth/signin` that collects email and password
- **FR-023**: System MUST provide a Forgot Password page at `/auth/forgot-password` that collects email address
- **FR-024**: System MUST provide a Reset Password page at `/auth/reset-password` that accepts a token parameter and collects new password
- **FR-025**: System MUST provide an Email Verification confirmation page that handles email verification tokens
- **FR-026**: System MUST display social login buttons (GitHub and Google) on both Sign Up and Sign In pages
- **FR-027**: System MUST use reusable UI components (form inputs, buttons, cards) for consistent design across all authentication pages
- **FR-028**: System MUST display inline validation errors for form fields (email format, password strength, required fields)
- **FR-029**: System MUST show loading states during API calls to provide feedback to users
- **FR-030**: System MUST redirect users to callback URL (from query parameter) after successful authentication, or to application dashboard at `/dashboard` if no callback URL is provided
- **FR-031**: Sign In page MUST include a "Forgot Password?" link that navigates to the Forgot Password page
- **FR-032**: All authentication pages MUST include a link to switch between Sign In and Sign Up pages
- **FR-033**: System MUST display success messages after password reset request submission
- **FR-034**: System MUST validate token expiration and display appropriate error messages for expired reset links
- **FR-035**: System MUST provide OIDC login page at `/auth/login` for external application authentication flows
- **FR-036**: System MUST provide OIDC consent page at `/auth/consent` showing requesting application details and permissions
- **FR-037**: System MUST redirect already authenticated users from auth pages (signup, signin, forgot-password) to dashboard or callback URL without showing the auth forms
- **FR-038**: System MUST timeout authentication API calls after 30 seconds and display a user-friendly error message with retry option
- **FR-039**: System MUST allow re-registration with an unverified email address, overwriting the pending account and resending the verification email

### Key Entities

- **User**: Represents an authenticated user in the system with attributes including email (unique identifier), hashed password, name, email verification status, account creation timestamp, and last sign-in timestamp
- **Session**: Represents an active user session with attributes including session token, user reference, expiration time, device information, and creation timestamp
- **OAuth Account**: Represents a linked social account (GitHub/Google) with attributes including provider name, provider user ID, user reference, access token, refresh token, and profile data
- **Password Reset Token**: Represents a time-limited token for password reset with attributes including token value, user reference, expiration time, and used status
- **OAuth Application**: Represents an external application registered for OIDC with attributes including client ID, client secret, application name, redirect URLs, trusted status, and consent requirements
- **OAuth Consent**: Represents user consent for an application with attributes including user reference, application reference, granted scopes, and consent timestamp
- **Authentication Page**: Represents a UI page in the client application with attributes including page route (e.g., `/auth/signin`), form fields required, validation rules, API endpoint to call, success redirect target, and error handling behavior
- **UI Component**: Represents a reusable interface element (form input, button, card, alert) with attributes including component type, styling properties, validation behavior, and accessibility features

## Assumptions and Dependencies

### Assumptions

- **A-001**: Users have modern web browsers that support JavaScript and cookies
- **A-002**: Users have access to their email inbox to receive verification and password reset emails
- **A-003**: The SSO server backend is fully operational and accessible at the configured base URL
- **A-004**: Email delivery service (Resend) is configured and operational for sending authentication emails
- **A-005**: OAuth providers (GitHub, Google) are correctly configured with valid client IDs and secrets
- **A-006**: Users understand basic web navigation and form filling
- **A-007**: The application runs in environments with stable network connectivity for API calls

### Dependencies

- **D-001**: SSO Server must be running and accessible on port 3000 with all authentication endpoints operational
- **D-002**: Shared UI component library (@repo/ui) must provide necessary components (Input, Form, Card, Button, Alert)
- **D-003**: Authentication client library (@repo/auth-config/client) must expose methods for all authentication operations
- **D-004**: Database must be accessible and contain proper schema for user, session, and OAuth tables
- **D-005**: OIDC provider configuration in SSO server must include correct login and consent page paths
- **D-006**: Shadcn UI components must be properly installed and configured in the shared UI package
- **D-007**: Environment variables must be set for API base URL and OAuth provider configuration

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can complete account registration in under 90 seconds from start to finish
- **SC-002**: Users can sign in successfully in under 15 seconds
- **SC-003**: Password reset emails are delivered within 2 minutes of request
- **SC-004**: OIDC authorization flows complete in under 10 seconds for authenticated users
- **SC-005**: System successfully handles 1000 concurrent authentication requests without performance degradation
- **SC-006**: 95% of users successfully complete registration on their first attempt without errors
- **SC-007**: Failed authentication attempts are logged with 100% accuracy for security monitoring
- **SC-008**: Social login success rate exceeds 90% for users with valid OAuth provider accounts
- **SC-009**: Zero XSS or SQL injection vulnerabilities in authentication flows as verified by security scan
- **SC-010**: Pages load and become interactive in under 2 seconds on 4G mobile connections
- **SC-011**: Authentication forms are fully accessible and usable with screen readers
- **SC-012**: System maintains 99.9% uptime for authentication services
- **SC-013**: Users receive immediate visual feedback (within 100ms) when interacting with form elements
- **SC-014**: Form validation errors appear inline within 200ms of user input
- **SC-015**: Users can navigate entire authentication flow using only keyboard (full keyboard accessibility)
- **SC-016**: All UI components maintain consistent visual design across all authentication pages
- **SC-017**: Social login buttons are clearly distinguishable and labeled with provider names
- **SC-018**: Users can complete password reset flow without leaving the application (no external tab navigation required)
