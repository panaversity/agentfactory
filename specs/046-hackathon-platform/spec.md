# Feature Specification: Hackathon Platform

**Feature Branch**: `hackathon-platform`
**Created**: 2025-12-22
**Status**: Ready
**Input**: User description: "Design and complete hackathon app for Panaversity. AI-first hackathon platform to standardize ongoing hackathons. Integrates with existing SSO as OAuth client. Supports organizers, managers, judges, mentors, participants, and teams."

## Context

This specification defines a new Next.js 16 application (`apps/hackathon/`) within the existing Nx monorepo. The hackathon platform will consume the live SSO server (`apps/sso/`) as an OAuth client - **the SSO server will NOT be modified**. All SSO-related changes (adding OAuth client configuration, database seeding) will be documented as manual instructions for the user.

### Assumptions

- **A1**: The existing SSO server remains unchanged; hackathon app registers as a new OAuth public client
- **A2**: User authentication happens via SSO; hackathon app receives JWT with user ID and organization context
- **A3**: Hackathon data lives in a separate database from SSO (or shared Neon instance with separate schema)
- **A4**: The platform will use Next.js 16 patterns (async params, Turbopack, Cache Components)
- **A5**: AI features (team matching, judging assistance) are Phase 2 enhancements; core CRUD is Phase 1
- **A6**: Each hackathon belongs to an organization (multi-tenant via organization_id from SSO)

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Organizer Creates Hackathon (Priority: P1)

An organizer (user with "organizer" role in the hackathon platform) creates a new hackathon event with basic details: title, description, dates, registration deadline, team size limits, and judging criteria.

**Why this priority**: This is the foundational action - without hackathon events, no other functionality works. Every hackathon workflow starts here.

**Independent Test**: Can be fully tested by an organizer logging in, clicking "Create Hackathon", filling in required fields, and seeing the new hackathon listed. Delivers value by enabling the core event creation workflow.

**Acceptance Scenarios**:

1. **Given** a user authenticated via SSO with organizer role, **When** they navigate to the dashboard and click "Create Hackathon", **Then** they see a form with title, description, start date, end date, registration deadline, min/max team size, and judging criteria fields
2. **Given** an organizer on the create hackathon form, **When** they fill all required fields and submit, **Then** the hackathon is created and they are redirected to the hackathon management page
3. **Given** an organizer creating a hackathon, **When** they set an end date before the start date, **Then** they see a validation error preventing submission

---

### User Story 2 - Participant Registers and Joins/Creates Team (Priority: P1)

A participant browses available hackathons, registers for one, and either creates a new team or joins an existing team via invite code.

**Why this priority**: Participants are the primary users - hackathon success depends on participant registration and team formation. This is co-P1 with organizer creation.

**Independent Test**: Can be tested by a user logging in, browsing hackathons, clicking "Register", then either creating a team (getting an invite code) or entering an invite code to join a team.

**Acceptance Scenarios**:

1. **Given** an authenticated user, **When** they browse the hackathon catalog, **Then** they see a list of open hackathons with title, dates, and registration status
2. **Given** a user viewing a hackathon detail page, **When** they click "Register", **Then** they are registered as a participant and prompted to create or join a team
3. **Given** a registered participant, **When** they click "Create Team" and enter a team name, **Then** a team is created with them as team lead and they receive an invite code
4. **Given** a registered participant with an invite code, **When** they enter the code, **Then** they are added to the team if the team hasn't reached max size

---

### User Story 3 - Judge Scores Submissions (Priority: P2)

A judge reviews team submissions for an assigned hackathon, scores them against defined criteria, and provides feedback.

**Why this priority**: Judging is essential for hackathon completion but happens after submissions - hence P2. The system still has value without judging in early testing.

**Independent Test**: Can be tested by a judge logging in, seeing assigned hackathons, selecting a submission, scoring each criterion, and submitting their evaluation.

**Acceptance Scenarios**:

1. **Given** a user with judge role assigned to a hackathon, **When** they access the judging dashboard, **Then** they see a list of submissions awaiting their review
2. **Given** a judge viewing a submission, **When** they enter scores for each criterion and submit, **Then** the scores are saved and the submission is marked as "reviewed by [judge name]"
3. **Given** a judge, **When** they have reviewed all assigned submissions, **Then** their judging status shows as "complete"

---

### User Story 4 - Team Submits Project (Priority: P2)

A team lead submits their hackathon project before the deadline, including project name, description, repository link, demo link, and optional presentation.

**Why this priority**: Submissions are the culmination of participant work but depend on hackathon creation and team formation being complete first.

**Independent Test**: Can be tested by a team lead navigating to their hackathon, clicking "Submit Project", filling in submission details, and seeing confirmation.

**Acceptance Scenarios**:

1. **Given** a team lead during an active hackathon, **When** they navigate to their team page, **Then** they see a "Submit Project" button
2. **Given** a team lead on the submission form, **When** they fill in project name, description, repo URL, and demo URL, **Then** the submission is saved with timestamp
3. **Given** a hackathon past submission deadline, **When** a team lead tries to submit, **Then** they see an error indicating the deadline has passed

---

### User Story 5 - Organizer Manages Hackathon Roles (Priority: P2)

An organizer assigns roles (judge, mentor, manager) to registered users for a specific hackathon.

**Why this priority**: Role assignment enables judging and mentoring workflows but is administrative setup that can happen after initial hackathon creation.

**Independent Test**: Can be tested by an organizer selecting a hackathon, viewing registered users, and assigning a role (e.g., judge) to a user.

**Acceptance Scenarios**:

1. **Given** an organizer on the hackathon management page, **When** they click "Manage Roles", **Then** they see a list of users associated with the hackathon
2. **Given** an organizer viewing user list, **When** they assign "judge" role to a user, **Then** that user can access the judging dashboard for this hackathon
3. **Given** an organizer, **When** they remove a role from a user, **Then** the user loses access to role-specific features immediately

---

### User Story 6 - Mentor Provides Guidance to Teams (Priority: P3)

A mentor views teams assigned to them, sends messages/feedback, and tracks their engagement with teams.

**Why this priority**: Mentoring enhances hackathon quality but is optional for MVP; hackathons can run without mentor features.

**Independent Test**: Can be tested by a mentor logging in, seeing assigned teams, and sending a message to a team.

**Acceptance Scenarios**:

1. **Given** a user with mentor role for a hackathon, **When** they access the mentor dashboard, **Then** they see teams assigned to them
2. **Given** a mentor viewing a team, **When** they send a message, **Then** the team members receive the message in their team chat

---

### User Story 7 - Organizer Views Analytics Dashboard (Priority: P3)

An organizer views hackathon analytics: registration counts, team formation progress, submission status, and judging completion.

**Why this priority**: Analytics provide valuable insights but are enhancement features after core workflows are functional.

**Independent Test**: Can be tested by an organizer selecting a hackathon and viewing the analytics dashboard with registration and submission metrics.

**Acceptance Scenarios**:

1. **Given** an organizer on the hackathon management page, **When** they click "Analytics", **Then** they see registration count, team count, submission count, and judging progress
2. **Given** an organizer viewing analytics, **When** the data updates, **Then** the dashboard reflects current counts within 30 seconds

---

### Edge Cases

- What happens when a team leader leaves a team? (Team marked as "needs leader" status; organizer/manager manually assigns new leader via role management UI - auto-promotion deferred to Phase 2)
- What happens when registration deadline passes with incomplete teams? (Teams below min size cannot submit - show warning to participants)
- How does system handle concurrent judge scoring of the same submission? (Each judge's score is independent - stored separately, aggregated for final score)
- What happens if SSO is temporarily unavailable? (Show friendly error message; cached sessions remain valid until expiry)
- What happens when max teams registered for a hackathon? (Phase 2 enhancement - MVP has unlimited teams per hackathon)

## Requirements *(mandatory)*

### Functional Requirements

**Authentication & Authorization**

- **FR-001**: System MUST authenticate users via existing Panaversity SSO OAuth2/OIDC flow (PKCE for public client)
- **FR-002**: System MUST extract user ID and organization ID from SSO JWT token for multi-tenancy
- **FR-003**: System MUST support five hackathon-specific roles: organizer, manager, judge, mentor, participant
- **FR-004**: System MUST scope all hackathon data to the user's organization (multi-tenant isolation)

**Hackathon Management**

- **FR-005**: Organizers MUST be able to create hackathons with: title, description, start date, end date, registration deadline, team size constraints (min/max), and judging criteria
- **FR-006**: Organizers MUST be able to edit hackathon details before the start date
- **FR-007**: Organizers MUST be able to publish/unpublish hackathons to control visibility
- **FR-008**: System MUST display hackathon status: draft, open for registration, active, judging, completed

**Team & Participant Management**

- **FR-009**: Participants MUST be able to register for open hackathons
- **FR-010**: Participants MUST be able to create teams with a unique name, generating an invite code
- **FR-011**: Participants MUST be able to join teams using an invite code if team hasn't reached max size
- **FR-012**: Team leads MUST be able to remove members from their team before submission deadline
- **FR-013**: System MUST prevent teams below minimum size from submitting

**Submissions**

- **FR-014**: Team leads MUST be able to submit projects with: name, description, repository URL, demo URL, and optional presentation link
- **FR-015**: System MUST enforce submission deadline - reject submissions after deadline
- **FR-016**: System MUST allow one submission per team per hackathon (updates allowed until deadline)

**Judging**

- **FR-017**: System MUST display judging criteria defined by organizer (e.g., innovation, execution, impact)
- **FR-018**: Judges MUST be able to score submissions on each criterion (1-10 scale)
- **FR-019**: Judges MUST be able to provide written feedback per submission
- **FR-020**: System MUST calculate aggregate scores (average across criteria and judges)
- **FR-021**: Organizers MUST be able to view final rankings based on aggregate scores

**Role Assignment**

- **FR-022**: Organizers MUST be able to assign/revoke judge, mentor, manager roles per hackathon
- **FR-023**: Managers MUST have same permissions as organizers except creating/deleting hackathons

**Communication**

- **FR-024**: System MUST support team-internal messaging (basic chat or comments)
- **FR-025**: Mentors MUST be able to send messages to their assigned teams

### Key Entities

- **Hackathon**: Represents a hackathon event with title, description, dates, status, organization ownership, and configuration (team size, judging criteria)
- **Team**: A group of participants for a specific hackathon; has name, invite code, leader reference, and hackathon reference
- **TeamMember**: Junction between user and team, storing join timestamp and role within team (leader/member)
- **Submission**: A team's project submission with name, description, URLs, timestamp, and hackathon reference
- **Score**: A judge's evaluation of a submission; includes scores per criterion, feedback, and judge reference
- **HackathonRole**: Per-hackathon role assignment linking user to hackathon with role type (organizer/manager/judge/mentor/participant)
- **JudgingCriterion**: Defines a scoring criterion for a hackathon (e.g., "Innovation", "Technical Execution") with weight

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Organizers can create a new hackathon and see it listed within 30 seconds of form submission
- **SC-002**: Participants can complete registration and team creation flow in under 3 minutes
- **SC-003**: Judges can score a submission (all criteria) in under 5 minutes
- **SC-004**: System supports 500 concurrent participants during peak registration periods without degradation
- **SC-005**: 95% of users complete their primary task (register, submit, judge) on first attempt
- **SC-006**: Page load time under 2 seconds for all core workflows (catalog, registration, submission, judging)
- **SC-007**: Zero unauthorized cross-organization data access (validated via security testing)

## Constraints

- **C1**: SSO server (`apps/sso/`) is NOT to be modified by this implementation - all SSO changes are manual instructions
- **C2**: Must be a new app within the existing Nx monorepo structure
- **C3**: Must use Next.js 16 patterns (async params, Turbopack, Cache Components where applicable)
- **C4**: Must integrate with existing Panaversity organization model from SSO

## Non-Goals

- **NG-1**: AI-powered team matching (Phase 2 enhancement)
- **NG-2**: AI-assisted judging automation (Phase 2 enhancement)
- **NG-3**: Video conferencing integration
- **NG-4**: Payment processing for hackathon entry fees
- **NG-5**: Mobile native apps (web-responsive only for MVP)
- **NG-6**: Notification system (email/push) - manual checks for MVP
- **NG-7**: Sponsor management and branding customization
- **NG-8**: Max teams capacity limit per hackathon (unlimited in MVP)
- **NG-9**: Automatic team leader promotion (manual assignment via organizer in MVP)

## SSO Integration Instructions (Manual Steps Required)

The following changes must be made to the SSO server by the user. **Do not execute these automatically.**

### 1. Add OAuth Client to trusted-clients.ts

Add to `apps/sso/src/lib/trusted-clients.ts` in the `TRUSTED_CLIENTS` array:

```typescript
{
  clientId: "hackathon-public-client",
  name: "Panaversity Hackathon Platform",
  type: "public" as const,
  redirectUrls: getRedirectUrls([
    "http://localhost:3002/api/auth/callback",
    "https://hackathon.panaversity.org/api/auth/callback",
  ]),
  disabled: false,
  skipConsent: true,
  metadata: {},
},
```

### 2. Add Client Description

Add to `CLIENT_DESCRIPTIONS` in the same file:

```typescript
"hackathon-public-client": {
  purpose: "Panaversity Hackathon Platform",
  audience: "Hackathon organizers, participants, judges, and mentors",
  security: "Public client with PKCE, no client secret",
},
```

### 3. Run SSO Seed Script

After adding the client configuration:

```bash
cd apps/sso
pnpm run seed:setup
```

This registers the OAuth client in the SSO database for token storage.

## Dependencies

- **D1**: Existing SSO server (`apps/sso/`) with Better Auth OAuth2/OIDC provider
- **D2**: Neon PostgreSQL database (new instance or shared with separate schema)
- **D3**: Next.js 16.x (new app to be created)
- **D4**: Drizzle ORM for database operations

## Risks

- **R1**: SSO JWT token structure changes could break hackathon auth - mitigate by using stable claims (sub, org_id)
- **R2**: High concurrent load during hackathon registration peaks - mitigate with caching and rate limiting
- **R3**: Team formation complexity (invites, removals, promotions) - mitigate with clear status UI
