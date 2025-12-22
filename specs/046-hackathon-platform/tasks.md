# Tasks: Hackathon Platform

**Feature**: 046-hackathon-platform
**Input**: Design documents from `/specs/046-hackathon-platform/`
**Prerequisites**: plan.md, spec.md, data-model.md, research.md, quickstart.md

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## User Story Mapping

| Story | Priority | Title | Independent Test |
|-------|----------|-------|------------------|
| US1 | P1 | Organizer Creates Hackathon | Create hackathon, see it listed |
| US2 | P1 | Participant Registers & Joins Team | Register, create/join team |
| US3 | P2 | Judge Scores Submissions | View submission, score criteria |
| US4 | P2 | Team Submits Project | Submit project with URLs |
| US5 | P2 | Organizer Manages Roles | Assign/revoke roles |
| US6 | P3 | Mentor Provides Guidance | View teams, send messages |
| US7 | P3 | Organizer Views Analytics | View dashboard metrics |

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Scaffold Next.js 16 app within Nx monorepo

- [ ] T001 Use `npx nx generate @nx/next:application hackathon --directory=apps/hackathon --style=css --appDir=true` to scaffold app. Verify with `ls apps/hackathon/`.
- [ ] T002 [P] Configure port 3002 in `apps/hackathon/project.json` serve target options.
- [ ] T003 [P] Use `cd apps/hackathon && pnpm add next@^16.0.0 react@^19.0.0 react-dom@^19.0.0` to install core deps.
- [ ] T004 [P] Use `cd apps/hackathon && pnpm add drizzle-orm@^0.36.0 @neondatabase/serverless@^0.10.0` for database.
- [ ] T005 [P] Use `cd apps/hackathon && pnpm add arctic@^2.0.0 jose@^6.1.0 iron-session@^8.0.0` for OAuth.
- [ ] T006 [P] Use `cd apps/hackathon && pnpm add nanoid@^5.0.0 zod@^3.23.0` for utils.
- [ ] T007 [P] Use `cd apps/hackathon && pnpm add @radix-ui/react-alert-dialog @radix-ui/react-dialog @radix-ui/react-dropdown-menu @radix-ui/react-label @radix-ui/react-slot @radix-ui/react-tooltip lucide-react clsx tailwind-merge class-variance-authority` for UI.
- [ ] T008 [P] Use `cd apps/hackathon && pnpm add -D drizzle-kit@^0.28.0 tsx@^4.21.0 tailwindcss@^3.4.0 autoprefixer@^10.4.0 postcss@^8.4.0` for dev deps.
- [ ] T009 Create directory structure: `mkdir -p apps/hackathon/src/{app/{api/{auth,hackathons,teams,submissions,scores},'(auth)/{login,callback}','(dashboard)/{dashboard,hackathons,teams}'},components/{ui,hackathons,teams,layout},db/queries,lib/{auth,validation},types}`.
- [ ] T010 [P] Create `apps/hackathon/.env.local.example` with all environment variables from quickstart.md.
- [ ] T011 [P] Create `apps/hackathon/tailwind.config.ts` with content paths and primary color theme.
- [ ] T012 [P] Create `apps/hackathon/postcss.config.js` with tailwindcss and autoprefixer plugins.
- [ ] T013 Use `cd apps/hackathon && npx shadcn@latest init` to initialize shadcn/ui.
- [ ] T014 Use `cd apps/hackathon && npx shadcn@latest add button card dialog dropdown-menu input label table form textarea select` to add components.

**Checkpoint**: `pnpm dev` runs on port 3002, Tailwind compiles, no TypeScript errors.

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Database schema, OAuth integration, session management - MUST complete before any user story

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### 2.1 Database Schema

- [ ] T015 Create `apps/hackathon/src/db/schema.ts` with all 8 tables from data-model.md. **Doc**: Fetch drizzle-orm docs via Context7 for pgTable and relations patterns.
- [ ] T016 Create `apps/hackathon/drizzle.config.ts` with schema path and Neon connection.
- [ ] T017 Create `apps/hackathon/src/db/index.ts` with Neon pool and drizzle instance export.
- [ ] T018 Use `cd apps/hackathon && pnpm db:generate` to generate SQL migrations. Verify with `ls drizzle/`.
- [ ] T019 Use `cd apps/hackathon && pnpm db:push` to apply migrations to database.

### 2.2 OAuth Client (SSO Integration)

- [ ] T020 Create `apps/hackathon/src/lib/auth/oauth-client.ts` with Arctic OAuth2Client for PKCE flow. **Doc**: Fetch arctic docs via Context7 for OAuth2Client configuration.
- [ ] T021 Create `apps/hackathon/src/lib/auth/session.ts` with iron-session for encrypted cookie sessions. **Doc**: Fetch iron-session docs via Context7 for getIronSession patterns.
- [ ] T022 Create `apps/hackathon/src/lib/auth/jwt-verify.ts` with jose for JWKS verification. **Doc**: Fetch jose docs via Context7 for jwtVerify and createRemoteJWKSet.
- [ ] T023 Create `apps/hackathon/src/app/api/auth/login/route.ts` with PKCE state/verifier generation and SSO redirect.
- [ ] T024 Create `apps/hackathon/src/app/api/auth/callback/route.ts` with code exchange, JWT verification, session creation.
- [ ] T025 Create `apps/hackathon/src/app/api/auth/logout/route.ts` with session destruction and SSO logout redirect.
- [ ] T026 Create `apps/hackathon/middleware.ts` with route protection for authenticated paths.
- [ ] T027 Create `apps/hackathon/src/app/(auth)/login/page.tsx` with SSO login button.

### 2.3 Authorization Utilities

- [ ] T028 Create `apps/hackathon/src/lib/auth/permissions.ts` with ROLE_PERMISSIONS map and requireRole/can helpers.
- [ ] T029 Create `apps/hackathon/src/types/index.ts` with SessionData, HackathonRole, and common types.

### 2.4 Validation Schemas

- [ ] T030 [P] Create `apps/hackathon/src/lib/validation/hackathon.ts` with createHackathonSchema and updateHackathonSchema.
- [ ] T031 [P] Create `apps/hackathon/src/lib/validation/team.ts` with createTeamSchema and joinTeamSchema.
- [ ] T032 [P] Create `apps/hackathon/src/lib/validation/submission.ts` with createSubmissionSchema and scoreSubmissionSchema.

### 2.5 Base Layouts

- [ ] T033 Create `apps/hackathon/src/app/layout.tsx` with root layout, fonts, and Tailwind styles.
- [ ] T034 Create `apps/hackathon/src/app/(dashboard)/layout.tsx` with authenticated layout wrapper and navigation.
- [ ] T035 Create `apps/hackathon/src/components/layout/navbar.tsx` with user menu and logout button.
- [ ] T036 Create `apps/hackathon/src/components/layout/sidebar.tsx` with role-based navigation links.

**Checkpoint**: OAuth login flow works, session persists, middleware protects routes.

---

## Phase 3: User Story 1 - Organizer Creates Hackathon (Priority: P1) 🎯 MVP

**Goal**: Organizers can create hackathons with all required fields and see them listed on dashboard.

**Independent Test**: Log in as any user, click "Create Hackathon", fill form, submit, see hackathon in list.

### Database Queries for US1

- [ ] T037 [US1] Create `apps/hackathon/src/db/queries/hackathons.ts` with getHackathonsByOrg, getHackathonById, createHackathon, getUserRole.

### API Routes for US1

- [ ] T038 [US1] Create `apps/hackathon/src/app/api/hackathons/route.ts` with GET (list) and POST (create) handlers.
- [ ] T039 [US1] Create `apps/hackathon/src/app/api/hackathons/[id]/route.ts` with GET, PATCH, DELETE handlers. Use Next.js 16 async params pattern.
- [ ] T040 [US1] Create `apps/hackathon/src/app/api/hackathons/[id]/publish/route.ts` with POST to toggle published status.

### UI Components for US1

- [ ] T041 [P] [US1] Create `apps/hackathon/src/components/hackathons/hackathon-card.tsx` with status badge, dates, and action buttons.
- [ ] T042 [P] [US1] Create `apps/hackathon/src/components/hackathons/hackathon-status-badge.tsx` with color-coded status.
- [ ] T043 [US1] Create `apps/hackathon/src/components/hackathons/create-button.tsx` with dialog trigger.
- [ ] T044 [US1] Create `apps/hackathon/src/components/hackathons/create-form.tsx` with all hackathon fields and validation.
- [ ] T045 [US1] Create `apps/hackathon/src/components/hackathons/edit-form.tsx` for updating hackathon details.

### Pages for US1

- [ ] T046 [US1] Create `apps/hackathon/src/app/(dashboard)/dashboard/page.tsx` with hackathon list and create button.
- [ ] T047 [US1] Create `apps/hackathon/src/app/(dashboard)/hackathons/[id]/page.tsx` with hackathon detail view. Use Next.js 16 async params.
- [ ] T048 [US1] Create `apps/hackathon/src/app/(dashboard)/hackathons/[id]/manage/page.tsx` with organizer management tabs.

**Checkpoint**: US1 complete - organizers can create hackathons, see them listed, edit details, publish/unpublish.

---

## Phase 4: User Story 2 - Participant Registers & Joins Team (Priority: P1)

**Goal**: Participants can browse hackathons, register, and create or join teams.

**Independent Test**: Log in, browse hackathons, register for one, create a team (get invite code), have another user join with code.

### Database Queries for US2

- [ ] T049 [US2] Create `apps/hackathon/src/db/queries/teams.ts` with getTeamsByHackathon, getTeamById, createTeam, joinTeamByInviteCode, getUserTeamForHackathon.
- [ ] T050 [US2] Create `apps/hackathon/src/db/queries/participants.ts` with registerForHackathon, getRegistrationStatus, getParticipants.

### API Routes for US2

- [ ] T051 [US2] Create `apps/hackathon/src/app/api/hackathons/[id]/register/route.ts` with POST for participant registration.
- [ ] T052 [US2] Create `apps/hackathon/src/app/api/teams/route.ts` with GET (list user's teams) and POST (create team).
- [ ] T053 [US2] Create `apps/hackathon/src/app/api/teams/[id]/route.ts` with GET, PATCH, DELETE handlers.
- [ ] T054 [US2] Create `apps/hackathon/src/app/api/teams/join/route.ts` with POST for joining via invite code.
- [ ] T055 [US2] Create `apps/hackathon/src/app/api/teams/[id]/members/route.ts` with GET (list), DELETE (remove member).

### UI Components for US2

- [ ] T056 [P] [US2] Create `apps/hackathon/src/components/hackathons/hackathon-catalog.tsx` with browsable list and filters.
- [ ] T057 [P] [US2] Create `apps/hackathon/src/components/hackathons/register-button.tsx` with registration action.
- [ ] T058 [P] [US2] Create `apps/hackathon/src/components/teams/team-card.tsx` with member count and invite code display.
- [ ] T059 [US2] Create `apps/hackathon/src/components/teams/create-team-form.tsx` with team name and description inputs.
- [ ] T060 [US2] Create `apps/hackathon/src/components/teams/join-team-form.tsx` with invite code input.
- [ ] T061 [US2] Create `apps/hackathon/src/components/teams/team-members-list.tsx` with member list and remove button.
- [ ] T062 [US2] Create `apps/hackathon/src/components/teams/invite-code-display.tsx` with copy-to-clipboard functionality.

### Pages for US2

- [ ] T063 [US2] Create `apps/hackathon/src/app/(dashboard)/hackathons/page.tsx` with public hackathon catalog.
- [ ] T064 [US2] Create `apps/hackathon/src/app/(dashboard)/teams/page.tsx` with user's teams list.
- [ ] T065 [US2] Create `apps/hackathon/src/app/(dashboard)/teams/[id]/page.tsx` with team detail and member management.
- [ ] T066 [US2] Update `apps/hackathon/src/app/(dashboard)/hackathons/[id]/page.tsx` to show registration status and team info.

**Checkpoint**: US2 complete - participants can browse, register, create teams, join with invite code.

---

## Phase 5: User Story 4 - Team Submits Project (Priority: P2)

**Goal**: Team leads can submit projects with name, description, and URLs before deadline.

**Independent Test**: As team lead, go to team page, click "Submit Project", fill form, see submission listed.

### Database Queries for US4

- [ ] T067 [US4] Create `apps/hackathon/src/db/queries/submissions.ts` with getSubmissionsByHackathon, getSubmissionByTeam, createSubmission, updateSubmission.

### API Routes for US4

- [ ] T068 [US4] Create `apps/hackathon/src/app/api/submissions/route.ts` with POST (create/update submission).
- [ ] T069 [US4] Create `apps/hackathon/src/app/api/submissions/[id]/route.ts` with GET, PATCH handlers.
- [ ] T070 [US4] Create `apps/hackathon/src/app/api/hackathons/[id]/submissions/route.ts` with GET (list all submissions).

### UI Components for US4

- [ ] T071 [P] [US4] Create `apps/hackathon/src/components/submissions/submission-card.tsx` with project info and URLs.
- [ ] T072 [US4] Create `apps/hackathon/src/components/submissions/submit-form.tsx` with project name, description, URLs, deadline check.
- [ ] T073 [US4] Create `apps/hackathon/src/components/submissions/submission-status.tsx` with submitted/not-submitted indicator.

### Pages for US4

- [ ] T074 [US4] Update `apps/hackathon/src/app/(dashboard)/teams/[id]/page.tsx` to show submission form and status.
- [ ] T075 [US4] Create `apps/hackathon/src/app/(dashboard)/hackathons/[id]/submissions/page.tsx` for viewing all submissions (organizer view).

**Checkpoint**: US4 complete - team leads can submit projects, updates allowed until deadline, late submissions blocked.

---

## Phase 6: User Story 3 - Judge Scores Submissions (Priority: P2)

**Goal**: Judges can view submissions, score against criteria, and provide feedback.

**Independent Test**: As judge, access judging dashboard, see assigned submissions, score each criterion, submit scores.

### Database Queries for US3

- [ ] T076 [US3] Create `apps/hackathon/src/db/queries/scores.ts` with getScoresBySubmission, createScore, getJudgeProgress, getLeaderboard.
- [ ] T077 [US3] Create `apps/hackathon/src/db/queries/criteria.ts` with getCriteriaByHackathon, createCriterion, updateCriterion.

### API Routes for US3

- [ ] T078 [US3] Create `apps/hackathon/src/app/api/hackathons/[id]/criteria/route.ts` with GET, POST, PATCH for judging criteria.
- [ ] T079 [US3] Create `apps/hackathon/src/app/api/submissions/[id]/scores/route.ts` with GET (view), POST (submit score).
- [ ] T080 [US3] Create `apps/hackathon/src/app/api/hackathons/[id]/leaderboard/route.ts` with GET for rankings.

### UI Components for US3

- [ ] T081 [P] [US3] Create `apps/hackathon/src/components/judging/submission-review-card.tsx` with project details and scoring form.
- [ ] T082 [P] [US3] Create `apps/hackathon/src/components/judging/criteria-list.tsx` with editable criteria for organizers.
- [ ] T083 [US3] Create `apps/hackathon/src/components/judging/score-form.tsx` with slider/input per criterion and feedback textarea.
- [ ] T084 [US3] Create `apps/hackathon/src/components/judging/judge-progress.tsx` with reviewed/total count.
- [ ] T085 [US3] Create `apps/hackathon/src/components/judging/leaderboard.tsx` with ranked submissions table.

### Pages for US3

- [ ] T086 [US3] Create `apps/hackathon/src/app/(dashboard)/hackathons/[id]/judge/page.tsx` with judging dashboard and submission queue.
- [ ] T087 [US3] Create `apps/hackathon/src/app/(dashboard)/hackathons/[id]/leaderboard/page.tsx` with final rankings.

**Checkpoint**: US3 complete - judges can score submissions, see progress, organizers see leaderboard.

---

## Phase 7: User Story 5 - Organizer Manages Roles (Priority: P2)

**Goal**: Organizers can assign and revoke judge, mentor, manager roles for hackathon.

**Independent Test**: As organizer, go to role management, search user, assign "judge" role, verify user can access judging dashboard.

### Database Queries for US5

- [ ] T088 [US5] Create `apps/hackathon/src/db/queries/roles.ts` with getRolesByHackathon, assignRole, revokeRole, getUsersByRole.

### API Routes for US5

- [ ] T089 [US5] Create `apps/hackathon/src/app/api/hackathons/[id]/roles/route.ts` with GET (list), POST (assign), DELETE (revoke).
- [ ] T090 [US5] Create `apps/hackathon/src/app/api/hackathons/[id]/users/route.ts` with GET for searchable user list.

### UI Components for US5

- [ ] T091 [P] [US5] Create `apps/hackathon/src/components/roles/role-badge.tsx` with color-coded role indicator.
- [ ] T092 [US5] Create `apps/hackathon/src/components/roles/role-management.tsx` with user search and role assignment.
- [ ] T093 [US5] Create `apps/hackathon/src/components/roles/user-role-list.tsx` with assigned users per role type.

### Pages for US5

- [ ] T094 [US5] Create `apps/hackathon/src/app/(dashboard)/hackathons/[id]/manage/roles/page.tsx` with role management interface.

**Checkpoint**: US5 complete - organizers can assign/revoke roles, permissions enforced immediately.

---

## Phase 8: User Story 6 - Mentor Provides Guidance (Priority: P3)

**Goal**: Mentors can view assigned teams and send messages/feedback.

**Independent Test**: As mentor, access mentor dashboard, see assigned teams, send message, verify team receives it.

### Database Queries for US6

- [ ] T095 [US6] Create `apps/hackathon/src/db/queries/messages.ts` with getMessagesByTeam, createMessage, getUnreadCount.
- [ ] T096 [US6] Add getMentorTeams to `apps/hackathon/src/db/queries/teams.ts` for mentor-assigned teams.

### API Routes for US6

- [ ] T097 [US6] Create `apps/hackathon/src/app/api/teams/[id]/messages/route.ts` with GET (list), POST (send message).
- [ ] T098 [US6] Create `apps/hackathon/src/app/api/hackathons/[id]/mentor/route.ts` with GET for mentor's assigned teams.

### UI Components for US6

- [ ] T099 [P] [US6] Create `apps/hackathon/src/components/messages/message-list.tsx` with chat-style message display.
- [ ] T100 [P] [US6] Create `apps/hackathon/src/components/messages/message-input.tsx` with text input and send button.
- [ ] T101 [US6] Create `apps/hackathon/src/components/messages/team-chat.tsx` combining list and input.

### Pages for US6

- [ ] T102 [US6] Create `apps/hackathon/src/app/(dashboard)/hackathons/[id]/mentor/page.tsx` with mentor dashboard and team list.
- [ ] T103 [US6] Update `apps/hackathon/src/app/(dashboard)/teams/[id]/page.tsx` to include team chat section.

**Checkpoint**: US6 complete - mentors can message teams, team members see messages in team page.

---

## Phase 9: User Story 7 - Organizer Views Analytics (Priority: P3)

**Goal**: Organizers can view hackathon metrics: registrations, teams, submissions, judging progress.

**Independent Test**: As organizer, go to analytics dashboard, see registration count, team count, submission count, judging progress.

### Database Queries for US7

- [ ] T104 [US7] Create `apps/hackathon/src/db/queries/analytics.ts` with getHackathonStats, getRegistrationTrend, getJudgingProgress.

### API Routes for US7

- [ ] T105 [US7] Create `apps/hackathon/src/app/api/hackathons/[id]/analytics/route.ts` with GET for all metrics.

### UI Components for US7

- [ ] T106 [P] [US7] Create `apps/hackathon/src/components/analytics/stat-card.tsx` with metric value and label.
- [ ] T107 [P] [US7] Create `apps/hackathon/src/components/analytics/progress-bar.tsx` with percentage display.
- [ ] T108 [US7] Create `apps/hackathon/src/components/analytics/analytics-dashboard.tsx` with all metrics grid.

### Pages for US7

- [ ] T109 [US7] Create `apps/hackathon/src/app/(dashboard)/hackathons/[id]/manage/analytics/page.tsx` with analytics dashboard.

**Checkpoint**: US7 complete - organizers see real-time metrics, dashboard refreshes within 30 seconds.

---

## Phase 10: Polish & Cross-Cutting Concerns

**Purpose**: Error handling, loading states, performance, security hardening

- [ ] T110 [P] Create `apps/hackathon/src/components/ui/loading-spinner.tsx` for loading states.
- [ ] T111 [P] Create `apps/hackathon/src/components/ui/error-message.tsx` for error display.
- [ ] T112 [P] Create `apps/hackathon/src/app/error.tsx` with global error boundary.
- [ ] T113 [P] Create `apps/hackathon/src/app/not-found.tsx` with 404 page.
- [ ] T114 Add loading.tsx files to each route group for Suspense boundaries.
- [ ] T115 Review all API routes for proper error handling and status codes.
- [ ] T116 Add rate limiting middleware for API routes using token bucket pattern.
- [ ] T117 Verify multi-tenant isolation: all queries filter by organizationId.
- [ ] T118 Run security audit: verify no cross-org data access (SC-007).
- [ ] T119 Create `apps/hackathon/scripts/seed-dev.ts` with sample data for testing.
- [ ] T120 Use `pnpm build` to verify production build succeeds.
- [ ] T121 Verify all success criteria from spec (SC-001 through SC-007).
- [ ] T122 Run quickstart.md validation: complete developer onboarding flow.

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup) → Phase 2 (Foundational) → BLOCKS ALL USER STORIES
                                         ↓
         ┌─────────────────────────────────────────────────┐
         ↓                    ↓                    ↓        ↓
    Phase 3 (US1)       Phase 4 (US2)       (US3-US7 depend on US1/US2)
         ↓                    ↓
    Phase 5 (US4) ←──── depends on US2 (teams exist)
         ↓
    Phase 6 (US3) ←──── depends on US4 (submissions exist)
         ↓
    Phase 7-9 (US5-US7) ←── can run in parallel after US3
         ↓
    Phase 10 (Polish)
```

### User Story Dependencies

| Story | Depends On | Can Start After |
|-------|------------|-----------------|
| US1 (Hackathon CRUD) | Foundational | Phase 2 complete |
| US2 (Registration/Teams) | Foundational | Phase 2 complete |
| US4 (Submissions) | US2 (teams) | Phase 4 complete |
| US3 (Judging) | US4 (submissions) | Phase 5 complete |
| US5 (Role Management) | US1 | Phase 3 complete |
| US6 (Mentoring) | US2, US5 | Phase 4+7 complete |
| US7 (Analytics) | US1, US2, US4 | Phase 5 complete |

### Parallel Opportunities

**Within Phase 1 (Setup):**
```
T002-T008: All dependency installations (parallel)
T010-T012: Config files (parallel)
```

**Within Phase 2 (Foundational):**
```
T020-T022: Auth lib files (parallel)
T030-T032: Validation schemas (parallel)
T033-T036: Layout components (parallel)
```

**Within User Story Phases:**
```
T041-T042: US1 card components (parallel)
T056-T058: US2 catalog/team components (parallel)
T081-T082: US3 judging components (parallel)
```

---

## Parallel Example: Phase 2 Foundational

```bash
# Launch auth lib files in parallel:
Task T020: "Create oauth-client.ts with Arctic OAuth2Client"
Task T021: "Create session.ts with iron-session"
Task T022: "Create jwt-verify.ts with jose"

# Launch validation schemas in parallel:
Task T030: "Create hackathon.ts validation schemas"
Task T031: "Create team.ts validation schemas"
Task T032: "Create submission.ts validation schemas"
```

---

## Implementation Strategy

### MVP First (User Stories 1 + 2)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Hackathon CRUD)
4. **STOP and VALIDATE**: Test US1 independently
5. Complete Phase 4: User Story 2 (Registration/Teams)
6. **STOP and VALIDATE**: Test US2 independently
7. Deploy MVP with hackathon creation + participant registration + team formation

### Incremental Delivery

1. **MVP**: Setup + Foundational + US1 + US2 → Core platform works
2. **+Submissions**: US4 → Teams can submit projects
3. **+Judging**: US3 → Judges can score
4. **+Roles**: US5 → Full role management
5. **+Mentoring**: US6 → Mentor features
6. **+Analytics**: US7 → Dashboard metrics

### Suggested MVP Scope

**Minimum Viable Product (User Stories 1 + 2 only)**:
- Organizers can create/manage hackathons
- Participants can register and form teams
- Core value delivered: hackathon setup and team formation

**Estimated tasks for MVP**: T001-T066 (~66 tasks)

---

## Summary

| Phase | Story | Task Count | Parallel Tasks |
|-------|-------|------------|----------------|
| 1 | Setup | 14 | 10 |
| 2 | Foundational | 22 | 12 |
| 3 | US1 (Hackathon CRUD) | 12 | 4 |
| 4 | US2 (Registration/Teams) | 18 | 6 |
| 5 | US4 (Submissions) | 9 | 2 |
| 6 | US3 (Judging) | 12 | 4 |
| 7 | US5 (Roles) | 7 | 2 |
| 8 | US6 (Mentoring) | 9 | 4 |
| 9 | US7 (Analytics) | 6 | 4 |
| 10 | Polish | 13 | 6 |
| **Total** | | **122** | **54** |

**Format Validation**: ✅ All 122 tasks follow checklist format with checkbox, ID, labels, and file paths.
