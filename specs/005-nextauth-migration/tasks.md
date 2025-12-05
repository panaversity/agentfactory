# Tasks: NextAuth to Better Auth User Migration

**Input**: Design documents from `/specs/005-nextauth-migration/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: No automated tests requested. Manual verification via --dry-run flag.

**Organization**: Tasks build the single migration script incrementally, with each user story adding specific functionality.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single script**: `scripts/migrate-nextauth-users.ts`
- All functionality in one file for simplicity

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and script scaffold

- [ ] T001 Create script file with imports and types in scripts/migrate-nextauth-users.ts
- [ ] T002 Add TypeScript interfaces for OldUser, OldProfile, and migration results
- [ ] T003 Implement CLI argument parser (--dry-run, --limit, --offset, --batch-size, --verbose flags)
- [ ] T004 Implement config loader with env var validation (NEXT_AUTH_PROD_DB_MIRROR, DATABASE_URL)
- [ ] T005 Implement database connection initializer for Neon serverless (source and target)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

- [ ] T006 Implement logger utility with progress percentage and ETA calculation
- [ ] T007 Implement source database query: users LEFT JOIN profile with LIMIT/OFFSET
- [ ] T008 Implement target database email lookup for conflict detection
- [ ] T009 Implement transaction wrapper (begin, commit, rollback)
- [ ] T010 Add banner output and configuration summary printer

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Dry Run Migration (Priority: P1)

**Goal**: Enable safe verification of migration logic without modifying any data

**Independent Test**: Run `--dry-run` and verify log output shows intended operations without DB changes

### Implementation for User Story 1

- [ ] T011 [US1] Add dryRun flag handling throughout script
- [ ] T012 [US1] Implement dry-run logging for INSERT operations (would create user X)
- [ ] T013 [US1] Implement dry-run logging for MERGE operations (would update user.id for X)
- [ ] T014 [US1] Add dry-run summary output (total to-migrate, to-merge, country normalizations)
- [ ] T015 [US1] Ensure no database writes when --dry-run flag is set

**Checkpoint**: Dry-run mode fully functional - can verify migration logic safely

---

## Phase 4: User Story 2 - Batch Migration with Progress (Priority: P1)

**Goal**: Process users in batches of 500 with transaction safety and progress tracking

**Independent Test**: Run `--limit 500` and verify single batch processes with progress output

### Implementation for User Story 2

- [ ] T016 [US2] Implement batch fetching loop with configurable batch size (default 500)
- [ ] T017 [US2] Implement transaction-per-batch pattern (begin before batch, commit after)
- [ ] T018 [US2] Implement batch rollback on any error within batch
- [ ] T019 [US2] Add progress tracking: percentage complete and ETA after each batch
- [ ] T020 [US2] Implement batch statistics logging (inserted, merged, errors per batch)
- [ ] T021 [US2] Add timing measurement for ETA calculation

**Checkpoint**: Batch processing works with transaction safety and progress tracking

---

## Phase 5: User Story 5 - Conflict Resolution for Overlapping Users (Priority: P1)

**Goal**: Update target user.id to match NextAuth user.id, preserving FK references and filling NULL fields

**Independent Test**: Migrate overlapping user, verify user.id changed and all FK references updated

### Implementation for User Story 5

- [ ] T022 [US5] Implement mergeOverlappingUser function skeleton
- [ ] T023 [US5] Implement FK reference update for account.user_id
- [ ] T024 [US5] Implement FK reference update for session.user_id
- [ ] T025 [US5] Implement FK reference update for member.user_id
- [ ] T026 [US5] Implement FK reference update for oauth_access_token.user_id
- [ ] T027 [US5] Implement FK reference update for oauth_consent.user_id
- [ ] T028 [US5] Implement FK reference update for apikey.user_id
- [ ] T029 [US5] Implement FK reference update for invitation.inviter_id
- [ ] T030 [US5] Implement FK reference update for oauth_application.user_id
- [ ] T031 [US5] Implement user.id update after all FK references updated
- [ ] T032 [US5] Implement fillNullFields function (only update NULL fields in target from source)
- [ ] T033 [US5] Implement credential account upsert with bcrypt password hash
- [ ] T034 [US5] Add merge logging with old/new ID and fields filled

**Checkpoint**: Conflict resolution fully functional - overlapping users get ID updated with FK integrity

---

## Phase 6: User Story 3 - Country Normalization (Priority: P2)

**Goal**: Normalize all country values to full country names

**Independent Test**: Verify "PK" becomes "Pakistan" and NULL becomes "Pakistan"

### Implementation for User Story 3

- [ ] T035 [P] [US3] Create COUNTRY_MAP constant with all 18 country code mappings
- [ ] T036 [US3] Implement normalizeCountry function with NULL default to "Pakistan"
- [ ] T037 [US3] Add unknown country code warning logging
- [ ] T038 [US3] Integrate country normalization into user insert and merge operations
- [ ] T039 [US3] Add country normalization statistics to summary output

**Checkpoint**: Country normalization working - all country values transformed correctly

---

## Phase 7: User Story 4 - Username Recreation (Priority: P2)

**Goal**: Generate unique usernames from email addresses

**Independent Test**: Verify username format "prefix-xxxx" generated from email

### Implementation for User Story 4

- [ ] T040 [P] [US4] Implement extractEmailPrefix function (lowercase, special chars to dash)
- [ ] T041 [US4] Implement generateRandomSuffix function (4 char alphanumeric)
- [ ] T042 [US4] Implement username collision detection with Set tracking
- [ ] T043 [US4] Implement collision resolution with retry and timestamp fallback
- [ ] T044 [US4] Integrate username generation into user insert operations
- [ ] T045 [US4] Add username transformation logging (old → new)

**Checkpoint**: Username generation working - all users get unique usernames

---

## Phase 8: User Story 6 - Resume Capability (Priority: P3)

**Goal**: Enable resuming migration from a specific offset

**Independent Test**: Run `--offset 5000` and verify processing starts from position 5001

### Implementation for User Story 6

- [ ] T046 [US6] Implement --offset flag handling in main loop
- [ ] T047 [US6] Adjust progress calculation to account for offset
- [ ] T048 [US6] Add offset information to configuration summary
- [ ] T049 [US6] Ensure --offset and --limit work together correctly

**Checkpoint**: Resume capability working - can continue interrupted migration

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Final integration, error handling, and documentation

- [ ] T050 Implement new user INSERT function (user table + account table)
- [ ] T051 Add password hash validation (verify $2a$, $2b$, $2y$ prefix)
- [ ] T052 Implement name resolution (use email prefix if name is NULL)
- [ ] T053 Add comprehensive error handling with exit codes (0=success, 1=config, 2=batch)
- [ ] T054 Implement final summary reporter with all statistics
- [ ] T055 Add verification query suggestions to output
- [ ] T056 Final code cleanup and comments
- [ ] T057 Test full migration flow with --dry-run
- [ ] T058 Test limited migration with --limit 100
- [ ] T059 Update docs/migration/CONTINUE-MIGRATION.prompt.md with completion status

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-8)**: All depend on Foundational phase completion
  - P1 stories (US1, US2, US5) should be done first
  - P2 stories (US3, US4) build on P1 foundation
  - P3 story (US6) is enhancement
- **Polish (Phase 9)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (Dry Run)**: Can start after Foundational - No dependencies
- **User Story 2 (Batch)**: Can start after Foundational - No dependencies
- **User Story 5 (Conflict)**: Can start after Foundational - No dependencies (but complex)
- **User Story 3 (Country)**: Can start after US1/US2 - Integrates with insert/merge
- **User Story 4 (Username)**: Can start after US1/US2 - Integrates with insert
- **User Story 6 (Resume)**: Can start after US2 - Extends batch processing

### Within Each User Story

- Core function implementation first
- Integration with main flow second
- Logging and statistics third
- Verification last

### Parallel Opportunities

Since this is a single file, limited parallelism. However:
- T035 and T040 are independent helper functions [P]
- Different user stories can be worked on by different developers if they coordinate on the single file

---

## Parallel Example: Foundation Phase

```bash
# These foundation tasks can be developed in parallel (separate functions):
T006: Logger utility
T007: Source query function
T008: Target lookup function
T009: Transaction wrapper
T010: Banner printer
```

---

## Implementation Strategy

### MVP First (P1 Stories Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational
3. Complete Phase 3: User Story 1 (Dry Run)
4. Complete Phase 4: User Story 2 (Batch)
5. Complete Phase 5: User Story 5 (Conflict Resolution)
6. **STOP and VALIDATE**: Test with --dry-run on full dataset
7. Deploy for production migration

### Full Implementation

1. MVP (P1 stories) → Verify with dry-run
2. Add User Story 3 (Country) → Verify normalizations
3. Add User Story 4 (Username) → Verify username format
4. Add User Story 6 (Resume) → Test resume capability
5. Polish phase → Final cleanup
6. Production migration

---

## Task Summary

| Phase | Tasks | Story Coverage |
|-------|-------|----------------|
| Setup | T001-T005 (5) | Infrastructure |
| Foundational | T006-T010 (5) | Core utilities |
| US1 Dry Run | T011-T015 (5) | P1 |
| US2 Batch | T016-T021 (6) | P1 |
| US5 Conflict | T022-T034 (13) | P1 |
| US3 Country | T035-T039 (5) | P2 |
| US4 Username | T040-T045 (6) | P2 |
| US6 Resume | T046-T049 (4) | P3 |
| Polish | T050-T059 (10) | Cross-cutting |

**Total Tasks**: 59

**MVP Scope**: T001-T034 (34 tasks) - Covers Setup + Foundational + P1 stories

---

## Notes

- Single file implementation for simplicity
- All database operations use raw SQL via Neon serverless
- Transaction per batch for safety
- Dry-run mode is critical for verification
- Password hashes copied EXACTLY without transformation
- FK reference updates must happen BEFORE user.id update
- Only fill NULL fields for overlapping users (preserve existing data)
