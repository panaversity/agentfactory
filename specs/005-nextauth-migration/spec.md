# Feature Specification: NextAuth to Better Auth User Migration

**Feature Branch**: `005-nextauth-migration`
**Created**: 2024-12-05
**Status**: Ready for Planning
**Input**: User description: "NextAuth to Better Auth user migration script with batch processing, conflict resolution, and comprehensive edge case handling"

## Overview

Migrate approximately 14,821 users from the legacy NextAuth production database to the new Better Auth SSO system. This migration preserves user IDs (critical for microservice FK references), handles password hash compatibility (bcrypt → scrypt progressive migration), and ensures data integrity through batch processing with transaction safety.

### Migration Statistics

| Metric | Count |
|--------|-------|
| Source database users | 14,821 |
| Target database users | 4 (test users) |
| Users with profile data | 7,250 |
| Overlapping emails | 3 |
| Admin users | 1 |

## Success Evals *(mandatory - defined BEFORE specification)*

These evaluation criteria define what success looks like BEFORE we specify how to achieve it.

### Eval 1: Data Integrity (P0 - Blocking)
- **Metric**: 100% of source users with passwords are accounted for in target (migrated or explicitly skipped)
- **Target**: Zero unexplained missing users
- **Measurement**: Compare source count vs (target migrated + target skipped)

### Eval 2: Login Functionality (P0 - Blocking)
- **Metric**: 95%+ migrated users can successfully login with original password on first attempt
- **Target**: <5% failure rate (accounting for edge cases like NULL passwords)
- **Measurement**: Automated login test against sample of 100 random migrated users

### Eval 3: Performance (P1 - Critical)
- **Metric**: Migration completion time for 14,821 users
- **Target**: Under 30 minutes wall-clock time
- **Measurement**: Script execution time from start to finish

### Eval 4: Data Transformation Accuracy (P1 - Critical)
- **Metric**: Country normalization correctness
- **Target**: 100% of "PK" → "Pakistan", 100% of NULL → "Pakistan", 0% unknown codes preserved without warning
- **Measurement**: Post-migration SQL query on country distribution

### Eval 5: Dry-Run Fidelity (P2 - Important)
- **Metric**: Dry-run output matches actual migration counts
- **Target**: <1% variance between dry-run predictions and actual results
- **Measurement**: Compare dry-run summary vs post-migration summary

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Dry Run Migration (Priority: P1)

As a database administrator, I want to run a complete dry-run of the migration without modifying any data, so that I can verify the migration logic and identify potential issues before committing changes.

**Why this priority**: A dry-run is essential for risk mitigation. It allows validation of all edge cases, query logic, and data transformations without risking production data.

**Independent Test**: Can be fully tested by running `--dry-run` flag and verifying log output matches expected migration counts and transformations.

**Acceptance Scenarios**:

1. **Given** source DB with 14,821 users and target DB with 4 users, **When** running with `--dry-run` flag, **Then** script logs all intended operations without modifying either database
2. **Given** dry-run mode, **When** encountering an overlapping email, **Then** script logs "SKIP: User exists" with user details
3. **Given** dry-run mode, **When** processing completes, **Then** script outputs summary with: total processed, to-migrate, to-skip, country normalizations applied

---

### User Story 2 - Batch Migration with Progress (Priority: P1)

As a database administrator, I want to migrate users in batches of 500 with progress tracking, so that I can monitor long-running migration and have transaction safety per batch.

**Why this priority**: Batch processing enables transaction rollback on failure, progress monitoring, and resumability for a 14K+ user migration.

**Independent Test**: Can be tested with `--limit 500` to process single batch and verify transaction commits correctly.

**Acceptance Scenarios**:

1. **Given** 14,821 users to migrate, **When** migration runs, **Then** users are processed in batches of 500 with progress percentage and ETA displayed after each batch
2. **Given** a batch of 500 users, **When** an error occurs mid-batch, **Then** entire batch is rolled back and error is logged with batch number
3. **Given** batch 5 of 30 completes, **When** checking progress, **Then** log shows "Batch 5/30 complete | Progress: 16.7% | ETA: X minutes"

---

### User Story 3 - Country Normalization (Priority: P2)

As a data steward, I want all country values normalized to full country names, so that the target database has consistent country data.

**Why this priority**: Data consistency is important but not blocking. Migration can proceed with normalization as a transformation step.

**Independent Test**: Can be tested by querying source for country distribution and verifying target has normalized values.

**Acceptance Scenarios**:

1. **Given** a user with country="PK", **When** migrated, **Then** target user has country="Pakistan"
2. **Given** a user with country=NULL, **When** migrated, **Then** target user has country="Pakistan" (default)
3. **Given** a user with country="Pakistan", **When** migrated, **Then** target user has country="Pakistan" (unchanged)
4. **Given** a user with country="US", **When** migrated, **Then** target user has country="United States"

---

### User Story 4 - Username Recreation (Priority: P2)

As a system administrator, I want usernames to be regenerated from email addresses, so that we have a consistent username format in the new system.

**Why this priority**: Username format standardization is valuable but existing usernames could be preserved as fallback.

**Independent Test**: Can be tested by comparing generated usernames against expected format.

**Acceptance Scenarios**:

1. **Given** a user with email="john.doe@gmail.com", **When** migrated, **Then** username is generated as "john-doe-{random4}" format
2. **Given** generated username conflicts with existing, **When** conflict detected, **Then** append incremental suffix until unique
3. **Given** migration log, **When** username regenerated, **Then** log shows "Username: old='p-16-muhammad' → new='muhammad-junaid-a1b2'"

---

### User Story 5 - Conflict Resolution for Overlapping Users (Priority: P1)

As a database administrator, I want overlapping users to have their Better Auth user.id updated to match the NextAuth user.id, so that microservice FK references remain valid and students experience zero friction.

**Why this priority**: Critical for seamless transition - students must login with same credentials and all FK references across microservices must work. The NextAuth user.id is the source of truth used across all systems.

**Independent Test**: Can be tested by migrating overlapping user, verifying user.id changed to NextAuth ID, and all FK references updated.

**Acceptance Scenarios**:

1. **Given** user email exists in target DB with different user.id, **When** processing that user, **Then** update target user.id to match NextAuth user.id
2. **Given** overlapping user with Better Auth id="abc123" and NextAuth id="xyz789", **When** migrated, **Then** target user.id becomes "xyz789" and all FK references updated
3. **Given** overlapping user, **When** updating user.id, **Then** update ALL FK references: account.user_id, session.user_id, member.user_id, oauth_access_token.user_id, oauth_consent.user_id
4. **Given** overlapping user with existing non-null city="Lahore" in target and city="Karachi" in source, **When** migrated, **Then** city remains "Lahore" (only nullable fields updated)
5. **Given** overlapping user with NULL city in target and city="Karachi" in source, **When** migrated, **Then** city becomes "Karachi" (fill in missing data)
6. **Given** 3 overlapping users, **When** migration completes, **Then** summary shows "Merged (id updated): 3"

---

### User Story 6 - Resume Capability (Priority: P3)

As a database administrator, I want to resume migration from a specific offset, so that I can continue after interruption without re-processing.

**Why this priority**: Resume capability is nice-to-have for reliability but not critical for single-run migration.

**Independent Test**: Can be tested by running with `--offset 5000` and verifying processing starts from user 5001.

**Acceptance Scenarios**:

1. **Given** `--offset 5000` flag, **When** migration starts, **Then** first user processed is at position 5001
2. **Given** migration interrupted at batch 10, **When** resumed with `--offset 5000`, **Then** continues from correct position
3. **Given** `--offset` and `--limit` combined, **When** running `--offset 1000 --limit 500`, **Then** processes users 1001-1500 only

---

### Edge Cases

- **EC-001**: What happens when user has NULL name? → Use email prefix as name (e.g., "john" from "john@example.com")
- **EC-002**: What happens when password is empty string vs NULL? → Skip both (no credential to migrate)
- **EC-003**: What happens when profile record doesn't exist for user? → Use NULL for city/gender/father_name
- **EC-004**: What happens when database connection drops mid-batch? → Rollback current batch, log error, exit with non-zero code
- **EC-005**: What happens when UUID format is invalid? → Log error, skip user, continue with next
- **EC-006**: What happens when country code is unknown (not in mapping)? → Keep original value, log warning
- **EC-007**: What happens when email has unusual format? → Preserve as-is (already validated in source)
- **EC-008**: What happens when role is not 'user' or 'admin'? → Preserve original role value
- **EC-009**: What happens when username generation creates very long string? → Truncate to 50 chars, ensure uniqueness
- **EC-010**: What happens when created_at/updated_at are NULL? → Use current timestamp
- **EC-011**: What happens when overlapping user has FK references in target? → Update ALL FK references (account, session, member, oauth_access_token, oauth_consent) in single transaction
- **EC-012**: What happens when overlapping user has non-null field in target but different value in source? → Keep target value (don't overwrite existing data)
- **EC-013**: What happens when overlapping user has null field in target but value in source? → Fill with source value (enrich data)
- **EC-014**: What happens when overlapping user already has credential account? → Update password hash to source bcrypt hash (ensure login works)
- **EC-015**: What happens when updating user.id causes FK constraint violation? → Disable FK checks, update all references, re-enable FK checks (single transaction)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST process users in batches of 500 with transaction isolation per batch
- **FR-002**: System MUST preserve user.id exactly as-is (UUID text format) for FK integrity
- **FR-003**: System MUST create account record with provider_id='credential' containing bcrypt password hash
- **FR-004**: System MUST normalize country codes to full names (PK→Pakistan, US→United States, etc.)
- **FR-005**: System MUST default NULL country values to "Pakistan"
- **FR-006**: System MUST regenerate usernames from email addresses with uniqueness guarantee
- **FR-007**: System MUST handle overlapping users (same email) by updating target user.id to match source NextAuth user.id and updating all FK references
- **FR-008**: System MUST JOIN profile table to include city, gender, father_name fields
- **FR-009**: System MUST support `--dry-run` flag that logs all operations without database writes
- **FR-010**: System MUST support `--limit N` flag to process only N users
- **FR-011**: System MUST support `--offset N` flag to start from position N
- **FR-012**: System MUST display progress percentage and ETA after each batch
- **FR-013**: System MUST log comprehensive details: user email, old/new username, country transformation, batch stats
- **FR-014**: System MUST rollback entire batch on any error within that batch
- **FR-015**: System MUST preserve created_at and updated_at timestamps from source
- **FR-016**: System MUST preserve email_verified status from source (maps to email_verified boolean)
- **FR-017**: System MUST preserve role field (admin/user) from source
- **FR-018**: System MUST generate new UUID for account.id (not user.id which is preserved)
- **FR-019**: System MUST update ALL FK references when changing user.id for overlapping users (account, session, member, oauth_access_token, oauth_consent, apikey, invitation, oauth_application)
- **FR-020**: System MUST only update nullable fields in target when target value is NULL (preserve existing non-null data)
- **FR-021**: System MUST update/create credential account for overlapping users with source bcrypt password hash
- **FR-022**: System MUST copy bcrypt password hash EXACTLY as-is without any transformation or re-hashing
- **FR-023**: System MUST verify password hash format starts with $2a$, $2b$, or $2y$ before migration

### Non-Functional Requirements

- **NFR-001**: Script MUST complete full migration within 30 minutes (500 users/batch, ~30 batches)
- **NFR-002**: Script MUST use connection pooling for database efficiency
- **NFR-003**: Script MUST handle Neon serverless database connection characteristics
- **NFR-004**: Script MUST exit with code 0 on success, non-zero on failure

### Key Entities

- **Source User (NextAuth)**: id (UUID), email, name, password (bcrypt), emailVerified, image, role, username, phone_number, country, created_at, updated_at
- **Source Profile (NextAuth)**: user_id, city, gender, father_name
- **Target User (Better Auth)**: id (text), email, name, email_verified, image, role, username, phone_number, country, city, gender, father_name, created_at, updated_at
- **Target Account (Better Auth)**: id (text), account_id, provider_id='credential', user_id, password (bcrypt hash preserved)

### Data Transformation Rules

| Source Field | Target Field | Transformation |
|-------------|--------------|----------------|
| users.id | user.id | Preserve exactly (UUID as text) |
| users.email | user.email | No change |
| users.name | user.name | If NULL, use email prefix |
| users.password | account.password | Preserve bcrypt hash |
| users.emailVerified | user.email_verified | Boolean mapping |
| users.image | user.image | No change |
| users.role | user.role | No change |
| users.username | (discarded) | Regenerate from email |
| users.phone_number | user.phone_number | No change |
| users.country | user.country | Normalize (PK→Pakistan, NULL→Pakistan) |
| profile.city | user.city | No change |
| profile.gender | user.gender | No change |
| profile.father_name | user.father_name | No change |

### Country Normalization Mapping

```
PK → Pakistan
US → United States
GB → United Kingdom
CA → Canada
AE → United Arab Emirates
SA → Saudi Arabia
IN → India
AU → Australia
DE → Germany
QA → Qatar
MY → Malaysia
AF → Afghanistan
CN → China
IE → Ireland
EG → Egypt
TR → Turkey
OM → Oman
IT → Italy
NULL → Pakistan (default)
Pakistan → Pakistan (no change)
[other] → Keep original (log warning)
```

## Constraints

- **C-001**: Source database is READ-ONLY (mirror) - no modifications allowed
- **C-002**: Target database must maintain referential integrity - user must exist before account
- **C-003**: Email uniqueness constraint in target - duplicate emails will fail
- **C-004**: Username uniqueness constraint in target - must guarantee unique usernames
- **C-005**: Must use environment variables for database URLs (never hardcode)

## Non-Goals

- **NG-001**: NOT migrating OAuth provider accounts (Google, GitHub) - only credential accounts
- **NG-002**: NOT migrating sessions - users will need to re-login after migration
- **NG-003**: NOT deleting any existing data in target - only update user.id and fill NULL fields
- **NG-004**: NOT converting bcrypt to scrypt during migration - progressive migration on password change
- **NG-005**: NOT migrating organization memberships - handled separately if needed

## Assumptions

- **A-001**: All source users have valid email addresses (already validated)
- **A-002**: All source passwords are bcrypt hashes (verified in smoke test)
- **A-003**: Source database connection string provided via NEXT_AUTH_PROD_DB_MIRROR env var
- **A-004**: Target database connection string provided via DATABASE_URL env var
- **A-005**: Better Auth password verification already supports bcrypt (implemented in auth.ts:189-200)
- **A-006**: 4 existing users in target are test users that may be overwritten or skipped
- **A-007**: Profile data join is LEFT JOIN - users without profile get NULL values

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of source users with passwords are processed (migrated or skipped with reason)
- **SC-002**: All migrated users can login with their original password within 5 seconds
- **SC-003**: Migration completes in under 30 minutes for full 14,821 user dataset
- **SC-004**: Zero data loss - all source fields mapped to target fields
- **SC-005**: Dry-run matches actual migration (same counts, same transformations)
- **SC-006**: Post-migration verification query shows expected user count (~14,825)

### Verification Queries

```sql
-- Count verification
SELECT COUNT(*) FROM public."user";  -- Expected: ~14,825

-- Credential account verification
SELECT COUNT(*) FROM public.account WHERE provider_id = 'credential';  -- Expected: ~14,821

-- Country normalization verification
SELECT country, COUNT(*) FROM public."user" GROUP BY country ORDER BY COUNT(*) DESC;
-- Expected: "Pakistan" as top country (~8,400)

-- Sample login test
SELECT u.id, u.email, a.password
FROM public."user" u
JOIN public.account a ON u.id = a.user_id
WHERE u.email = 'test@example.com' AND a.provider_id = 'credential';
```

## Dependencies

- **DEP-001**: `@neondatabase/serverless` - Neon database driver
- **DEP-002**: `drizzle-orm` - ORM for type-safe queries (optional, can use raw SQL)
- **DEP-003**: `dotenv` - Environment variable loading
- **DEP-004**: Source database accessible and populated
- **DEP-005**: Target database accessible with schema deployed

## Environment Variables

```bash
# Source database (read-only NextAuth mirror)
NEXT_AUTH_PROD_DB_MIRROR=postgresql://...

# Target database (Better Auth SSO)
DATABASE_URL=postgresql://...
```

## Related Documents

- `docs/migration/nextauth-to-betterauth-guide.md` - Migration guide
- `docs/migration/CONTINUE-MIGRATION.prompt.md` - Continuation prompt
- `history/prompts/004-nextauth-migration/001-smoke-test.prompt.md` - Smoke test results
- `src/lib/auth.ts:189-200` - bcrypt password verification
- `scripts/test-password-migration.ts` - Test migration script (reference)
