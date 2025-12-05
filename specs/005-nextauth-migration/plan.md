# Implementation Plan: NextAuth to Better Auth User Migration

**Branch**: `005-nextauth-migration` | **Date**: 2024-12-05 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/005-nextauth-migration/spec.md`

## Summary

Migrate 14,821 users from NextAuth PostgreSQL database to Better Auth SSO system with:
- Batch processing (500 users/batch) with transaction safety
- ID preservation for microservice FK integrity
- Conflict resolution: update target user.id to match source, preserve non-null data
- Country normalization and username regeneration
- Bcrypt password hash preservation (no re-hashing)

## Technical Context

**Language/Version**: TypeScript 5.x (tsx runtime)
**Primary Dependencies**: @neondatabase/serverless, drizzle-orm, dotenv
**Storage**: PostgreSQL (Neon serverless) - Source: read-only mirror, Target: Better Auth DB
**Testing**: Manual verification with --dry-run, --limit flags
**Target Platform**: Node.js CLI script
**Project Type**: single CLI script
**Performance Goals**: Complete 14,821 users in under 30 minutes (~500 users/minute)
**Constraints**: Source DB read-only, preserve referential integrity, no data loss
**Scale/Scope**: 14,821 users, 7,250 profiles, 3 overlapping users

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| Defense in Depth | PASS | Transaction isolation per batch, rollback on failure |
| Least Privilege | PASS | Source DB read-only, target operations scoped |
| Fail Secure | PASS | Batch rollback on error, exit non-zero on failure |
| No Secrets in Client Code | PASS | Database URLs from env vars only |
| Standards Compliance | PASS | Better Auth schema, bcrypt hash compatibility |
| Secure Defaults | PASS | Dry-run mode default recommended |
| Audit Everything | PASS | Comprehensive logging with progress tracking |
| Token Hygiene | N/A | Migration doesn't handle tokens |

**Gate Status**: PASS - All applicable principles satisfied.

## Project Structure

### Documentation (this feature)

```text
specs/005-nextauth-migration/
├── spec.md              # Feature specification
├── plan.md              # This file
├── checklists/
│   └── requirements.md  # Quality checklist
└── tasks.md             # Task breakdown (via /sp.tasks)
```

### Source Code (repository root)

```text
scripts/
└── migrate-nextauth-users.ts   # Main migration script

# Supporting files (existing)
src/lib/auth.ts                 # bcrypt verification (lines 189-200)
scripts/test-password-migration.ts  # Reference implementation
```

**Structure Decision**: Single CLI script in `scripts/` directory. No new directories needed. Script is self-contained with inline schema definitions to avoid import complexity.

## Architecture

### Component Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    migrate-nextauth-users.ts                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐          │
│  │   CLI       │    │   Config    │    │   Logger    │          │
│  │   Parser    │───▶│   Loader    │───▶│   Setup     │          │
│  └─────────────┘    └─────────────┘    └─────────────┘          │
│         │                                     │                   │
│         ▼                                     ▼                   │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐          │
│  │   Source    │    │   Target    │    │   Progress  │          │
│  │   DB Conn   │    │   DB Conn   │    │   Tracker   │          │
│  └─────────────┘    └─────────────┘    └─────────────┘          │
│         │                 │                   │                   │
│         ▼                 ▼                   ▼                   │
│  ┌─────────────────────────────────────────────────┐            │
│  │                  Batch Processor                 │            │
│  ├─────────────────────────────────────────────────┤            │
│  │  For each batch of 500 users:                   │            │
│  │  1. Fetch from source (users LEFT JOIN profile) │            │
│  │  2. Transform data (country, username, name)    │            │
│  │  3. Check for conflicts (email exists?)         │            │
│  │  4. Execute migration:                          │            │
│  │     - New user: INSERT user + INSERT account    │            │
│  │     - Conflict: UPDATE user.id + FKs + account  │            │
│  │  5. Commit transaction or rollback              │            │
│  └─────────────────────────────────────────────────┘            │
│                           │                                       │
│                           ▼                                       │
│  ┌─────────────────────────────────────────────────┐            │
│  │                Summary Reporter                  │            │
│  │  - Total processed, migrated, merged, skipped   │            │
│  │  - Country normalization stats                  │            │
│  │  - Errors encountered                           │            │
│  └─────────────────────────────────────────────────┘            │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
```

### Data Flow

```
Source DB (NextAuth Mirror)              Target DB (Better Auth)
┌────────────────────────┐              ┌────────────────────────┐
│ public.users           │              │ public."user"          │
│ ├── id (UUID)         ─┼─ PRESERVE ──▶│ ├── id (text)          │
│ ├── email             ─┼─ COPY ──────▶│ ├── email              │
│ ├── name              ─┼─ TRANSFORM ─▶│ ├── name (or email pfx)│
│ ├── password          ─┼──┐           │ ├── username (new)     │
│ ├── emailVerified     ─┼──┼─ COPY ───▶│ ├── email_verified     │
│ ├── country           ─┼──┼─NORMALIZE▶│ ├── country            │
│ └── ...               ─┼──┼─ COPY ───▶│ └── ...                │
├────────────────────────┤  │           ├────────────────────────┤
│ public.profile         │  │           │ public.account         │
│ ├── city              ─┼──┼─ COPY ───▶│ ├── id (new UUID)      │
│ ├── gender            ─┼──┼─ COPY ───▶│ ├── user_id (= user.id)│
│ └── father_name       ─┼──┼─ COPY ───▶│ ├── provider_id='cred' │
└────────────────────────┘  └─ EXACT ──▶│ └── password (bcrypt)  │
                                         └────────────────────────┘
```

### Conflict Resolution Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                     For Each Overlapping User                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  Source User (NextAuth)         Target User (Better Auth)        │
│  ┌──────────────────┐          ┌──────────────────┐             │
│  │ id: "xyz-789"    │          │ id: "abc-123"    │ ◀─ OLD ID   │
│  │ email: same      │          │ email: same      │             │
│  │ city: "Karachi"  │          │ city: NULL       │             │
│  └──────────────────┘          └──────────────────┘             │
│           │                              │                        │
│           │      MERGE OPERATION         │                        │
│           ▼                              ▼                        │
│  ┌─────────────────────────────────────────────────┐            │
│  │ 1. Disable FK constraints (transaction)          │            │
│  │ 2. Update user.id: "abc-123" → "xyz-789"        │            │
│  │ 3. Update ALL FK references:                     │            │
│  │    - account.user_id                             │            │
│  │    - session.user_id                             │            │
│  │    - member.user_id                              │            │
│  │    - oauth_access_token.user_id                  │            │
│  │    - oauth_consent.user_id                       │            │
│  │    - apikey.user_id                              │            │
│  │    - invitation.inviter_id                       │            │
│  │    - oauth_application.user_id                   │            │
│  │ 4. Fill NULL fields from source:                 │            │
│  │    - city: NULL → "Karachi"                      │            │
│  │ 5. Update/create credential account with bcrypt  │            │
│  │ 6. Re-enable FK constraints                      │            │
│  │ 7. Commit transaction                            │            │
│  └─────────────────────────────────────────────────┘            │
│                              │                                    │
│                              ▼                                    │
│  ┌──────────────────┐                                            │
│  │ id: "xyz-789"    │ ◀─ NEW ID (matches source)                │
│  │ email: same      │                                            │
│  │ city: "Karachi"  │ ◀─ Filled from source                     │
│  └──────────────────┘                                            │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
```

## Implementation Sequence

### Phase 1: Core Infrastructure (Foundation)

| Order | Component | Description | Dependencies |
|-------|-----------|-------------|--------------|
| 1.1 | CLI Parser | Parse --dry-run, --limit, --offset flags | None |
| 1.2 | Config Loader | Load env vars, validate DB URLs | CLI Parser |
| 1.3 | Logger | Progress tracking, ETA calculation | None |
| 1.4 | DB Connections | Initialize source (read) and target (write) | Config |

### Phase 2: Data Transformation (Helpers)

| Order | Component | Description | Dependencies |
|-------|-----------|-------------|--------------|
| 2.1 | Country Normalizer | Map country codes to full names | None |
| 2.2 | Username Generator | Create unique username from email | None |
| 2.3 | Name Resolver | Extract name from email if NULL | None |
| 2.4 | Hash Validator | Verify bcrypt hash format | None |

### Phase 3: Migration Logic (Core)

| Order | Component | Description | Dependencies |
|-------|-----------|-------------|--------------|
| 3.1 | Source Query | Fetch users with profiles in batches | Phase 1 |
| 3.2 | Conflict Detector | Check if email exists in target | Phase 1 |
| 3.3 | New User Inserter | INSERT user + account for new users | Phase 2 |
| 3.4 | Conflict Resolver | UPDATE id + FKs for overlapping users | Phase 2 |
| 3.5 | Batch Processor | Orchestrate batch with transaction | Phase 3.1-3.4 |

### Phase 4: Execution (Main)

| Order | Component | Description | Dependencies |
|-------|-----------|-------------|--------------|
| 4.1 | Main Loop | Process all batches with progress | Phase 3.5 |
| 4.2 | Summary Reporter | Output final statistics | Phase 4.1 |
| 4.3 | Exit Handler | Return appropriate exit code | Phase 4.2 |

## Key Algorithms

### Batch Processing Algorithm

```
function processBatches(offset, limit):
    totalUsers = countSourceUsers()
    processedCount = 0
    migratedCount = 0
    mergedCount = 0
    skippedCount = 0

    while processedCount < min(totalUsers - offset, limit):
        batchStart = offset + processedCount
        batch = fetchSourceBatch(batchStart, BATCH_SIZE)

        beginTransaction()
        try:
            for user in batch:
                if userExistsInTarget(user.email):
                    if dryRun:
                        log("MERGE: Would update user.id for {email}")
                    else:
                        mergeOverlappingUser(user)
                    mergedCount++
                else:
                    if dryRun:
                        log("INSERT: Would create user {email}")
                    else:
                        insertNewUser(user)
                    migratedCount++
                processedCount++

            commitTransaction()
            logProgress(processedCount, totalUsers)
        catch error:
            rollbackTransaction()
            log("ERROR: Batch failed at {batchStart}: {error}")
            exit(1)

    return { processedCount, migratedCount, mergedCount, skippedCount }
```

### Conflict Resolution Algorithm

```
function mergeOverlappingUser(sourceUser):
    targetUser = findByEmail(sourceUser.email)
    oldId = targetUser.id
    newId = sourceUser.id  // NextAuth ID takes precedence

    if oldId == newId:
        // IDs already match, just fill nulls and update password
        fillNullFields(targetUser, sourceUser)
        upsertCredentialAccount(newId, sourceUser.password)
        return

    // Must update ID and all FK references
    disableForeignKeyChecks()

    // Update all FK references FIRST (before changing user.id)
    UPDATE account SET user_id = newId WHERE user_id = oldId
    UPDATE session SET user_id = newId WHERE user_id = oldId
    UPDATE member SET user_id = newId WHERE user_id = oldId
    UPDATE oauth_access_token SET user_id = newId WHERE user_id = oldId
    UPDATE oauth_consent SET user_id = newId WHERE user_id = oldId
    UPDATE apikey SET user_id = newId WHERE user_id = oldId
    UPDATE invitation SET inviter_id = newId WHERE inviter_id = oldId
    UPDATE oauth_application SET user_id = newId WHERE user_id = oldId

    // Now update the user.id itself
    UPDATE user SET id = newId WHERE id = oldId

    // Fill NULL fields from source (don't overwrite existing data)
    fillNullFields(targetUser, sourceUser)

    // Ensure credential account exists with correct password
    upsertCredentialAccount(newId, sourceUser.password)

    enableForeignKeyChecks()
```

### Country Normalization Map

```typescript
const COUNTRY_MAP: Record<string, string> = {
  'PK': 'Pakistan',
  'US': 'United States',
  'GB': 'United Kingdom',
  'CA': 'Canada',
  'AE': 'United Arab Emirates',
  'SA': 'Saudi Arabia',
  'IN': 'India',
  'AU': 'Australia',
  'DE': 'Germany',
  'QA': 'Qatar',
  'MY': 'Malaysia',
  'AF': 'Afghanistan',
  'CN': 'China',
  'IE': 'Ireland',
  'EG': 'Egypt',
  'TR': 'Turkey',
  'OM': 'Oman',
  'IT': 'Italy',
  'Pakistan': 'Pakistan',  // Already correct
};

function normalizeCountry(country: string | null): string {
  if (!country) return 'Pakistan';  // Default
  const normalized = COUNTRY_MAP[country];
  if (normalized) return normalized;
  // Unknown code - keep original but log warning
  console.warn(`Unknown country code: ${country}`);
  return country;
}
```

### Username Generation Algorithm

```typescript
function generateUsername(email: string, existingUsernames: Set<string>): string {
  // Extract prefix from email
  const prefix = email.split('@')[0]
    .toLowerCase()
    .replace(/[^a-z0-9]/g, '-')  // Replace special chars with dash
    .replace(/-+/g, '-')          // Collapse multiple dashes
    .replace(/^-|-$/g, '')        // Trim leading/trailing dashes
    .substring(0, 40);            // Leave room for suffix

  // Generate random suffix
  const randomSuffix = () => Math.random().toString(36).substring(2, 6);

  let username = `${prefix}-${randomSuffix()}`;
  let attempts = 0;

  // Handle collisions (max 10 attempts)
  while (existingUsernames.has(username) && attempts < 10) {
    username = `${prefix}-${randomSuffix()}`;
    attempts++;
  }

  if (attempts >= 10) {
    // Fallback: append timestamp
    username = `${prefix}-${Date.now().toString(36)}`;
  }

  existingUsernames.add(username);  // Track for this batch
  return username;
}
```

## CLI Interface

```bash
# Usage
./node_modules/.bin/tsx scripts/migrate-nextauth-users.ts [options]

# Options
--dry-run           Log operations without modifying database (recommended first)
--limit <N>         Process only N users (for testing)
--offset <N>        Start from position N (for resuming)
--batch-size <N>    Users per batch (default: 500)
--verbose           Enable detailed logging

# Examples
# Dry run (always do this first)
tsx scripts/migrate-nextauth-users.ts --dry-run

# Test with 100 users
tsx scripts/migrate-nextauth-users.ts --dry-run --limit 100

# Resume from position 5000
tsx scripts/migrate-nextauth-users.ts --offset 5000

# Full migration
tsx scripts/migrate-nextauth-users.ts
```

## Output Format

```
═══════════════════════════════════════════════════════════════════
          NextAuth to Better Auth User Migration v1.0
═══════════════════════════════════════════════════════════════════

Configuration:
├── Source DB: postgresql://...@ep-red-dust-ahfa1cay-pooler... (14,821 users)
├── Target DB: postgresql://...@ep-cool-brook-a4wxx04i-pooler... (4 users)
├── Mode: DRY RUN (no database modifications)
├── Batch Size: 500
├── Offset: 0
└── Limit: (all)

═══════════════════════════════════════════════════════════════════

Processing batch 1/30 (users 1-500)...
  [INSERT] user@example.com → id preserved, country: PK → Pakistan
  [INSERT] test@test.com → id preserved, country: NULL → Pakistan
  [MERGE] existing@sso.com → id: abc123 → xyz789, filled: city, gender
  ...
  Batch complete: 498 inserted, 2 merged, 0 errors

[Progress: 3.4% | 500/14,821 | ETA: 12 minutes]

Processing batch 2/30 (users 501-1000)...
...

═══════════════════════════════════════════════════════════════════
                         Migration Summary
═══════════════════════════════════════════════════════════════════

Results:
├── Total Processed: 14,821
├── New Users Inserted: 14,818
├── Overlapping Users Merged: 3
├── Errors: 0
└── Duration: 8 minutes 42 seconds

Country Normalizations:
├── PK → Pakistan: 8,368
├── NULL → Pakistan: 7
├── US → United States: 42
└── (other): 6,404

Username Regenerations: 14,821 (all users)

═══════════════════════════════════════════════════════════════════

Verification Queries:
  SELECT COUNT(*) FROM public."user";  -- Expected: ~14,825
  SELECT COUNT(*) FROM public.account WHERE provider_id = 'credential';  -- Expected: ~14,821

Migration complete. Exit code: 0
```

## Error Handling

| Error Type | Handling | Exit Code |
|------------|----------|-----------|
| Missing env vars | Log error, exit immediately | 1 |
| Source DB connection failure | Log error, exit immediately | 1 |
| Target DB connection failure | Log error, exit immediately | 1 |
| Batch processing error | Rollback batch, log error, exit | 2 |
| Individual user error | Log, skip user, continue batch | 0 (warning) |
| Invalid password hash | Log warning, skip user | 0 (warning) |

## Test Strategy

### Manual Testing

1. **Dry Run Full**: `--dry-run` with all users - verify counts match expectations
2. **Limited Test**: `--limit 100` - verify small batch works correctly
3. **Conflict Test**: Ensure overlapping users get ID updated correctly
4. **Resume Test**: `--offset 5000` - verify resumption works
5. **Password Test**: Login with migrated user's original password

### Verification Queries

```sql
-- Post-migration verification
SELECT COUNT(*) FROM public."user";
SELECT COUNT(*) FROM public.account WHERE provider_id = 'credential';
SELECT country, COUNT(*) FROM public."user" GROUP BY country ORDER BY COUNT(*) DESC;

-- Verify specific migrated user
SELECT u.id, u.email, u.country, a.password
FROM public."user" u
JOIN public.account a ON u.id = a.user_id
WHERE u.email = 'test@example.com' AND a.provider_id = 'credential';
```

## Rollback Strategy

If migration fails or needs to be reverted:

1. **Before Migration**: Backup target database
2. **During Migration**: Each batch is transactional - rollback on error
3. **After Migration**: If issues found, restore from backup

```bash
# Pre-migration backup (recommended)
pg_dump "$DATABASE_URL" > backup-pre-migration-$(date +%Y%m%d).sql

# Restore if needed
psql "$DATABASE_URL" < backup-pre-migration-YYYYMMDD.sql
```

## Dependencies

| Dependency | Version | Purpose |
|------------|---------|---------|
| @neondatabase/serverless | ^0.9.0 | Neon database driver |
| drizzle-orm | ^0.29.0 | SQL query builder (optional) |
| dotenv | ^16.0.0 | Environment variable loading |
| tsx | ^4.0.0 | TypeScript execution |

## Environment Variables

```bash
# Required
NEXT_AUTH_PROD_DB_MIRROR=postgresql://...  # Source (read-only)
DATABASE_URL=postgresql://...               # Target (read-write)

# Optional
MIGRATION_BATCH_SIZE=500                    # Default batch size
MIGRATION_LOG_LEVEL=info                    # Logging verbosity
```

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Password hash corruption | Low | Critical | Exact copy, no transformation |
| FK constraint violation | Medium | High | Update FKs before user.id |
| Duplicate email conflict | Low | Medium | Check before insert |
| Connection timeout | Medium | Low | Batch processing, resume capability |
| Data loss | Low | Critical | Dry-run first, backup before |

## Complexity Tracking

No constitution violations require justification. Implementation follows minimal complexity:
- Single script (no new directories)
- Standard dependencies only
- Transaction-based safety
- CLI-driven workflow
