# Continue NextAuth to Better Auth Migration

## Context

Read these files first:
1. `docs/migration/nextauth-to-betterauth-guide.md` - Full migration guide
2. `history/prompts/004-nextauth-migration/001-smoke-test.prompt.md` - Smoke test results
3. `src/lib/auth.ts` - Password verification already configured (lines 179-192)
4. `scripts/test-password-migration.ts` - Test script (reference for full script)

## What's Done

- [x] bcrypt password verification working in auth.ts
- [x] Schema mapping documented
- [x] Test migration of 4 users successful
- [x] Profile columns (city, gender, father_name, country) in DB

## What's Needed

Implement full migration script at `scripts/migrate-nextauth-users.ts`:

### Requirements

1. **Batch Processing**
   - 500 users per batch
   - Transaction per batch
   - Progress logging with percentage and ETA

2. **Source Query (Old NextAuth DB)**
   ```sql
   SELECT u.id, u.name, u.email, u.password, u."emailVerified",
          u.image, u.role, u.username, u.phone_number, u.country,
          u.created_at, u.updated_at,
          p.city, p.gender, p.father_name
   FROM public.users u
   LEFT JOIN public.profile p ON u.id = p.user_id
   WHERE u.password IS NOT NULL
   ORDER BY u.created_at
   ```

3. **Conflict Resolution (60 users)**
   For users existing in both DBs:
   - Get old NextAuth user ID
   - Update Better Auth user.id to match
   - Update all FK references (account, session, member, etc.)

4. **Insert Logic**
   ```typescript
   // For each user not in Better Auth:
   // 1. Insert into "user" table (preserve ID)
   // 2. Insert into "account" table with provider_id='credential'
   ```

5. **Safety Features**
   - `--dry-run` flag (log without insert)
   - `--limit N` flag (process only N users)
   - `--offset N` flag (resume from position)
   - Duplicate email check before insert

### Environment Variables

```bash
NEXT_AUTH_PROD_DB_MIRROR  # Source (read-only)
DATABASE_URL              # Target
```

### Expected Output

```
=== NextAuth to Better Auth Migration ===
Source: NEXT_AUTH_PROD_DB_MIRROR (14,821 users)
Target: DATABASE_URL (126 users)
Overlap: 60 users

Mode: DRY RUN (no changes)

Processing batch 1/30 (500 users)...
  - Migrated: 498
  - Skipped (exists): 2
  - Errors: 0

[Progress: 3.4% | ETA: 12 minutes]
...
```

## Run Commands

```bash
# Dry run first
./node_modules/.bin/tsx scripts/migrate-nextauth-users.ts --dry-run

# Limited test
./node_modules/.bin/tsx scripts/migrate-nextauth-users.ts --limit 100

# Full migration
./node_modules/.bin/tsx scripts/migrate-nextauth-users.ts
```

## Verification After Migration

```bash
# Count users
source .env && psql "$DATABASE_URL" -c "SELECT COUNT(*) FROM public.\"user\";"

# Should be ~14,900+ (14,821 migrated + 126 existing - 60 overlap)
```
