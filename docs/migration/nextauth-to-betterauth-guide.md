# NextAuth to Better Auth Migration Guide

## Overview

This guide documents the migration of ~14,800 users from NextAuth to Better Auth SSO.

**Status:** Smoke test completed ✅ | Full migration pending

## Database Configuration

```bash
# Old NextAuth production mirror (read-only source)
NEXT_AUTH_PROD_DB_MIRROR='postgresql://...'

# New Better Auth database (migration target)
DATABASE_URL='postgresql://...'
```

## Schema Mapping

### User Table

| NextAuth (users) | Better Auth (user) | Notes |
|------------------|-------------------|-------|
| `id` (uuid) | `id` (text) | **MUST PRESERVE** - used across microservices |
| `email` | `email` | unique |
| `name` | `name` | required |
| `emailVerified` (bool) | `email_verified` (bool) | |
| `image` | `image` | |
| `role` | `role` | |
| `username` | `username` | unique |
| `phone_number` | `phone_number` | |
| `country` | `country` | |
| `created_at` | `created_at` | |
| `updated_at` | `updated_at` | |

### Profile Table → User Table

| NextAuth (profile) | Better Auth (user) |
|-------------------|-------------------|
| `city` | `city` |
| `gender` | `gender` |
| `father_name` | `father_name` |

### Password Storage

| NextAuth | Better Auth |
|----------|-------------|
| `users.password` (bcrypt) | `account.password` with `provider_id='credential'` |

## Password Hash Compatibility

**Problem:** NextAuth uses bcrypt (`$2b$12$...`), Better Auth uses scrypt (hex).

**Solution:** Custom password verification in `src/lib/auth.ts:179-192`:

```typescript
password: {
  verify: async ({ password, hash }) => {
    // Bcrypt hashes from NextAuth
    if (hash.startsWith('$2a$') || hash.startsWith('$2b$') || hash.startsWith('$2y$')) {
      return bcrypt.compare(password, hash);
    }
    // Scrypt for new Better Auth users
    const { verifyPassword } = await import('better-auth/crypto');
    return verifyPassword({ password, hash });
  },
},
```

**Progressive Migration:** When users change their password, they get a new scrypt hash automatically.

## Migration Statistics

| Metric | Count |
|--------|-------|
| Total users in NextAuth | 14,821 |
| Users in Better Auth | 126 |
| Overlapping (both DBs) | 60 |
| To migrate (new) | ~14,761 |

## Conflict Resolution Strategy

For the 60 users that exist in both databases:

1. **Keep Better Auth user data** (more recent)
2. **Update `user.id`** to match NextAuth ID (critical for microservice FKs)
3. **Update all FK references:**
   - `account.user_id`
   - `session.user_id`
   - `member.user_id`
   - `oauth_access_token.user_id`
   - `oauth_consent.user_id`

## Migration Script Requirements

### Batch Processing
- Process 500 users per batch
- Transaction per batch (rollback on failure)
- Progress logging with ETA
- Resume capability (track last processed ID)

### Data Validation
- Verify email uniqueness before insert
- Handle NULL values gracefully
- Validate UUID format

### Safety Features
- Dry-run mode (log without insert)
- Backup verification before start
- Rollback script generation

## Test Migration Script

Location: `scripts/test-password-migration.ts`

```bash
# Run test migration (3 users)
./node_modules/.bin/tsx scripts/test-password-migration.ts
```

## Verification Steps

1. **Count verification:**
   ```sql
   SELECT COUNT(*) FROM public."user";
   SELECT COUNT(*) FROM public.account WHERE provider_id = 'credential';
   ```

2. **Login test:** Try logging in with a migrated user's email and original password

3. **ID verification:**
   ```sql
   SELECT u.id, u.email, a.provider_id
   FROM public."user" u
   JOIN public.account a ON u.id = a.user_id
   WHERE u.email = 'test@example.com';
   ```

## Files Modified

| File | Change |
|------|--------|
| `src/lib/auth.ts` | Added bcrypt import and custom password verify |
| `scripts/test-password-migration.ts` | Created test migration script |

## Next Steps

1. [ ] Implement full migration script with batch processing
2. [ ] Add conflict resolution for 60 overlapping users
3. [ ] Create rollback script
4. [ ] Run dry-run on staging
5. [ ] Execute production migration
6. [ ] Verify all users can login
7. [ ] Monitor for issues post-migration

## Related Documents

- PHR: `history/prompts/004-nextauth-migration/001-smoke-test.prompt.md`
- Auth config: `src/lib/auth.ts`
- Schema: `auth-schema.ts`
