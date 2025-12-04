---
id: 004-001
title: NextAuth to Better Auth Migration Smoke Test
stage: spec
date: 2024-12-04
surface: claude-code
model: claude-opus-4-5
feature: 004-nextauth-migration
branch: main
user: mjs
command: manual
labels: [migration, nextauth, better-auth, bcrypt, password]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - path: src/lib/auth.ts
    action: modified
    description: Added bcrypt password verification support
  - path: scripts/test-password-migration.ts
    action: created
    description: Test migration script for smoke testing
tests:
  - name: bcrypt-login
    status: passed
    description: Users with bcrypt passwords can login
  - name: scrypt-login
    status: passed
    description: New users with scrypt passwords can login
  - name: id-preservation
    status: passed
    description: User IDs preserved during migration
---

## Prompt

We have ~14,800 users in NextAuth production database that need to be migrated to Better Auth SSO. Requirements:
1. Preserve user IDs (used across microservices)
2. Handle password hash migration (NextAuth uses bcrypt, Better Auth uses scrypt)
3. Migrate profile fields (city, gender, father_name, country)
4. Handle ~60 users that exist in both systems (conflict resolution)
5. Test with a few users before full migration

## Response snapshot

### Smoke Test Results

**Successfully completed:**
1. Identified schema differences between NextAuth and Better Auth
2. Implemented custom bcrypt password verification in `auth.ts`
3. Created test migration script
4. Migrated 4 test users with preserved UUIDs
5. Verified login works with bcrypt passwords

### Key Findings

| Metric | Value |
|--------|-------|
| Old NextAuth DB | 14,821 users |
| New Better Auth DB | 126 users |
| Overlapping users | 60 |
| Users to migrate | ~14,761 |

### Schema Mapping

```
NextAuth                    Better Auth
─────────────────────────────────────────────
users.id           →        user.id (preserve!)
users.email        →        user.email
users.name         →        user.name
users.password     →        account.password (providerId='credential')
users.emailVerified →       user.email_verified
users.role         →        user.role
users.username     →        user.username
users.phone_number →        user.phone_number
profile.city       →        user.city
profile.gender     →        user.gender
profile.father_name →       user.father_name
users.country      →        user.country
```

### Password Verification Solution

Added to `src/lib/auth.ts:179-192`:
```typescript
password: {
  verify: async ({ password, hash }) => {
    // Check if this is a bcrypt hash (migrated from NextAuth)
    if (hash.startsWith('$2a$') || hash.startsWith('$2b$') || hash.startsWith('$2y$')) {
      return bcrypt.compare(password, hash);
    }
    // Default: Use Better Auth's scrypt verification
    const { verifyPassword } = await import('better-auth/crypto');
    return verifyPassword({ password, hash });
  },
},
```

## Outcome

- ✅ Impact: High - Enables migration of 14,800+ users without password reset
- 🧪 Tests: 4/4 passed (bcrypt login, scrypt login, ID preservation, profile fields)
- 📁 Files: 2 modified/created (auth.ts, test-password-migration.ts)
- 🔁 Next prompts: Full migration script with batch processing and conflict resolution
- 🧠 Reflection: bcrypt/scrypt compatibility was the critical unknown - solved with custom verify function

## Evaluation notes (flywheel)

- Failure modes observed: None - smoke test successful
- Graders run and results (PASS/FAIL): Manual verification PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Full batch migration with 500 users/batch
