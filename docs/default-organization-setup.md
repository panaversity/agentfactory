# Default Organization Setup

## Overview

Panaversity uses a **hybrid multi-tenant model**:
- **Default Organization**: All users automatically join "Panaversity" on signup
- **Custom Organizations**: Schools/institutions can create their own organizations
- **Multi-Membership**: Users can belong to multiple organizations

## Architecture

### 1. Centralized Static Configuration (Performance Optimization)

**Location**: `src/lib/trusted-clients.ts`
```typescript
export const DEFAULT_ORG_ID = "panaversity-default-org-id";
export const DEFAULT_ORG_NAME = "Panaversity";
export const DEFAULT_ORG_SLUG = "panaversity";
```

**Why centralized?**
- ✅ All static configuration in one place (OAuth clients + org config)
- ✅ Imported by both `auth.ts` (runtime) and `seed-setup.ts` (seeding)
- ✅ No DB lookup on every signup (performance)
- ✅ Single source of truth - prevents ID mismatches

**Why hardcode?**
- ✅ Single default org that never changes
- ✅ Simple and predictable
- ✅ Cached at startup with validation

**Trade-off**: If org ID changes, must update code. For a single default org, this is acceptable.

### 2. Database Hook (Auto-Join on Signup)

**Location**: `src/lib/auth.ts:449-474`
```typescript
databaseHooks: {
  user: {
    create: {
      after: async (user) => {
        await db.insert(member).values({
          userId: user.id,
          organizationId: DEFAULT_ORG_ID,  // ← Hardcoded!
          role: "member",
        });
      }
    }
  }
}
```

**What it does:**
- Automatically adds new user to default organization
- Ensures every user has `tenant_id` from day 1
- No null `tenant_id` to handle in application code

### 3. Seed Script (Setup)

**Location**: `scripts/seed-setup.ts`

**Local Mode** (`pnpm run seed:setup`):
```bash
✅ Creates admin user (admin@robolearn.io / admin@robolearn.io)
✅ Creates default "Panaversity" organization
✅ Adds admin as owner
✅ Creates test organization (for dev testing)
✅ Seeds all 3 OAuth clients
```

**Production Mode** (`pnpm run seed:prod`):
```bash
✅ Requires admin user already exists (manual signup)
✅ Creates default "Panaversity" organization
✅ Adds existing admin as owner
✅ Seeds only production OAuth clients (skips RoboLearn)
❌ Skips test organization
❌ Skips auto-creating admin user
```

---

## User Flow

### New User Signup

```
1. User signs up → auth.ts creates user
2. Database hook fires → adds user to DEFAULT_ORG_ID
3. User gets JWT token:
   {
     tenant_id: "panaversity-default-org-id",
     organization_ids: ["panaversity-default-org-id"],
     org_role: "member"
   }
```

### User Joins School Organization

```
1. MIT creates organization → org_id: "mit-school-id"
2. MIT invites user → user accepts
3. User now member of 2 orgs:
   {
     tenant_id: "panaversity-default-org-id",  // Still active org
     organization_ids: [
       "panaversity-default-org-id",
       "mit-school-id"
     ],
     org_role: "member"
   }
4. User switches to MIT:
   await organization.setActive({ organizationId: "mit-school-id" })
5. New token:
   {
     tenant_id: "mit-school-id",  // Changed!
     org_role: "student",  // Different role in MIT
     organization_ids: [...]
   }
```

---

## Setup Instructions

### Local Development

```bash
# 1. Install dependencies
pnpm install

# 2. Set up database
pnpm run db:push

# 3. Run seed script (creates admin + orgs + clients)
pnpm run seed:setup

# 4. Start dev server
pnpm run dev

# 5. Sign in with admin credentials
Email: admin@robolearn.io
Password: admin@robolearn.io
```

### Production Deployment

```bash
# 1. Deploy database schema
pnpm run db:push

# 2. Manually create admin user
# Visit: https://auth.yourdomain.com/auth/sign-up
# Sign up with admin email

# 3. Run production seed
pnpm run seed:prod

# 4. Verify setup
# Visit: https://auth.yourdomain.com/admin/clients
```

---

## Troubleshooting

### Issue: User doesn't have tenant_id after signup

**Cause**: Database hook failed or default org not created

**Solution**:
1. Check server logs for errors
2. Verify default org exists:
   ```sql
   SELECT * FROM organization WHERE id = 'panaversity-default-org-id';
   ```
3. Manually add user to org:
   ```sql
   INSERT INTO member (id, user_id, organization_id, role, created_at)
   VALUES (uuid(), 'user-id', 'panaversity-default-org-id', 'member', NOW());
   ```

### Issue: Admin can't access /admin/clients

**Cause**: User role is not "admin"

**Solution**:
```sql
UPDATE "user" SET role = 'admin' WHERE email = 'admin@robolearn.io';
```

### Issue: Production seed fails with "No admin user found"

**Cause**: Admin user must be created manually first

**Solution**:
1. Sign up manually at `/auth/sign-up`
2. Use email: `admin@robolearn.io`
3. Run seed again

---

## Key Files

| File | Purpose |
|------|---------|
| `scripts/seed-setup.ts` | Creates default org + admin user + OAuth clients |
| `src/lib/auth.ts:19` | Hardcoded `DEFAULT_ORG_ID` constant |
| `src/lib/auth.ts:449-474` | Database hook for auto-join |
| `src/lib/trusted-clients.ts` | OAuth client configurations |

---

## Benefits of This Approach

1. **No Null tenant_id**: Every user has organization from day 1
2. **Performance**: No DB lookup on signup (hardcoded org ID)
3. **Flexibility**: Users can still join multiple organizations
4. **B2B Ready**: Schools/institutions can create their own orgs
5. **Simpler Logic**: No need to handle null tenant_id case in app

---

## Comparison: Before vs After

### Before (Optional Organizations)
```typescript
{
  tenant_id: null,  // ← Need to handle null!
  organization_ids: [],
  org_role: null
}
```

**Problems**:
- App must handle null tenant_id
- Unclear which data is "default" vs "organization-scoped"
- Users have no organizational context

### After (Default Organization)
```typescript
{
  tenant_id: "panaversity-default-org-id",  // ← Always present!
  organization_ids: ["panaversity-default-org-id"],
  org_role: "member"
}
```

**Benefits**:
- Consistent data scoping
- Clear organizational context
- Simpler application logic
- Ready for B2B expansion

---

## Next Steps

1. **Update Frontend**: Use `tenant_id` for data scoping
2. **Create Org Management UI**: Allow institutions to create orgs
3. **Add Org Switcher**: Let users switch between organizations
4. **Implement Tenant Isolation**: Filter all queries by `tenant_id`

---

## References

- [Better Auth Organizations](https://www.better-auth.com/docs/plugins/organization)
- [Multi-Tenancy Documentation](./multi-tenancy.md)
- [Environment Variables](./environment-variables.md)
