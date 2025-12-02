# Multi-Tenancy & Organizations

## What are Organizations?

Organizations in Better Auth enable **multi-tenancy** - allowing users to belong to different tenants with role-based access control.

### Key Concepts

```
User: john@example.com
├── Member of "MIT School" (role: student)
├── Member of "Harvard School" (role: teacher)
└── Active Organization: "MIT School" ← becomes tenant_id in JWT
```

When the user selects an active organization, their JWT token includes:

```json
{
  "sub": "user-123",
  "email": "john@example.com",
  "tenant_id": "mit-school-org-id",
  "org_role": "student",
  "organization_ids": ["mit-school-org-id", "harvard-school-org-id"]
}
```

## When to Use Organizations

### ✅ Use organizations for:
- 🏫 **Multiple schools/institutions** using the same platform (each school = organization)
- 🏢 **B2B SaaS** where each company gets isolated data (each company = organization)
- 👥 **Team-based apps** with data segregation (each team = organization)
- 📊 **Department separation** within a company

### ❌ Organizations NOT needed for:
- Single-tenant applications (all users share same data)
- Simple user authentication without data segregation
- Basic OAuth flows (authorization works without organizations)

## How Organizations Work

### 1. Organization Plugin (enabled in auth.ts)

```typescript
import { organization } from "better-auth/plugins/organization";

export const auth = betterAuth({
  plugins: [
    organization({
      allowUserToCreateOrganization: true,
    }),
  ],
});
```

### 2. Tenant-Scoped Data Access

Applications use `tenant_id` from the JWT to scope database queries:

```typescript
// Frontend: Get session
const session = await authClient.getSession();
const tenantId = session.user.tenant_id;

// Backend: Query data scoped to organization
const lessons = await db.select()
  .from(lessons)
  .where(eq(lessons.organizationId, tenantId));
```

### 3. Organization Switching

Users can switch between organizations if they're members of multiple:

```typescript
// Change active organization (updates tenant_id in next token)
await authClient.organization.setActive({
  organizationId: "new-org-id"
});
```

## Seeding Test Organizations

The `seed:setup` script (without `--prod` flag) creates a test organization:

```bash
pnpm run seed:setup
```

**What it creates:**
- Organization: "RoboLearn Test Organization"
- Slug: `robolearn-test`
- Adds `admin@robolearn.io` as organization owner (if user exists)

**If test user doesn't exist:**
1. Sign up first: `http://localhost:3001/auth/sign-up` with `admin@robolearn.io`
2. Verify email
3. Run `pnpm run seed:setup` again to add organization membership

## Production Considerations

### Don't seed organizations in production

```bash
# Production mode skips organization seeding
pnpm run seed:prod --prod
```

**Why?**
- Organizations should be created by users or admins via the application UI
- Each institution/company should create their own organization
- Avoid pre-seeded test data in production databases

### Organization Management

Organizations can be managed via:
1. **Better Auth Admin API** (if enabled)
2. **Custom Admin Dashboard** in your application
3. **Direct Database Access** (for emergency cases)

## Testing Multi-Tenancy

### Test Scenario: Multiple Schools

```typescript
// School 1: MIT
const mitOrg = await createOrganization({ name: "MIT" });

// School 2: Harvard
const harvardOrg = await createOrganization({ name: "Harvard" });

// Add student to MIT
await addMember({
  userId: "student-1",
  organizationId: mitOrg.id,
  role: "member"
});

// Verify tenant isolation
const mitSession = await getSession(); // tenant_id = mit-org-id
const lessons = await getLessons(); // Only MIT's lessons returned
```

### Test Cases

✅ **Tenant Isolation**: User from Org A cannot access Org B's data
✅ **Multi-Membership**: User can belong to multiple organizations
✅ **Role-Based Access**: Different roles within same organization
✅ **Organization Switching**: Changing active organization updates tenant_id
✅ **Token Claims**: Verify `tenant_id` and `organization_ids` in JWT

## Real-World Example: School Management Platform

### Scenario
Platform serves 3 schools: MIT, Harvard, Stanford

### Setup
```typescript
// Each school gets an organization
const schools = [
  { id: "mit-id", name: "MIT", slug: "mit" },
  { id: "harvard-id", name: "Harvard", slug: "harvard" },
  { id: "stanford-id", name: "Stanford", slug: "stanford" },
];

// Students belong to their school
Student A → MIT (member)
Teacher B → MIT (admin) + Harvard (member)
Admin C → Platform-wide (super admin)
```

### Data Access
```typescript
// Student A logs in, selects MIT
session.user.tenant_id = "mit-id"

// Query lessons - automatically filtered by tenant_id
const lessons = await db.select()
  .from(lessons)
  .where(eq(lessons.organizationId, session.user.tenant_id));
// Returns: Only MIT's lessons

// Teacher B switches to Harvard
await organization.setActive({ organizationId: "harvard-id" });
// New token issued with tenant_id = "harvard-id"
```

### Benefits
- ✅ **Data Isolation**: MIT cannot see Harvard's data
- ✅ **Shared Infrastructure**: All schools use same auth server
- ✅ **Cross-School Users**: Teachers can work at multiple schools
- ✅ **Simplified Management**: One database, automatic scoping

## Common Patterns

### Pattern 1: B2B SaaS (Each Company = Organization)
```
Company A → Organization ID: company-a
  ├── Admin users (full access to company data)
  ├── Regular users (limited access)
  └── Data scoped by tenant_id = company-a
```

### Pattern 2: Educational Platform (Each School = Organization)
```
School A → Organization ID: school-a
  ├── Teachers (create lessons)
  ├── Students (view lessons)
  └── Lessons scoped by tenant_id = school-a
```

### Pattern 3: Team Collaboration (Each Team = Organization)
```
Team A → Organization ID: team-a
  ├── Owner (manage team)
  ├── Members (collaborate)
  └── Projects scoped by tenant_id = team-a
```

## Migration Strategy

### Starting Without Organizations

If you built your app without multi-tenancy and want to add it later:

**Before (Single-Tenant):**
```typescript
// All users share data
const lessons = await db.select().from(lessons);
```

**After (Multi-Tenant):**
```typescript
// Add organizationId to schema
const lessons = pgTable("lessons", {
  id: text("id").primaryKey(),
  organizationId: text("organization_id"), // NEW
  // ... other fields
});

// Filter by tenant
const lessons = await db.select()
  .from(lessons)
  .where(eq(lessons.organizationId, session.user.tenant_id));
```

**Migration Steps:**
1. Add `organizationId` column to all tenant-scoped tables
2. Create default organization for existing data
3. Update all existing records with default organization ID
4. Update queries to filter by `tenant_id`
5. Enable organization plugin in Better Auth

## FAQ

### Q: Do I need organizations for basic OAuth?
**A:** No. OAuth flows work fine without organizations. Organizations are only for multi-tenant data isolation.

### Q: Can a user belong to multiple organizations?
**A:** Yes! The `organization_ids` claim lists all organizations, and `tenant_id` shows the currently active one.

### Q: How do I delete an organization?
**A:** Use Better Auth's organization API or direct database deletion (cascades to members).

### Q: What if I don't want tenant isolation?
**A:** Simply don't filter by `tenant_id` in your queries. The token still includes it, but you can ignore it.

### Q: Can organizations have nested hierarchies?
**A:** Not natively. Better Auth organizations are flat. For nested structures (departments, teams), implement custom logic.

---

## References

- [Better Auth Organizations Docs](https://www.better-auth.com/docs/plugins/organization)
- [Multi-Tenancy Patterns](https://docs.microsoft.com/en-us/azure/architecture/guide/multitenant/overview)
- [JWT Token Claims Reference](../README.md#token-claims-reference)
