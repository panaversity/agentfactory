# Quickstart Guide: Hackathon Platform

**For**: Developers onboarding to the hackathon platform project
**Time to First Run**: 15-20 minutes
**Prerequisites**: Node.js 20+, pnpm, Docker (optional), Git

---

## Quick Links

- **Spec**: `specs/046-hackathon-platform/spec.md`
- **Data Model**: `specs/046-hackathon-platform/data-model.md`
- **Implementation Plan**: `specs/046-hackathon-platform/plan.md`
- **Research**: `specs/046-hackathon-platform/research.md`

---

## 1. Clone & Setup (5 minutes)

### 1.1 Clone Repository

```bash
# If not already cloned
git clone <repo-url>
cd h-app

# Checkout feature branch
git checkout hackathon-platform

# Install dependencies (from monorepo root)
pnpm install
```

### 1.2 Start SSO Server (Required Dependency)

The hackathon app authenticates via the existing SSO server. Start it first:

```bash
# Terminal 1: SSO Server
cd apps/sso

# Copy environment template
cp .env.local.example .env.local

# Edit .env.local - add your Neon DATABASE_URL
# Get from: https://neon.tech (free tier OK)

# Run migrations
pnpm db:push

# Seed SSO with hackathon client
pnpm run seed:setup

# Start SSO on port 3001
pnpm dev
```

**Verify**: Visit http://localhost:3001 - should see SSO homepage.

### 1.3 Configure Hackathon App

```bash
# Terminal 2: Hackathon App
cd apps/hackathon

# Copy environment template
cp .env.local.example .env.local

# Edit .env.local
```

**Required Environment Variables**:

```env
# Database - Option 1: Shared with SSO (recommended for dev)
DATABASE_URL=postgresql://user:pass@host/db?sslmode=require

# Database - Option 2: Separate Neon project
# DATABASE_URL=<create-new-neon-project>

# OAuth (SSO Integration) - Use defaults for local dev
SSO_OAUTH_CLIENT_ID=hackathon-public-client
SSO_OAUTH_ISSUER=http://localhost:3001
SSO_OAUTH_AUTHORIZE_URL=http://localhost:3001/oauth2/authorize
SSO_OAUTH_TOKEN_URL=http://localhost:3001/oauth2/token
SSO_OAUTH_USERINFO_URL=http://localhost:3001/oauth2/userinfo
SSO_OAUTH_JWKS_URL=http://localhost:3001/.well-known/jwks.json
SSO_OAUTH_REDIRECT_URI=http://localhost:3002/api/auth/callback

# Session Encryption - Generate with: openssl rand -base64 32
SESSION_SECRET=<paste-generated-secret>

# App Configuration
NEXT_PUBLIC_APP_NAME=Hackathon Platform
NEXT_PUBLIC_APP_URL=http://localhost:3002
NEXT_PUBLIC_SSO_URL=http://localhost:3001
```

**Generate SESSION_SECRET**:
```bash
openssl rand -base64 32
# Copy output to .env.local
```

### 1.4 Run Database Migrations

```bash
# Still in apps/hackathon

# Generate SQL from schema
pnpm db:generate

# Apply to database
pnpm db:push

# Seed development data (optional)
pnpm run seed:dev
```

**Verify**: Visit https://neon.tech → Your Project → Tables. Should see:
- `hackathons`
- `teams`
- `team_members`
- `submissions`
- `scores`
- `judging_criteria`
- `hackathon_roles`
- `team_messages`

---

## 2. First Run (5 minutes)

### 2.1 Start Development Server

```bash
# In apps/hackathon
pnpm dev
```

**Expected Output**:
```
▲ Next.js 16.0.0
- Local:        http://localhost:3002
- Ready in 2.3s
```

### 2.2 Test OAuth Flow

1. **Visit**: http://localhost:3002
2. **Click**: "Sign in with SSO"
3. **Redirects to**: http://localhost:3001/oauth2/authorize (SSO login)
4. **Login with**: Test user from SSO (or create new account)
5. **Consent**: Click "Allow" to grant access
6. **Redirects back**: http://localhost:3002/dashboard

**If successful**: You're logged in and see the hackathon dashboard.

**If redirect fails**: Check SSO client configuration in `apps/sso/src/lib/trusted-clients.ts`.

### 2.3 Create First Hackathon

1. **Dashboard**: Click "Create Hackathon"
2. **Fill form**:
   - Title: "Test Hackathon"
   - Description: "My first test hackathon"
   - Slug: "test-hackathon"
   - Dates: (future dates)
   - Team size: 2-5
3. **Submit**: Should create and redirect to hackathon details

**Expected Result**: Hackathon appears in dashboard, status = "draft"

---

## 3. Development Workflow

### 3.1 Project Structure

```
apps/hackathon/
├── src/
│   ├── app/                    # Next.js 16 App Router
│   │   ├── (auth)/            # Auth routes (login, callback)
│   │   ├── (dashboard)/       # Protected routes (requires login)
│   │   ├── api/               # API routes (REST endpoints)
│   │   ├── layout.tsx         # Root layout
│   │   └── page.tsx           # Home page
│   ├── components/            # React components
│   │   ├── ui/               # shadcn/ui components
│   │   ├── hackathons/       # Hackathon-specific components
│   │   ├── teams/            # Team components
│   │   └── layout/           # Layout components
│   ├── db/                    # Database layer
│   │   ├── schema.ts         # Drizzle schema (tables, indexes)
│   │   ├── index.ts          # DB connection
│   │   └── queries/          # Reusable query functions
│   ├── lib/                   # Utilities
│   │   ├── auth/             # OAuth & session management
│   │   ├── validation/       # Zod schemas
│   │   └── utils.ts          # Helper functions
│   └── types/                 # TypeScript types
└── scripts/                   # Utility scripts (seed, etc.)
```

### 3.2 Common Tasks

**Add a new API route**:
```bash
# Create route file
touch src/app/api/your-route/route.ts

# Implement GET/POST/etc handlers
# See existing routes for patterns
```

**Add a new database table**:
```typescript
// 1. Add to src/db/schema.ts
export const yourTable = pgTable("your_table", { ... });

// 2. Generate migration
pnpm db:generate

// 3. Apply to database
pnpm db:push
```

**Add a new component**:
```bash
# Create component file
touch src/components/your-feature/your-component.tsx

# Use shadcn for UI primitives
npx shadcn@latest add <component-name>
```

**Add validation schema**:
```typescript
// src/lib/validation/your-feature.ts
import { z } from "zod";

export const yourSchema = z.object({
  field: z.string().min(3),
});
```

### 3.3 Debugging Tips

**Check session data**:
```typescript
// In any server component or API route
import { getSession } from "@/lib/auth/session";

const session = await getSession();
console.log(session); // userId, email, organizationId, etc.
```

**Inspect database queries**:
```bash
# Open Drizzle Studio (visual DB browser)
pnpm db:studio

# Opens at http://localhost:4983
```

**View SSO OAuth flow**:
```bash
# SSO server logs show OAuth requests
# Check Terminal 1 (SSO server)
```

**Common errors**:

| Error | Cause | Fix |
|-------|-------|-----|
| "Invalid state" | Session expired | Clear cookies, retry login |
| "Unauthorized" | Session missing | Check middleware.ts, login again |
| "Database connection failed" | Wrong DATABASE_URL | Check .env.local |
| "JWKS fetch failed" | SSO not running | Start SSO server (Terminal 1) |

---

## 4. Testing

### 4.1 Manual Testing Checklist

**OAuth Flow**:
- [ ] Login redirects to SSO
- [ ] Callback creates session
- [ ] Dashboard loads with user data
- [ ] Logout clears session

**Hackathon CRUD**:
- [ ] Create hackathon (organizer)
- [ ] Edit hackathon details
- [ ] List hackathons (multi-tenant filtered)
- [ ] View hackathon details

**Team Formation**:
- [ ] Create team as participant
- [ ] Invite code generated
- [ ] Join team via invite code
- [ ] Team size limits enforced

**Submissions**:
- [ ] Submit project before deadline
- [ ] Update submission before deadline
- [ ] Cannot submit after deadline

**Judging**:
- [ ] Judge sees assigned submissions
- [ ] Score submission on all criteria
- [ ] Leaderboard calculates correctly

### 4.2 Automated Tests (Future)

```bash
# Unit tests (when implemented)
pnpm test

# E2E tests with Playwright
pnpm test:e2e
```

---

## 5. Multi-Tenant Testing

**Important**: All data is scoped to `organization_id`. Test cross-org isolation:

### 5.1 Create Test Organizations

```sql
-- In Drizzle Studio or pgAdmin
INSERT INTO organization (id, name, slug, created_at)
VALUES
  ('org-a', 'Organization A', 'org-a', NOW()),
  ('org-b', 'Organization B', 'org-b', NOW());
```

### 5.2 Create Users in Each Org

1. **Sign up** as User A (will auto-join default org)
2. **Manually assign** to org-a via database:
   ```sql
   UPDATE member
   SET organization_id = 'org-a'
   WHERE user_id = '<user-a-id>';
   ```

3. **Repeat** for User B with org-b

### 5.3 Verify Isolation

- Login as User A → Create hackathon
- Login as User B → Should NOT see User A's hackathon
- Check database: `hackathons.organization_id` matches user's org

**Expected**: Zero cross-org data leaks (SC-007)

---

## 6. Performance Testing

### 6.1 Load Testing with k6

**Install k6**:
```bash
# macOS
brew install k6

# Linux
sudo apt-get install k6
```

**Create load test** (`scripts/load-test.js`):
```javascript
import http from 'k6/http';
import { check } from 'k6';

export let options = {
  stages: [
    { duration: '30s', target: 100 }, // Ramp up to 100 users
    { duration: '1m', target: 500 },  // 500 concurrent users (SC-004)
    { duration: '30s', target: 0 },   // Ramp down
  ],
};

export default function() {
  let res = http.get('http://localhost:3002/api/hackathons');
  check(res, {
    'status is 200': (r) => r.status === 200,
    'response time < 2s': (r) => r.timings.duration < 2000, // SC-006
  });
}
```

**Run test**:
```bash
k6 run scripts/load-test.js
```

**Success criteria**:
- ✅ 500 concurrent users handled without errors (SC-004)
- ✅ 95% of requests < 2s response time (SC-006)

---

## 7. Troubleshooting

### Issue: "Cannot connect to SSO"

**Symptoms**: OAuth login fails, redirect errors

**Solutions**:
1. **Check SSO is running**: http://localhost:3001 should load
2. **Verify client registration**:
   ```bash
   cd apps/sso
   pnpm run seed:setup
   ```
3. **Check redirect URIs**: Must match exactly (http vs https, port)

### Issue: "Session expires immediately"

**Symptoms**: Login succeeds but redirects back to login

**Solutions**:
1. **Check SESSION_SECRET** is set in `.env.local`
2. **Clear cookies**: Browser Dev Tools → Application → Clear cookies
3. **Check middleware.ts**: Ensure proper session validation

### Issue: "Database connection failed"

**Symptoms**: 500 errors, "Cannot connect to database"

**Solutions**:
1. **Check DATABASE_URL** format:
   ```
   postgresql://user:pass@host/dbname?sslmode=require
   ```
2. **Verify Neon project**: Should be active, not paused
3. **Test connection**: Use Drizzle Studio or pgAdmin

### Issue: "Multi-tenant data leaks"

**Symptoms**: Users see hackathons from other organizations

**Solutions**:
1. **Check queries** include `organization_id` filter:
   ```typescript
   where: and(
     eq(hackathons.organizationId, session.organizationId),
     eq(hackathons.published, true)
   )
   ```
2. **Verify session** has correct `organizationId` from JWT
3. **Run isolation test** (Section 5.2)

---

## 8. Next Steps

After quickstart, proceed with implementation:

1. **Read the plan**: `specs/046-hackathon-platform/plan.md`
2. **Check data model**: `specs/046-hackathon-platform/data-model.md`
3. **Start with Phase 0**: Scaffold project structure
4. **Implement Phase 1**: Database + OAuth integration
5. **Test incrementally**: Validate each phase before moving forward

---

## 9. Useful Commands

```bash
# Development
pnpm dev                    # Start dev server (port 3002)
pnpm build                  # Production build
pnpm start                  # Start production server

# Database
pnpm db:generate            # Generate migrations from schema
pnpm db:push                # Apply migrations to database
pnpm db:studio              # Open Drizzle Studio (visual DB browser)
pnpm run seed:dev           # Seed development data

# Code Quality
pnpm lint                   # Run ESLint
pnpm type-check             # Run TypeScript compiler

# Testing (when implemented)
pnpm test                   # Run unit tests
pnpm test:e2e               # Run E2E tests
```

---

## 10. Resources

### Documentation
- [Next.js 16 Docs](https://nextjs.org/docs)
- [Drizzle ORM](https://orm.drizzle.team/)
- [Arctic OAuth](https://arctic.js.org/)
- [iron-session](https://github.com/vvo/iron-session)
- [shadcn/ui](https://ui.shadcn.com/)

### Project Specs
- Feature Spec: `specs/046-hackathon-platform/spec.md`
- Data Model: `specs/046-hackathon-platform/data-model.md`
- Implementation Plan: `specs/046-hackathon-platform/plan.md`
- Research: `specs/046-hackathon-platform/research.md`

### Internal References
- SSO Implementation: `apps/sso/`
- SSO OAuth Config: `apps/sso/src/lib/trusted-clients.ts`
- SSO Database Schema: `apps/sso/auth-schema.ts`

### Community
- Team Slack: `#hackathon-platform` (if applicable)
- GitHub Issues: Tag with `hackathon-platform`

---

## 11. FAQ

**Q: Can I use a different database than Neon?**

A: Yes, any PostgreSQL database works. Update `DATABASE_URL` in `.env.local`. Neon is recommended for serverless deployments.

**Q: Do I need to modify the SSO server?**

A: No. The SSO client configuration is added manually (documented in spec). SSO code remains unchanged.

**Q: Can I test without SSO?**

A: Not recommended. OAuth flow is core to the architecture. Mock auth can be added for unit tests only.

**Q: What's the database migration strategy?**

A: Drizzle Kit generates SQL migrations from schema changes. Always review generated SQL before applying (`db:push`).

**Q: How do I add a new role type?**

A:
1. Add to `hackathon_roles.role` enum in schema
2. Update `ROLE_PERMISSIONS` in `src/lib/auth/permissions.ts`
3. Regenerate schema: `pnpm db:generate && pnpm db:push`

**Q: Can participants be in multiple teams?**

A: No. Constraint: one team per user per hackathon. Enforced in `joinTeamByInviteCode()` query.

**Q: How are team leader promotions handled?**

A: MVP: Manual assignment by organizer/manager. Auto-promotion is Phase 2 (NG-9 in spec).

---

**Ready to start?** Follow Section 1 to set up your environment, then proceed to the implementation plan.

**Questions?** Refer to spec documents or raise in team Slack.
