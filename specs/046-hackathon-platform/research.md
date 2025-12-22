# Technical Research: Hackathon Platform

**Feature**: 046-hackathon-platform
**Date**: 2025-12-22
**Research Areas**: Next.js 16 OAuth Client, Drizzle Schema Design, Multi-tenant Architecture

---

## 1. Next.js 16 OAuth Client Integration

### 1.1 OAuth2 Client Flow (PKCE)

The hackathon app will consume SSO as a **public OAuth client** using PKCE (Proof Key for Code Exchange):

**Flow**:
```
1. User clicks "Login" → Redirect to SSO /oauth2/authorize
2. SSO authenticates user → Redirects back with authorization code
3. App exchanges code for access token at SSO /oauth2/token
4. App uses access token to fetch user info from SSO /oauth2/userinfo
5. App creates local session with user data from JWT claims
```

**Key Libraries**:
- **arctic** (recommended by Better Auth community): Lightweight OAuth client library
- **next-auth** v5 (alternative): Full-featured auth solution, may be overkill
- **oslo** (alternative): Low-level crypto utilities for PKCE

**Recommendation**: Use **arctic** for lightweight OAuth client implementation.

```typescript
// Example with arctic
import { OAuth2Client } from "arctic";

const oauthClient = new OAuth2Client({
  clientId: "hackathon-public-client",
  clientSecret: null, // Public client - no secret
  authorizationEndpoint: "http://localhost:3001/oauth2/authorize",
  tokenEndpoint: "http://localhost:3001/oauth2/token",
  redirectURI: "http://localhost:3002/api/auth/callback",
});

// Generate PKCE challenge
const state = generateState();
const codeVerifier = generateCodeVerifier();
const codeChallenge = await generateCodeChallenge(codeVerifier);

// Redirect to SSO
const url = await oauthClient.createAuthorizationURL({
  state,
  codeVerifier,
  scopes: ["openid", "profile", "email"],
});
```

### 1.2 Session Management

**Options**:
1. **Better Auth client** (similar to SSO) - full auth solution
2. **jose** (JWT library) - manual JWT verification
3. **iron-session** - encrypted session cookies

**Recommendation**: Use **jose** for JWT verification + **iron-session** for encrypted session storage.

**Why**:
- SSO already handles authentication
- Hackathon app just needs to verify JWT and create local session
- iron-session provides secure, encrypted cookies (no Redis needed for MVP)

```typescript
// JWT verification with jose
import { jwtVerify } from "jose";

const JWKS_URL = "http://localhost:3001/.well-known/jwks.json";
const JWKS = createRemoteJWKSet(new URL(JWKS_URL));

async function verifyToken(token: string) {
  const { payload } = await jwtVerify(token, JWKS, {
    issuer: "http://localhost:3001",
    audience: "hackathon-public-client",
  });
  return payload;
}
```

### 1.3 Next.js 16 Patterns

**Async Request APIs** (Breaking change from Next.js 14):
```typescript
// Next.js 16: params and searchParams are async
async function Page({ params }: { params: Promise<{ id: string }> }) {
  const { id } = await params;
}
```

**Server Actions** (Recommended for mutations):
```typescript
"use server";

export async function createHackathon(formData: FormData) {
  const session = await getSession();
  if (!session) throw new Error("Unauthorized");

  // Validate + insert into DB
  await db.insert(hackathons).values({ ... });
}
```

**Middleware for Auth** (Protect routes):
```typescript
// middleware.ts
export async function middleware(request: NextRequest) {
  const session = await getSession(request);

  if (!session && request.nextUrl.pathname.startsWith("/dashboard")) {
    return NextResponse.redirect(new URL("/login", request.url));
  }
}
```

---

## 2. Database Schema Design (Drizzle ORM)

### 2.1 Multi-tenant Architecture

**Key Decision**: How to scope data to organizations?

**Option 1: organization_id in every table** (Recommended)
```typescript
export const hackathons = pgTable("hackathons", {
  id: text("id").primaryKey(),
  organizationId: text("organization_id").notNull()
    .references(() => organizations.id),
  // ... other fields
});
```

**Option 2: Separate schema per organization**
- More complex, better isolation
- Not needed for MVP

**Recommendation**: Use Option 1 (organization_id column) for simplicity.

### 2.2 Role-Based Access Control

**Approach**: Per-hackathon roles stored in junction table

```typescript
export const hackathonRoles = pgTable("hackathon_roles", {
  id: text("id").primaryKey(),
  userId: text("user_id").notNull().references(() => users.id),
  hackathonId: text("hackathon_id").notNull().references(() => hackathons.id),
  role: text("role").notNull(), // organizer, manager, judge, mentor, participant
  createdAt: timestamp("created_at").defaultNow().notNull(),
});
```

**Query Pattern**:
```typescript
// Check if user is organizer for hackathon
const role = await db
  .select()
  .from(hackathonRoles)
  .where(
    and(
      eq(hackathonRoles.userId, session.userId),
      eq(hackathonRoles.hackathonId, hackathonId),
      eq(hackathonRoles.role, "organizer")
    )
  );
```

### 2.3 Invite Code Generation

**Approach**: Short, collision-resistant codes

```typescript
import { customAlphabet } from "nanoid";

// Generate 8-char alphanumeric code (no ambiguous chars)
const nanoid = customAlphabet("23456789ABCDEFGHJKLMNPQRSTUVWXYZ", 8);

export const teams = pgTable("teams", {
  id: text("id").primaryKey(),
  hackathonId: text("hackathon_id").notNull(),
  name: text("name").notNull(),
  inviteCode: text("invite_code").notNull().unique(), // e.g., "A3K7P2M9"
  leaderId: text("leader_id").notNull().references(() => users.id),
});
```

### 2.4 Judging Criteria Storage

**Option 1: JSON in hackathons table**
```typescript
export const hackathons = pgTable("hackathons", {
  judgingCriteria: text("judging_criteria"), // JSON: [{ name, weight, description }]
});
```

**Option 2: Separate table** (Recommended for queryability)
```typescript
export const judgingCriteria = pgTable("judging_criteria", {
  id: text("id").primaryKey(),
  hackathonId: text("hackathon_id").notNull(),
  name: text("name").notNull(), // e.g., "Innovation"
  weight: integer("weight").default(1), // Scoring weight
  description: text("description"),
  order: integer("order").notNull(), // Display order
});
```

**Recommendation**: Use Option 2 for better normalization and query flexibility.

### 2.5 Score Storage

**Approach**: Individual scores per judge per criterion

```typescript
export const scores = pgTable("scores", {
  id: text("id").primaryKey(),
  submissionId: text("submission_id").notNull(),
  judgeId: text("judge_id").notNull(),
  criterionId: text("criterion_id").notNull(),
  score: integer("score").notNull(), // 1-10
  feedback: text("feedback"), // Optional written feedback
  createdAt: timestamp("created_at").defaultNow().notNull(),
});

// Composite index for efficient queries
export const scoresIndex = index("scores_submission_judge_idx")
  .on(scores.submissionId, scores.judgeId);
```

**Aggregate Calculation**:
```sql
-- Calculate average score per submission
SELECT
  submission_id,
  AVG(score) as avg_score
FROM scores
GROUP BY submission_id
ORDER BY avg_score DESC;
```

---

## 3. API Route Patterns

### 3.1 Route Organization

```
apps/hackathon/src/app/api/
├── auth/
│   ├── login/route.ts           # Redirect to SSO /oauth2/authorize
│   ├── callback/route.ts        # Handle OAuth callback, exchange code for token
│   └── logout/route.ts          # Clear session, redirect to SSO logout
├── hackathons/
│   ├── route.ts                 # GET (list), POST (create)
│   └── [id]/
│       ├── route.ts             # GET (details), PATCH (update)
│       ├── teams/route.ts       # GET teams for hackathon
│       ├── roles/route.ts       # POST (assign role), DELETE (revoke)
│       └── submissions/route.ts # GET submissions for judging
├── teams/
│   ├── route.ts                 # POST (create team)
│   ├── [id]/route.ts            # GET (team details), PATCH (update)
│   ├── [id]/members/route.ts    # POST (add member), DELETE (remove)
│   └── join/route.ts            # POST (join via invite code)
├── submissions/
│   ├── route.ts                 # POST (submit project)
│   └── [id]/route.ts            # GET (details), PATCH (update)
└── scores/
    ├── route.ts                 # POST (submit score)
    └── [submissionId]/route.ts  # GET (all scores for submission)
```

### 3.2 Authorization Pattern

**Middleware Approach** (Shared logic):
```typescript
// lib/auth/require-role.ts
export async function requireRole(
  hackathonId: string,
  allowedRoles: string[]
): Promise<User> {
  const session = await getSession();
  if (!session) throw new UnauthorizedError();

  const role = await db
    .select()
    .from(hackathonRoles)
    .where(
      and(
        eq(hackathonRoles.userId, session.userId),
        eq(hackathonRoles.hackathonId, hackathonId)
      )
    )
    .limit(1);

  if (!role.length || !allowedRoles.includes(role[0].role)) {
    throw new ForbiddenError();
  }

  return session.user;
}

// Usage in route
export async function POST(request: Request) {
  const user = await requireRole(hackathonId, ["organizer", "manager"]);
  // ... handle request
}
```

---

## 4. Technology Stack Summary

### 4.1 Core Dependencies

```json
{
  "dependencies": {
    "next": "^16.0.0",
    "react": "^19.0.0",
    "react-dom": "^19.0.0",
    "drizzle-orm": "^0.36.0",
    "@neondatabase/serverless": "^0.10.0",
    "arctic": "^2.0.0", // OAuth client
    "jose": "^6.1.0", // JWT verification
    "iron-session": "^8.0.0", // Encrypted sessions
    "nanoid": "^5.0.0", // Invite code generation
    "zod": "^3.23.0" // Validation
  },
  "devDependencies": {
    "drizzle-kit": "^0.28.0",
    "@types/node": "^22.0.0",
    "typescript": "^5.6.0"
  }
}
```

### 4.2 Database

- **Neon Postgres**: Serverless PostgreSQL (shared instance with SSO or separate)
- **Drizzle ORM**: Type-safe schema and queries
- **Connection Pooling**: @neondatabase/serverless with connection pooling enabled

### 4.3 UI Framework

**Option 1**: shadcn/ui (recommended - consistent with SSO)
**Option 2**: Tailwind CSS only
**Option 3**: Radix UI primitives

**Recommendation**: Use **shadcn/ui** for consistency with SSO app.

---

## 5. Performance Considerations

### 5.1 Caching Strategy

**Static Pages** (Next.js Cache Components):
- Hackathon catalog (revalidate every 60s)
- Public hackathon details (revalidate on submission)

**Dynamic Pages**:
- Dashboard (user-specific, no cache)
- Team pages (private, no cache)

**Database Query Optimization**:
```typescript
// Use indexes for frequent queries
export const hackathonsOrgIndex = index("hackathons_org_idx")
  .on(hackathons.organizationId);

export const teamsHackathonIndex = index("teams_hackathon_idx")
  .on(teams.hackathonId);

export const rolesUserHackathonIndex = index("roles_user_hackathon_idx")
  .on(hackathonRoles.userId, hackathonRoles.hackathonId);
```

### 5.2 Concurrent Load Handling

**Expected Load** (from spec SC-004):
- 500 concurrent participants during registration peak

**Mitigation**:
1. **Connection pooling**: Neon serverless handles this automatically
2. **Rate limiting**: Apply to registration endpoints (10 req/min per IP)
3. **Optimistic UI**: Show success immediately, handle errors async

---

## 6. Security Considerations

### 6.1 CSRF Protection

**Approach**: Use iron-session's built-in CSRF tokens

```typescript
import { getIronSession } from "iron-session";

export async function POST(request: Request) {
  const session = await getIronSession(request);

  // Validate CSRF token from form
  const formData = await request.formData();
  const token = formData.get("csrf_token");

  if (token !== session.csrfToken) {
    throw new Error("Invalid CSRF token");
  }
}
```

### 6.2 Multi-tenant Isolation

**Enforce at Query Level**:
```typescript
// ALWAYS include organization_id in WHERE clause
const hackathons = await db
  .select()
  .from(hackathons)
  .where(
    and(
      eq(hackathons.organizationId, session.organizationId),
      eq(hackathons.status, "active")
    )
  );
```

**Validation**: Add integration tests to ensure no cross-org data leaks.

### 6.3 Input Validation

**Use Zod schemas**:
```typescript
import { z } from "zod";

const createHackathonSchema = z.object({
  title: z.string().min(3).max(100),
  description: z.string().max(2000),
  startDate: z.coerce.date(),
  endDate: z.coerce.date(),
  registrationDeadline: z.coerce.date(),
  minTeamSize: z.number().min(1).max(10),
  maxTeamSize: z.number().min(1).max(10),
});

export async function POST(request: Request) {
  const body = await request.json();
  const data = createHackathonSchema.parse(body); // Throws if invalid
}
```

---

## 7. Testing Strategy

### 7.1 Unit Tests

**Focus Areas**:
- JWT verification logic
- Invite code generation (collision testing)
- Score calculation (aggregation logic)

### 7.2 Integration Tests

**Critical Flows**:
1. OAuth login flow (login → callback → session creation)
2. Hackathon creation (organizer creates → participants see)
3. Team formation (create → invite → join → verify size limits)
4. Submission (team lead submits → judges see → scoring)
5. Multi-tenant isolation (org A cannot see org B hackathons)

### 7.3 E2E Tests (Playwright)

**User Journeys**:
- Organizer: Create hackathon → Assign judges → View results
- Participant: Browse → Register → Create team → Submit project
- Judge: Login → View submissions → Score → Submit feedback

---

## 8. Deployment Considerations

### 8.1 Environment Variables

```env
# OAuth Configuration
SSO_OAUTH_CLIENT_ID=hackathon-public-client
SSO_OAUTH_ISSUER=https://sso.panaversity.org
SSO_OAUTH_AUTHORIZE_URL=https://sso.panaversity.org/oauth2/authorize
SSO_OAUTH_TOKEN_URL=https://sso.panaversity.org/oauth2/token
SSO_OAUTH_USERINFO_URL=https://sso.panaversity.org/oauth2/userinfo
SSO_OAUTH_JWKS_URL=https://sso.panaversity.org/.well-known/jwks.json
SSO_OAUTH_REDIRECT_URI=https://hackathon.panaversity.org/api/auth/callback

# Database
DATABASE_URL=postgresql://...

# Session Encryption
SESSION_SECRET=<generated-32-byte-secret>

# App Configuration
NEXT_PUBLIC_APP_NAME=Hackathon Platform
NEXT_PUBLIC_APP_URL=https://hackathon.panaversity.org
```

### 8.2 Cloud Run Deployment

**Dockerfile** (from SSO pattern):
```dockerfile
FROM node:20-alpine AS base
# Install dependencies
WORKDIR /app
COPY package*.json ./
RUN npm ci --production

# Build app
COPY . .
RUN npm run build

# Run
EXPOSE 3002
CMD ["npm", "start"]
```

**Cloud Run Settings**:
- Min instances: 0 (scale to zero for cost savings)
- Max instances: 10
- Concurrency: 80 (Next.js default)
- CPU: 1
- Memory: 512MB

---

## 9. Open Questions & Decisions Needed

### Q1: Database Strategy
**Question**: Use shared Neon instance with SSO or separate database?

**Options**:
1. **Shared instance, separate schema**: `hackathon.*` vs `auth.*`
2. **Shared instance, same schema**: All tables in default schema
3. **Separate Neon project**: Isolated database

**Recommendation**: Option 2 (same schema) for simplicity. User data already in SSO DB via JWT claims.

### Q2: Notification Strategy
**Question**: How to notify users of team invites, judging assignments?

**Options**:
1. **Email notifications**: Use Resend (same as SSO)
2. **In-app notifications**: Bell icon with unread count
3. **Manual checks only** (MVP)

**Recommendation**: Option 3 for MVP, Option 2 for Phase 2.

### Q3: File Upload for Submissions
**Question**: Where to store presentation files?

**Options**:
1. **Vercel Blob**: Integrated with Next.js, 1GB free
2. **Cloudflare R2**: S3-compatible, generous free tier
3. **Links only** (GitHub, YouTube, etc.)

**Recommendation**: Option 3 for MVP (links only), Option 1 for Phase 2.

---

## 10. References

### Documentation
- [Next.js 16 Async APIs](https://nextjs.org/docs/app/building-your-application/upgrading/version-16)
- [Better Auth OAuth Client](https://better-auth.com/docs/oauth2-client)
- [Drizzle ORM Docs](https://orm.drizzle.team/docs/overview)
- [Arctic OAuth Library](https://arctic.js.org/)
- [iron-session](https://github.com/vvo/iron-session)

### Similar Implementations
- apps/sso (OAuth provider reference)
- apps/assessment (OAuth client reference - if exists)

### Security Best Practices
- [OWASP OAuth 2.0 Security](https://owasp.org/www-community/vulnerabilities/OAuth_2_0_Security)
- [PKCE Flow Diagram](https://datatracker.ietf.org/doc/html/rfc7636)

---

**Next Steps**: Use this research to design detailed plan, data model, and API contracts.
