# Implementation Plan: Hackathon Platform

**Feature**: 046-hackathon-platform
**Branch**: `hackathon-platform`
**Target**: Production-ready MVP
**Estimated Timeline**: 8 phases, ~40-50 hours

---

## Executive Summary

Build a Next.js 16 hackathon platform that integrates with existing Panaversity SSO as an OAuth2 public client. The platform supports multi-tenant hackathon management with roles (organizer, manager, judge, mentor, participant), team formation, project submissions, and judging workflows.

**Key Success Metrics** (from spec):
- ✅ SC-001: Hackathon creation in <30 seconds
- ✅ SC-002: Registration + team formation in <3 minutes
- ✅ SC-003: Judging a submission in <5 minutes
- ✅ SC-004: Support 500 concurrent users
- ✅ SC-005: 95% task completion on first attempt
- ✅ SC-006: Page loads <2 seconds
- ✅ SC-007: Zero cross-org data leaks

---

## Phase 0: Prerequisites & Setup

**Duration**: 2-3 hours
**Goal**: Scaffold app structure, configure dependencies, establish dev environment

### Tasks

#### 0.1 Create Next.js 16 App in Monorepo

```bash
# In monorepo root
npx nx generate @nx/next:application hackathon \
  --directory=apps/hackathon \
  --style=css \
  --appDir=true \
  --importPath=@h-app/hackathon

# Configure port in apps/hackathon/project.json
{
  "targets": {
    "serve": {
      "options": {
        "port": 3002
      }
    }
  }
}
```

**Files Created**:
- `apps/hackathon/` (Next.js 16 app)
- `apps/hackathon/project.json` (Nx configuration)
- `apps/hackathon/next.config.ts` (Next.js config)
- `apps/hackathon/tsconfig.json`

#### 0.2 Install Dependencies

```bash
cd apps/hackathon

# Core dependencies
pnpm add next@^16.0.0 react@^19.0.0 react-dom@^19.0.0
pnpm add drizzle-orm@^0.36.0 @neondatabase/serverless@^0.10.0
pnpm add arctic@^2.0.0 jose@^6.1.0 iron-session@^8.0.0
pnpm add nanoid@^5.0.0 zod@^3.23.0

# UI components (shadcn/ui pattern)
pnpm add @radix-ui/react-alert-dialog @radix-ui/react-dialog
pnpm add @radix-ui/react-dropdown-menu @radix-ui/react-label
pnpm add @radix-ui/react-slot @radix-ui/react-tooltip
pnpm add lucide-react clsx tailwind-merge class-variance-authority

# Dev dependencies
pnpm add -D drizzle-kit@^0.28.0 tsx@^4.21.0
pnpm add -D @types/node@^22.0.0 typescript@^5.6.0
pnpm add -D tailwindcss@^3.4.0 autoprefixer@^10.4.0 postcss@^8.4.0
```

**Package.json scripts**:
```json
{
  "scripts": {
    "dev": "next dev -p 3002",
    "build": "next build",
    "start": "next start -p 3002",
    "lint": "next lint",
    "db:generate": "drizzle-kit generate",
    "db:push": "drizzle-kit push",
    "db:studio": "drizzle-kit studio",
    "seed:dev": "npx tsx scripts/seed-dev.ts"
  }
}
```

#### 0.3 Directory Structure

Create the following structure:

```
apps/hackathon/
├── src/
│   ├── app/
│   │   ├── (auth)/           # Auth layout group
│   │   │   ├── login/
│   │   │   └── callback/
│   │   ├── (dashboard)/       # Authenticated layout group
│   │   │   ├── dashboard/
│   │   │   ├── hackathons/
│   │   │   └── teams/
│   │   ├── api/
│   │   │   ├── auth/
│   │   │   ├── hackathons/
│   │   │   ├── teams/
│   │   │   ├── submissions/
│   │   │   └── scores/
│   │   ├── layout.tsx
│   │   └── page.tsx
│   ├── components/
│   │   ├── ui/               # shadcn components
│   │   ├── hackathons/
│   │   ├── teams/
│   │   └── layout/
│   ├── db/
│   │   ├── schema.ts         # Drizzle schema (from data-model.md)
│   │   ├── index.ts          # DB connection
│   │   └── queries/          # Reusable queries
│   ├── lib/
│   │   ├── auth/             # OAuth & session logic
│   │   ├── validation/       # Zod schemas
│   │   └── utils.ts
│   └── types/
│       └── index.ts
├── scripts/
│   └── seed-dev.ts
├── drizzle/                  # Migrations
├── .env.local.example
├── drizzle.config.ts
├── middleware.ts             # Auth middleware
├── next.config.ts
├── tailwind.config.ts
└── tsconfig.json
```

**Create with**:
```bash
mkdir -p src/{app/{api/{auth,hackathons,teams,submissions,scores},'(auth)/{login,callback}','(dashboard)/{dashboard,hackathons,teams}'},components/{ui,hackathons,teams,layout},db/queries,lib/{auth,validation},types}
mkdir -p scripts drizzle
```

#### 0.4 Environment Configuration

**Create `.env.local.example`**:
```env
# Database
DATABASE_URL=postgresql://user:password@ep-xxx.us-east-2.aws.neon.tech/hackathon?sslmode=require

# OAuth Configuration (SSO Integration)
SSO_OAUTH_CLIENT_ID=hackathon-public-client
SSO_OAUTH_ISSUER=http://localhost:3001
SSO_OAUTH_AUTHORIZE_URL=http://localhost:3001/oauth2/authorize
SSO_OAUTH_TOKEN_URL=http://localhost:3001/oauth2/token
SSO_OAUTH_USERINFO_URL=http://localhost:3001/oauth2/userinfo
SSO_OAUTH_JWKS_URL=http://localhost:3001/.well-known/jwks.json
SSO_OAUTH_REDIRECT_URI=http://localhost:3002/api/auth/callback

# Session Encryption
SESSION_SECRET=<generate-with: openssl rand -base64 32>

# App Configuration
NEXT_PUBLIC_APP_NAME=Hackathon Platform
NEXT_PUBLIC_APP_URL=http://localhost:3002
NEXT_PUBLIC_SSO_URL=http://localhost:3001

# Production overrides
# SSO_OAUTH_ISSUER=https://sso.panaversity.org
# SSO_OAUTH_REDIRECT_URI=https://hackathon.panaversity.org/api/auth/callback
# NEXT_PUBLIC_APP_URL=https://hackathon.panaversity.org
```

**User Action Required**: Create `.env.local` from template and fill in values.

#### 0.5 Tailwind & UI Setup

**tailwind.config.ts**:
```typescript
import type { Config } from "tailwindcss";

const config: Config = {
  content: [
    "./src/pages/**/*.{js,ts,jsx,tsx,mdx}",
    "./src/components/**/*.{js,ts,jsx,tsx,mdx}",
    "./src/app/**/*.{js,ts,jsx,tsx,mdx}",
  ],
  theme: {
    extend: {
      colors: {
        primary: {
          50: "#f0f9ff",
          500: "#2563eb",
          600: "#1d4ed8",
        },
      },
    },
  },
  plugins: [],
};
export default config;
```

**Setup shadcn/ui**:
```bash
# Initialize shadcn
npx shadcn@latest init

# Add essential components
npx shadcn@latest add button card dialog dropdown-menu input label table
```

### Validation

- ✅ `pnpm dev` starts app on port 3002
- ✅ Environment variables load correctly
- ✅ Tailwind CSS compiles
- ✅ TypeScript has no errors

---

## Phase 1: Database Schema & SSO Integration

**Duration**: 4-5 hours
**Goal**: Set up database, implement OAuth flow, establish session management

### 1.1 Database Schema Implementation

**Task**: Create Drizzle schema from `data-model.md`

**File**: `src/db/schema.ts`

```typescript
import { pgTable, text, timestamp, integer, boolean, index, unique } from "drizzle-orm/pg-core";
import { relations } from "drizzle-orm";
import { customAlphabet } from "nanoid";

// Invite code generator
const generateInviteCode = customAlphabet("23456789ABCDEFGHJKLMNPQRSTUVWXYZ", 8);

// Copy all table definitions from data-model.md
export const hackathons = pgTable("hackathons", { /* ... */ });
export const judgingCriteria = pgTable("judging_criteria", { /* ... */ });
export const hackathonRoles = pgTable("hackathon_roles", { /* ... */ });
export const teams = pgTable("teams", { /* ... */ });
export const teamMembers = pgTable("team_members", { /* ... */ });
export const submissions = pgTable("submissions", { /* ... */ });
export const scores = pgTable("scores", { /* ... */ });
export const teamMessages = pgTable("team_messages", { /* ... */ });

// Relations (for Drizzle query builder)
// ... (copy from data-model.md)
```

**File**: `drizzle.config.ts`

```typescript
import { defineConfig } from "drizzle-kit";
import * as dotenv from "dotenv";

dotenv.config({ path: ".env.local" });

export default defineConfig({
  schema: "./src/db/schema.ts",
  out: "./drizzle",
  dialect: "postgresql",
  dbCredentials: {
    url: process.env.DATABASE_URL!,
  },
});
```

**File**: `src/db/index.ts`

```typescript
import { drizzle } from "drizzle-orm/neon-serverless";
import { Pool } from "@neondatabase/serverless";
import * as schema from "./schema";

if (!process.env.DATABASE_URL) {
  throw new Error("DATABASE_URL is not set");
}

const pool = new Pool({ connectionString: process.env.DATABASE_URL });
export const db = drizzle(pool, { schema });
```

**Run migrations**:
```bash
pnpm db:generate  # Generate SQL migrations
pnpm db:push      # Apply to database
```

### 1.2 OAuth Client Implementation

**File**: `src/lib/auth/oauth-client.ts`

```typescript
import { OAuth2Client } from "arctic";

const clientId = process.env.SSO_OAUTH_CLIENT_ID!;
const authorizeUrl = process.env.SSO_OAUTH_AUTHORIZE_URL!;
const tokenUrl = process.env.SSO_OAUTH_TOKEN_URL!;
const redirectUri = process.env.SSO_OAUTH_REDIRECT_URI!;

export const oauthClient = new OAuth2Client({
  clientId,
  clientSecret: null, // Public client
  authorizationEndpoint: authorizeUrl,
  tokenEndpoint: tokenUrl,
  redirectURI: redirectUri,
});

// PKCE utilities
export { generateState, generateCodeVerifier, generateCodeChallenge } from "arctic";
```

**File**: `src/lib/auth/session.ts`

```typescript
import { getIronSession } from "iron-session";
import { cookies } from "next/headers";

export interface SessionData {
  userId: string;
  email: string;
  name: string;
  organizationId: string;
  role: string; // SSO role (admin/user)
  accessToken: string;
  refreshToken?: string;
  expiresAt: number;
}

const sessionOptions = {
  password: process.env.SESSION_SECRET!,
  cookieName: "hackathon_session",
  cookieOptions: {
    secure: process.env.NODE_ENV === "production",
    httpOnly: true,
    sameSite: "lax" as const,
    maxAge: 60 * 60 * 24 * 7, // 7 days
  },
};

export async function getSession() {
  const cookieStore = await cookies();
  return getIronSession<SessionData>(cookieStore, sessionOptions);
}

export async function createSession(data: SessionData) {
  const session = await getSession();
  Object.assign(session, data);
  await session.save();
}

export async function destroySession() {
  const session = await getSession();
  session.destroy();
}
```

**File**: `src/lib/auth/jwt-verify.ts`

```typescript
import { jwtVerify, createRemoteJWKSet } from "jose";

const JWKS_URL = process.env.SSO_OAUTH_JWKS_URL!;
const ISSUER = process.env.SSO_OAUTH_ISSUER!;
const CLIENT_ID = process.env.SSO_OAUTH_CLIENT_ID!;

const JWKS = createRemoteJWKSet(new URL(JWKS_URL));

export async function verifyAccessToken(token: string) {
  const { payload } = await jwtVerify(token, JWKS, {
    issuer: ISSUER,
    audience: CLIENT_ID,
  });

  return payload as {
    sub: string;
    email: string;
    name: string;
    tenant_id: string;
    role: string;
    exp: number;
  };
}
```

### 1.3 OAuth Routes

**File**: `src/app/api/auth/login/route.ts`

```typescript
import { NextRequest, NextResponse } from "next/server";
import { oauthClient, generateState, generateCodeVerifier, generateCodeChallenge } from "@/lib/auth/oauth-client";
import { getSession } from "@/lib/auth/session";

export async function GET(request: NextRequest) {
  const state = generateState();
  const codeVerifier = generateCodeVerifier();
  const codeChallenge = await generateCodeChallenge(codeVerifier);

  // Store state and verifier in session for callback verification
  const session = await getSession();
  session.oauthState = state;
  session.codeVerifier = codeVerifier;
  await session.save();

  const url = await oauthClient.createAuthorizationURL({
    state,
    codeVerifier,
    scopes: ["openid", "profile", "email"],
  });

  return NextResponse.redirect(url);
}
```

**File**: `src/app/api/auth/callback/route.ts`

```typescript
import { NextRequest, NextResponse } from "next/server";
import { oauthClient } from "@/lib/auth/oauth-client";
import { getSession, createSession } from "@/lib/auth/session";
import { verifyAccessToken } from "@/lib/auth/jwt-verify";

export async function GET(request: NextRequest) {
  const { searchParams } = new URL(request.url);
  const code = searchParams.get("code");
  const state = searchParams.get("state");

  // Verify state matches
  const session = await getSession();
  if (!state || state !== session.oauthState) {
    return NextResponse.redirect(new URL("/login?error=invalid_state", request.url));
  }

  if (!code) {
    return NextResponse.redirect(new URL("/login?error=no_code", request.url));
  }

  try {
    // Exchange code for token
    const tokens = await oauthClient.validateAuthorizationCode(code, session.codeVerifier!);

    // Verify JWT and extract claims
    const claims = await verifyAccessToken(tokens.accessToken);

    // Create session
    await createSession({
      userId: claims.sub,
      email: claims.email,
      name: claims.name,
      organizationId: claims.tenant_id,
      role: claims.role,
      accessToken: tokens.accessToken,
      refreshToken: tokens.refreshToken,
      expiresAt: claims.exp * 1000, // Convert to ms
    });

    // Clear OAuth temp data
    delete session.oauthState;
    delete session.codeVerifier;
    await session.save();

    return NextResponse.redirect(new URL("/dashboard", request.url));
  } catch (error) {
    console.error("OAuth callback error:", error);
    return NextResponse.redirect(new URL("/login?error=auth_failed", request.url));
  }
}
```

**File**: `src/app/api/auth/logout/route.ts`

```typescript
import { NextRequest, NextResponse } from "next/server";
import { destroySession } from "@/lib/auth/session";

export async function POST(request: NextRequest) {
  await destroySession();

  // Redirect to SSO logout
  const ssoLogoutUrl = `${process.env.SSO_OAUTH_ISSUER}/api/auth/oauth2/endsession`;
  return NextResponse.redirect(ssoLogoutUrl);
}
```

### 1.4 Auth Middleware

**File**: `middleware.ts`

```typescript
import { NextRequest, NextResponse } from "next/server";
import { getSession } from "@/lib/auth/session";

export async function middleware(request: NextRequest) {
  const pathname = request.nextUrl.pathname;

  // Public routes
  if (pathname === "/" || pathname.startsWith("/api/auth") || pathname === "/login") {
    return NextResponse.next();
  }

  // Protected routes
  const session = await getSession();
  if (!session.userId) {
    return NextResponse.redirect(new URL("/login", request.url));
  }

  // Check token expiry
  if (Date.now() > session.expiresAt) {
    await session.destroy();
    return NextResponse.redirect(new URL("/login?error=expired", request.url));
  }

  return NextResponse.next();
}

export const config = {
  matcher: [
    "/((?!_next/static|_next/image|favicon.ico).*)",
  ],
};
```

### 1.5 Login Page

**File**: `src/app/(auth)/login/page.tsx`

```typescript
import { redirect } from "next/navigation";
import { getSession } from "@/lib/auth/session";

export default async function LoginPage() {
  const session = await getSession();

  // Redirect if already logged in
  if (session.userId) {
    redirect("/dashboard");
  }

  return (
    <div className="flex min-h-screen items-center justify-center bg-gray-50">
      <div className="w-full max-w-md space-y-8 rounded-lg bg-white p-8 shadow">
        <div className="text-center">
          <h2 className="text-3xl font-bold">Hackathon Platform</h2>
          <p className="mt-2 text-gray-600">
            Sign in with your Panaversity account
          </p>
        </div>

        <a
          href="/api/auth/login"
          className="block w-full rounded-lg bg-blue-600 px-4 py-3 text-center font-semibold text-white hover:bg-blue-700"
        >
          Sign in with SSO
        </a>
      </div>
    </div>
  );
}
```

### Validation

- ✅ Database schema created and migrated
- ✅ OAuth login flow redirects to SSO
- ✅ Callback successfully exchanges code for token
- ✅ Session created with user data from JWT
- ✅ Middleware protects dashboard routes
- ✅ Logout clears session and redirects to SSO

---

## Phase 2: Core Entities & Queries

**Duration**: 3-4 hours
**Goal**: Implement database query utilities and validation schemas

### 2.1 Validation Schemas

**File**: `src/lib/validation/hackathon.ts`

```typescript
import { z } from "zod";

export const createHackathonSchema = z.object({
  title: z.string().min(3, "Title must be at least 3 characters").max(100),
  description: z.string().min(10).max(2000),
  slug: z.string().min(3).max(50).regex(/^[a-z0-9-]+$/),
  startDate: z.coerce.date(),
  endDate: z.coerce.date(),
  registrationDeadline: z.coerce.date(),
  submissionDeadline: z.coerce.date(),
  minTeamSize: z.number().min(1).max(10),
  maxTeamSize: z.number().min(1).max(10),
}).refine(
  (data) => data.endDate > data.startDate,
  { message: "End date must be after start date", path: ["endDate"] }
).refine(
  (data) => data.registrationDeadline < data.startDate,
  { message: "Registration deadline must be before start date", path: ["registrationDeadline"] }
).refine(
  (data) => data.maxTeamSize >= data.minTeamSize,
  { message: "Max team size must be >= min team size", path: ["maxTeamSize"] }
);

export const updateHackathonSchema = createHackathonSchema.partial();
```

**File**: `src/lib/validation/team.ts`

```typescript
import { z } from "zod";

export const createTeamSchema = z.object({
  hackathonId: z.string().uuid(),
  name: z.string().min(3).max(50),
  description: z.string().max(500).optional(),
});

export const joinTeamSchema = z.object({
  inviteCode: z.string().length(8),
});
```

**File**: `src/lib/validation/submission.ts`

```typescript
import { z } from "zod";

export const createSubmissionSchema = z.object({
  teamId: z.string().uuid(),
  hackathonId: z.string().uuid(),
  projectName: z.string().min(3).max(100),
  description: z.string().min(10).max(2000),
  repositoryUrl: z.string().url(),
  demoUrl: z.string().url().optional(),
  presentationUrl: z.string().url().optional(),
});

export const scoreSubmissionSchema = z.object({
  submissionId: z.string().uuid(),
  criterionId: z.string().uuid(),
  score: z.number().min(1).max(10),
  feedback: z.string().max(1000).optional(),
});
```

### 2.2 Database Query Utilities

**File**: `src/db/queries/hackathons.ts`

```typescript
import { db } from "..";
import { hackathons, hackathonRoles, judgingCriteria } from "../schema";
import { eq, and } from "drizzle-orm";

export async function getHackathonsByOrg(organizationId: string) {
  return await db.query.hackathons.findMany({
    where: and(
      eq(hackathons.organizationId, organizationId),
      eq(hackathons.published, true)
    ),
    orderBy: (hackathons, { desc }) => [desc(hackathons.createdAt)],
  });
}

export async function getHackathonById(id: string) {
  return await db.query.hackathons.findFirst({
    where: eq(hackathons.id, id),
    with: {
      criteria: true,
    },
  });
}

export async function createHackathon(data: typeof hackathons.$inferInsert) {
  const [hackathon] = await db.insert(hackathons).values(data).returning();

  // Create default judging criteria
  await db.insert(judgingCriteria).values([
    {
      hackathonId: hackathon.id,
      name: "Innovation",
      description: "Originality and creativity of the solution",
      weight: 2,
      maxScore: 10,
      order: 1,
    },
    {
      hackathonId: hackathon.id,
      name: "Technical Execution",
      description: "Code quality, architecture, and implementation",
      weight: 2,
      maxScore: 10,
      order: 2,
    },
    {
      hackathonId: hackathon.id,
      name: "Impact",
      description: "Potential real-world usefulness",
      weight: 1,
      maxScore: 10,
      order: 3,
    },
    {
      hackathonId: hackathon.id,
      name: "Presentation",
      description: "Demo quality and communication",
      weight: 1,
      maxScore: 10,
      order: 4,
    },
  ]);

  // Assign creator as organizer
  await db.insert(hackathonRoles).values({
    hackathonId: hackathon.id,
    userId: data.createdBy,
    role: "organizer",
    assignedBy: data.createdBy,
  });

  return hackathon;
}

export async function getUserRole(userId: string, hackathonId: string) {
  const role = await db.query.hackathonRoles.findFirst({
    where: and(
      eq(hackathonRoles.userId, userId),
      eq(hackathonRoles.hackathonId, hackathonId)
    ),
  });

  return role?.role ?? null;
}
```

**File**: `src/db/queries/teams.ts`

```typescript
import { db } from "..";
import { teams, teamMembers } from "../schema";
import { eq, and, count } from "drizzle-orm";

export async function getTeamsByHackathon(hackathonId: string) {
  return await db.query.teams.findMany({
    where: eq(teams.hackathonId, hackathonId),
    with: {
      members: true,
    },
  });
}

export async function getTeamById(teamId: string) {
  return await db.query.teams.findFirst({
    where: eq(teams.id, teamId),
    with: {
      members: true,
      submission: true,
    },
  });
}

export async function createTeam(data: typeof teams.$inferInsert, leaderId: string) {
  const [team] = await db.insert(teams).values(data).returning();

  // Add leader as first member
  await db.insert(teamMembers).values({
    teamId: team.id,
    userId: leaderId,
    role: "leader",
  });

  return team;
}

export async function joinTeamByInviteCode(inviteCode: string, userId: string) {
  // Find team
  const team = await db.query.teams.findFirst({
    where: eq(teams.inviteCode, inviteCode),
    with: {
      members: true,
    },
  });

  if (!team) throw new Error("Invalid invite code");

  // Check if user already in team
  const existingMember = team.members.find((m) => m.userId === userId);
  if (existingMember) throw new Error("Already a member");

  // Check team size limit
  const hackathon = await db.query.hackathons.findFirst({
    where: eq(hackathons.id, team.hackathonId),
  });

  if (team.members.length >= hackathon!.maxTeamSize) {
    throw new Error("Team is full");
  }

  // Add member
  await db.insert(teamMembers).values({
    teamId: team.id,
    userId,
    role: "member",
  });

  return team;
}

export async function getUserTeamForHackathon(userId: string, hackathonId: string) {
  // Find team via team_members join
  const result = await db
    .select({ team: teams })
    .from(teamMembers)
    .innerJoin(teams, eq(teamMembers.teamId, teams.id))
    .where(
      and(
        eq(teamMembers.userId, userId),
        eq(teams.hackathonId, hackathonId)
      )
    )
    .limit(1);

  return result[0]?.team ?? null;
}
```

**File**: `src/db/queries/submissions.ts`

```typescript
import { db } from "..";
import { submissions, scores } from "../schema";
import { eq, and, sql } from "drizzle-orm";

export async function getSubmissionsByHackathon(hackathonId: string) {
  return await db.query.submissions.findMany({
    where: eq(submissions.hackathonId, hackathonId),
    with: {
      team: true,
      scores: true,
    },
  });
}

export async function createSubmission(data: typeof submissions.$inferInsert) {
  const [submission] = await db.insert(submissions).values(data).returning();
  return submission;
}

export async function getLeaderboard(hackathonId: string) {
  // Complex aggregation query
  return await db.execute(sql`
    WITH weighted_scores AS (
      SELECT
        s.submission_id,
        s.judge_id,
        SUM(s.score * jc.weight) / NULLIF(SUM(jc.weight), 0) AS judge_score
      FROM ${scores} s
      JOIN ${judgingCriteria} jc ON s.criterion_id = jc.id
      JOIN ${submissions} sub ON s.submission_id = sub.id
      WHERE sub.hackathon_id = ${hackathonId}
      GROUP BY s.submission_id, s.judge_id
    )
    SELECT
      sub.id,
      sub.project_name,
      t.name AS team_name,
      AVG(ws.judge_score) AS final_score,
      COUNT(DISTINCT ws.judge_id) AS num_judges
    FROM weighted_scores ws
    JOIN ${submissions} sub ON ws.submission_id = sub.id
    JOIN ${teams} t ON sub.team_id = t.id
    GROUP BY sub.id, sub.project_name, t.name
    ORDER BY final_score DESC
  `);
}
```

### 2.3 Authorization Utilities

**File**: `src/lib/auth/permissions.ts`

```typescript
import { getUserRole } from "@/db/queries/hackathons";

export const ROLE_PERMISSIONS = {
  organizer: ["create_hackathon", "assign_roles", "edit_teams", "judge", "mentor"],
  manager: ["assign_roles", "edit_teams", "judge", "mentor"],
  judge: ["judge"],
  mentor: ["mentor"],
  participant: ["submit"],
};

export async function requireRole(
  userId: string,
  hackathonId: string,
  allowedRoles: string[]
) {
  const role = await getUserRole(userId, hackathonId);

  if (!role || !allowedRoles.includes(role)) {
    throw new Error("Unauthorized");
  }

  return role;
}

export async function can(
  userId: string,
  hackathonId: string,
  permission: string
): Promise<boolean> {
  const role = await getUserRole(userId, hackathonId);
  if (!role) return false;

  const permissions = ROLE_PERMISSIONS[role as keyof typeof ROLE_PERMISSIONS];
  return permissions?.includes(permission) ?? false;
}
```

### Validation

- ✅ Zod schemas validate inputs correctly
- ✅ Query functions return correct data types
- ✅ Multi-tenant queries scoped to organization
- ✅ Authorization checks prevent unauthorized access

---

## Phase 3: Hackathon Management (FR-005 to FR-008)

**Duration**: 5-6 hours
**Goal**: Implement organizer workflows for creating and managing hackathons

### 3.1 Hackathon CRUD API Routes

**File**: `src/app/api/hackathons/route.ts`

```typescript
import { NextRequest, NextResponse } from "next/server";
import { getSession } from "@/lib/auth/session";
import { createHackathonSchema } from "@/lib/validation/hackathon";
import { getHackathonsByOrg, createHackathon } from "@/db/queries/hackathons";

// GET /api/hackathons - List hackathons for user's organization
export async function GET(request: NextRequest) {
  const session = await getSession();
  if (!session.userId) {
    return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
  }

  const hackathons = await getHackathonsByOrg(session.organizationId);
  return NextResponse.json({ hackathons });
}

// POST /api/hackathons - Create new hackathon
export async function POST(request: NextRequest) {
  const session = await getSession();
  if (!session.userId) {
    return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
  }

  try {
    const body = await request.json();
    const data = createHackathonSchema.parse(body);

    const hackathon = await createHackathon({
      ...data,
      organizationId: session.organizationId,
      createdBy: session.userId,
      status: "draft",
      published: false,
    });

    return NextResponse.json({ hackathon }, { status: 201 });
  } catch (error) {
    if (error instanceof z.ZodError) {
      return NextResponse.json({ error: error.errors }, { status: 400 });
    }
    return NextResponse.json({ error: "Failed to create hackathon" }, { status: 500 });
  }
}
```

**File**: `src/app/api/hackathons/[id]/route.ts`

```typescript
import { NextRequest, NextResponse } from "next/server";
import { getSession } from "@/lib/auth/session";
import { requireRole } from "@/lib/auth/permissions";
import { getHackathonById } from "@/db/queries/hackathons";
import { updateHackathonSchema } from "@/lib/validation/hackathon";
import { db } from "@/db";
import { hackathons } from "@/db/schema";
import { eq } from "drizzle-orm";

// GET /api/hackathons/[id] - Get hackathon details
export async function GET(
  request: NextRequest,
  { params }: { params: Promise<{ id: string }> }
) {
  const { id } = await params;
  const hackathon = await getHackathonById(id);

  if (!hackathon) {
    return NextResponse.json({ error: "Not found" }, { status: 404 });
  }

  return NextResponse.json({ hackathon });
}

// PATCH /api/hackathons/[id] - Update hackathon
export async function PATCH(
  request: NextRequest,
  { params }: { params: Promise<{ id: string }> }
) {
  const { id } = await params;
  const session = await getSession();

  if (!session.userId) {
    return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
  }

  // Check permission
  await requireRole(session.userId, id, ["organizer", "manager"]);

  try {
    const body = await request.json();
    const data = updateHackathonSchema.parse(body);

    const [updated] = await db
      .update(hackathons)
      .set(data)
      .where(eq(hackathons.id, id))
      .returning();

    return NextResponse.json({ hackathon: updated });
  } catch (error) {
    if (error instanceof z.ZodError) {
      return NextResponse.json({ error: error.errors }, { status: 400 });
    }
    return NextResponse.json({ error: "Failed to update" }, { status: 500 });
  }
}
```

### 3.2 Hackathon Dashboard UI

**File**: `src/app/(dashboard)/dashboard/page.tsx`

```typescript
import { redirect } from "next/navigation";
import { getSession } from "@/lib/auth/session";
import { getHackathonsByOrg } from "@/db/queries/hackathons";
import { HackathonCard } from "@/components/hackathons/hackathon-card";
import { CreateHackathonButton } from "@/components/hackathons/create-button";

export default async function DashboardPage() {
  const session = await getSession();
  if (!session.userId) redirect("/login");

  const hackathons = await getHackathonsByOrg(session.organizationId);

  return (
    <div className="container mx-auto px-4 py-8">
      <div className="mb-8 flex items-center justify-between">
        <div>
          <h1 className="text-3xl font-bold">Hackathons</h1>
          <p className="text-gray-600">Manage and participate in hackathons</p>
        </div>
        <CreateHackathonButton />
      </div>

      <div className="grid gap-6 md:grid-cols-2 lg:grid-cols-3">
        {hackathons.map((hackathon) => (
          <HackathonCard key={hackathon.id} hackathon={hackathon} />
        ))}
      </div>

      {hackathons.length === 0 && (
        <div className="text-center py-12">
          <p className="text-gray-500">No hackathons yet</p>
          <CreateHackathonButton />
        </div>
      )}
    </div>
  );
}
```

**File**: `src/components/hackathons/create-button.tsx`

```typescript
"use client";

import { useState } from "react";
import { Dialog, DialogContent, DialogHeader, DialogTitle, DialogTrigger } from "@/components/ui/dialog";
import { Button } from "@/components/ui/button";
import { CreateHackathonForm } from "./create-form";

export function CreateHackathonButton() {
  const [open, setOpen] = useState(false);

  return (
    <Dialog open={open} onOpenChange={setOpen}>
      <DialogTrigger asChild>
        <Button>Create Hackathon</Button>
      </DialogTrigger>
      <DialogContent className="max-w-2xl">
        <DialogHeader>
          <DialogTitle>Create New Hackathon</DialogTitle>
        </DialogHeader>
        <CreateHackathonForm onSuccess={() => setOpen(false)} />
      </DialogContent>
    </Dialog>
  );
}
```

**File**: `src/components/hackathons/create-form.tsx`

```typescript
"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";

export function CreateHackathonForm({ onSuccess }: { onSuccess: () => void }) {
  const router = useRouter();
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  async function handleSubmit(e: React.FormEvent<HTMLFormElement>) {
    e.preventDefault();
    setLoading(true);
    setError(null);

    const formData = new FormData(e.currentTarget);
    const data = {
      title: formData.get("title") as string,
      description: formData.get("description") as string,
      slug: formData.get("slug") as string,
      startDate: new Date(formData.get("startDate") as string),
      endDate: new Date(formData.get("endDate") as string),
      registrationDeadline: new Date(formData.get("registrationDeadline") as string),
      submissionDeadline: new Date(formData.get("submissionDeadline") as string),
      minTeamSize: parseInt(formData.get("minTeamSize") as string),
      maxTeamSize: parseInt(formData.get("maxTeamSize") as string),
    };

    try {
      const res = await fetch("/api/hackathons", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(data),
      });

      if (!res.ok) {
        const error = await res.json();
        throw new Error(error.message || "Failed to create");
      }

      onSuccess();
      router.refresh();
    } catch (err) {
      setError(err instanceof Error ? err.message : "Something went wrong");
    } finally {
      setLoading(false);
    }
  }

  return (
    <form onSubmit={handleSubmit} className="space-y-4">
      {/* Form fields implementation */}
      {error && <div className="text-red-600">{error}</div>}
      <Button type="submit" disabled={loading}>
        {loading ? "Creating..." : "Create Hackathon"}
      </Button>
    </form>
  );
}
```

### Validation

- ✅ Organizers can create hackathons (FR-005)
- ✅ Organizers can edit hackathon details (FR-006)
- ✅ Hackathons display correct status (FR-008)
- ✅ Multi-tenant isolation works (no cross-org leaks)

---

## Phase 4: Team & Participant Flows (FR-009 to FR-013)

**Duration**: 5-6 hours
**Goal**: Implement participant registration, team creation, and joining

### 4.1 Team API Routes

**File**: `src/app/api/teams/route.ts`

```typescript
import { NextRequest, NextResponse } from "next/server";
import { getSession } from "@/lib/auth/session";
import { createTeamSchema } from "@/lib/validation/team";
import { createTeam, getUserTeamForHackathon } from "@/db/queries/teams";

// POST /api/teams - Create team
export async function POST(request: NextRequest) {
  const session = await getSession();
  if (!session.userId) {
    return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
  }

  try {
    const body = await request.json();
    const data = createTeamSchema.parse(body);

    // Check if user already has team for this hackathon
    const existingTeam = await getUserTeamForHackathon(session.userId, data.hackathonId);
    if (existingTeam) {
      return NextResponse.json({ error: "Already in a team" }, { status: 400 });
    }

    const team = await createTeam(data, session.userId);
    return NextResponse.json({ team }, { status: 201 });
  } catch (error) {
    return NextResponse.json({ error: "Failed to create team" }, { status: 500 });
  }
}
```

**File**: `src/app/api/teams/join/route.ts`

```typescript
import { NextRequest, NextResponse } from "next/server";
import { getSession } from "@/lib/auth/session";
import { joinTeamSchema } from "@/lib/validation/team";
import { joinTeamByInviteCode } from "@/db/queries/teams";

// POST /api/teams/join - Join team via invite code
export async function POST(request: NextRequest) {
  const session = await getSession();
  if (!session.userId) {
    return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
  }

  try {
    const body = await request.json();
    const { inviteCode } = joinTeamSchema.parse(body);

    const team = await joinTeamByInviteCode(inviteCode, session.userId);
    return NextResponse.json({ team });
  } catch (error) {
    const message = error instanceof Error ? error.message : "Failed to join team";
    return NextResponse.json({ error: message }, { status: 400 });
  }
}
```

### 4.2 Team UI Components

**File**: `src/app/(dashboard)/hackathons/[id]/page.tsx`

```typescript
import { redirect } from "next/navigation";
import { getSession } from "@/lib/auth/session";
import { getHackathonById } from "@/db/queries/hackathons";
import { getUserTeamForHackathon, getTeamsByHackathon } from "@/db/queries/teams";
import { RegisterButton } from "@/components/hackathons/register-button";
import { TeamSection } from "@/components/teams/team-section";

export default async function HackathonPage({ params }: { params: Promise<{ id: string }> }) {
  const { id } = await params;
  const session = await getSession();
  if (!session.userId) redirect("/login");

  const hackathon = await getHackathonById(id);
  if (!hackathon) redirect("/dashboard");

  const userTeam = await getUserTeamForHackathon(session.userId, id);

  return (
    <div className="container mx-auto px-4 py-8">
      <div className="mb-8">
        <h1 className="text-3xl font-bold">{hackathon.title}</h1>
        <p className="text-gray-600 mt-2">{hackathon.description}</p>
      </div>

      {!userTeam && <RegisterButton hackathonId={id} />}

      {userTeam && <TeamSection team={userTeam} />}
    </div>
  );
}
```

### Validation

- ✅ Participants can register for hackathons (FR-009)
- ✅ Participants can create teams (FR-010)
- ✅ Participants can join via invite code (FR-011)
- ✅ Team size limits enforced (FR-013)

---

## Phase 5: Submissions & Judging (FR-014 to FR-021)

**Duration**: 5-6 hours
**Goal**: Implement project submission and judging workflows

### 5.1 Submission API Routes

(Similar implementation pattern as above)

### 5.2 Judging UI

(Judge dashboard, scoring interface)

### Validation

- ✅ Team leads can submit projects (FR-014)
- ✅ Submission deadlines enforced (FR-015)
- ✅ Judges can score submissions (FR-018)
- ✅ Aggregate scores calculated correctly (FR-020)
- ✅ Leaderboard displays rankings (FR-021)

---

## Phase 6: Role Management (FR-022, FR-023)

**Duration**: 3-4 hours
**Goal**: Implement role assignment UI for organizers

(Implementation details)

---

## Phase 7: Communication (FR-024, FR-025)

**Duration**: 3-4 hours
**Goal**: Basic team messaging and mentor feedback

(Implementation details)

---

## Phase 8: Polish & Testing

**Duration**: 5-6 hours
**Goal**: E2E tests, performance optimization, documentation

### 8.1 Integration Tests
### 8.2 Performance Testing (SC-004)
### 8.3 Security Audit (SC-007)
### 8.4 Documentation

---

## Deployment Checklist

### Pre-Deployment

- [ ] **SSO Client Registration** (Manual - User Action):
  ```bash
  # 1. Add client to apps/sso/src/lib/trusted-clients.ts
  # 2. Run: cd apps/sso && pnpm run seed:setup
  ```

- [ ] **Database Setup**:
  ```bash
  # Option 1: Shared Neon instance with SSO
  DATABASE_URL=<same-as-sso>

  # Option 2: Separate Neon project
  # Create new project at https://neon.tech
  ```

- [ ] **Environment Variables** (Production):
  ```env
  DATABASE_URL=<production-neon-url>
  SSO_OAUTH_ISSUER=https://sso.panaversity.org
  SSO_OAUTH_REDIRECT_URI=https://hackathon.panaversity.org/api/auth/callback
  SESSION_SECRET=<generate-with-openssl-rand>
  NEXT_PUBLIC_APP_URL=https://hackathon.panaversity.org
  ```

### Deployment Steps

1. **Build Application**:
   ```bash
   pnpm run build
   ```

2. **Run Migrations**:
   ```bash
   pnpm db:push
   ```

3. **Seed Production Data** (Optional):
   ```bash
   pnpm run seed:prod
   ```

4. **Deploy to Cloud Run**:
   ```bash
   # Using existing SSO deployment pattern
   gcloud run deploy hackathon-platform \
     --source . \
     --region us-central1 \
     --allow-unauthenticated
   ```

### Post-Deployment

- [ ] Test OAuth flow end-to-end
- [ ] Verify multi-tenant isolation
- [ ] Run performance tests (500 concurrent users)
- [ ] Monitor error logs for 24 hours

---

## Success Metrics Validation

After deployment, validate each success criterion:

| Criterion | Test | Target | How to Measure |
|-----------|------|--------|----------------|
| SC-001 | Create hackathon | <30s | Time from form submit to list view |
| SC-002 | Registration + team | <3min | Complete user flow timing |
| SC-003 | Score submission | <5min | Judge scores all criteria |
| SC-004 | Concurrent load | 500 users | Load test with k6/artillery |
| SC-005 | First-attempt success | 95% | User testing + analytics |
| SC-006 | Page load time | <2s | Lighthouse audit |
| SC-007 | Multi-tenant isolation | Zero leaks | Security audit + tests |

---

## Appendix A: File Checklist

Complete file list with implementation status:

**Phase 0: Setup**
- [ ] `apps/hackathon/package.json`
- [ ] `apps/hackathon/next.config.ts`
- [ ] `apps/hackathon/tailwind.config.ts`
- [ ] `apps/hackathon/.env.local.example`

**Phase 1: Database & Auth**
- [ ] `src/db/schema.ts`
- [ ] `src/db/index.ts`
- [ ] `drizzle.config.ts`
- [ ] `src/lib/auth/oauth-client.ts`
- [ ] `src/lib/auth/session.ts`
- [ ] `src/lib/auth/jwt-verify.ts`
- [ ] `src/app/api/auth/login/route.ts`
- [ ] `src/app/api/auth/callback/route.ts`
- [ ] `src/app/api/auth/logout/route.ts`
- [ ] `middleware.ts`

**Phase 2: Queries & Validation**
- [ ] `src/lib/validation/hackathon.ts`
- [ ] `src/lib/validation/team.ts`
- [ ] `src/lib/validation/submission.ts`
- [ ] `src/db/queries/hackathons.ts`
- [ ] `src/db/queries/teams.ts`
- [ ] `src/db/queries/submissions.ts`
- [ ] `src/lib/auth/permissions.ts`

**Phase 3-7**: (50+ additional files)

**Total Files**: ~80-100

---

**Next Steps**: Begin with Phase 0 setup. Each phase builds on the previous, with clear validation checkpoints.
