# Hackathon Platform Architecture

## Overview

A multi-tenant hackathon management platform integrated with Panaversity SSO for authentication and user management.

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           HACKATHON PLATFORM                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────┐     ┌──────────────┐     ┌──────────────┐                │
│  │   Next.js    │     │   Drizzle    │     │    Neon      │                │
│  │   App (3002) │────▶│     ORM      │────▶│  PostgreSQL  │                │
│  └──────┬───────┘     └──────────────┘     └──────────────┘                │
│         │                                                                   │
│         │ OAuth 2.1 + OIDC (PKCE)                                          │
│         ▼                                                                   │
│  ┌──────────────┐     ┌──────────────┐                                     │
│  │     SSO      │────▶│   M2M API    │  (User lookup by username)          │
│  │   (3001)     │     │  /api/m2m/*  │                                     │
│  └──────────────┘     └──────────────┘                                     │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Authentication Flow

### OAuth 2.1 with PKCE (Public Client)

```
User                    Hackathon App              SSO
  │                          │                      │
  │──── Click Login ────────▶│                      │
  │                          │                      │
  │                          │── Generate PKCE ────▶│
  │                          │   (state + verifier) │
  │                          │                      │
  │◀─── Redirect to SSO ─────│                      │
  │                          │                      │
  │─────────────────────────────── Login ─────────▶│
  │                          │                      │
  │◀──────────────────────── Redirect + Code ──────│
  │                          │                      │
  │──── Callback ───────────▶│                      │
  │                          │── Exchange Code ────▶│
  │                          │   (with verifier)    │
  │                          │                      │
  │                          │◀─── Tokens ──────────│
  │                          │   (access + id)      │
  │                          │                      │
  │                          │── Verify JWT ───────▶│
  │                          │   (via JWKS)         │
  │                          │                      │
  │◀─── Dashboard ───────────│                      │
  │                          │                      │
```

### Session Management

- **Technology**: iron-session (encrypted HTTP-only cookies)
- **Cookie Name**: `hackathon-session`
- **Lifetime**: 7 days
- **Contents**:
  ```typescript
  {
    isLoggedIn: boolean;
    user: {
      id: string;
      email: string;
      name: string;
      username: string;      // For @mentions and lookups
      image?: string;        // Profile avatar
      organizationId: string;
      organizationName?: string;
    };
    accessToken: string;
    idToken: string;
    expiresAt: number;
  }
  ```

## Data Architecture

### Denormalization Strategy

User data is denormalized across all tables to minimize API calls:

```
SSO (Source of Truth)          Hackathon DB (Denormalized)
┌──────────────────┐           ┌──────────────────────────────┐
│ users            │           │ hackathon_roles              │
│ ├─ id            │──────────▶│ ├─ user_id                   │
│ ├─ username      │           │ ├─ username                  │
│ ├─ name          │           │ ├─ name                      │
│ ├─ email         │           │ ├─ email                     │
│ └─ image         │           │ └─ image                     │
└──────────────────┘           └──────────────────────────────┘
```

**Tables with denormalized user data:**

| Table | User Fields |
|-------|-------------|
| `organization_members` | username, name, email, image |
| `hackathon_roles` | username, name, email, image |
| `teams` | leader_username, leader_name, leader_image |
| `team_members` | username, name, email, image |
| `submissions` | submitter_username, submitter_name |
| `scores` | judge_username, judge_name |
| `team_messages` | sender_username, sender_name, sender_image |
| `winners` | announcer_username, announcer_name |

### Multi-Tenancy

All tables include `organization_id` for:
- Row-level security queries
- Data isolation
- Future sharding capability

```sql
-- Example: Get all hackathons for an organization
SELECT * FROM hackathons WHERE organization_id = $1;

-- Example: Index for efficient org-scoped queries
CREATE INDEX hackathons_org_idx ON hackathons(organization_id);
```

## Database Schema

### Core Entities

```
organizations
├── id (PK)
├── name
├── slug (unique)
├── description
├── logo_url
├── website_url
└── is_active

organization_members
├── id (PK)
├── organization_id (FK)
├── user_id
├── username, name, email, image  (denormalized)
├── role (owner|admin|manager|member)
└── joined_at

hackathons
├── id (PK)
├── organization_id (FK)
├── title, description, slug
├── start_date, end_date
├── registration_deadline, submission_deadline
├── min_team_size, max_team_size
├── prizes (JSON)
├── organizers (JSON)
├── sponsors (JSON)
├── categories (JSON)
├── status (draft|open|active|judging|completed)
└── published

hackathon_roles
├── id (PK)
├── hackathon_id (FK)
├── organization_id (FK)
├── user_id
├── username, name, email, image  (denormalized)
├── role (organizer|manager|judge|mentor|participant)
├── status (invited|active|declined)
└── max_teams_assigned (for mentors)

teams
├── id (PK)
├── hackathon_id (FK)
├── organization_id (FK)
├── name, description
├── invite_code (unique)
├── leader_id
├── leader_username, leader_name, leader_image  (denormalized)
└── status (forming|ready|submitted|disqualified)

team_members
├── id (PK)
├── team_id (FK)
├── organization_id (FK)
├── user_id
├── username, name, email, image  (denormalized)
├── role (leader|member)
├── status (invited|accepted|declined)
├── invited_at
└── joined_at (null until accepted)

submissions
├── id (PK)
├── team_id (FK)
├── hackathon_id (FK)
├── organization_id (FK)
├── project_name, description
├── repository_url, demo_url, presentation_url
├── category_id
├── status (submitted|under_review|scored)
├── submitted_by
└── submitter_username, submitter_name  (denormalized)

judging_criteria
├── id (PK)
├── hackathon_id (FK)
├── name, description
├── weight, max_score
└── order

scores
├── id (PK)
├── submission_id (FK)
├── organization_id (FK)
├── criterion_id (FK)
├── judge_id
├── judge_username, judge_name  (denormalized)
├── score
└── feedback

winners
├── id (PK)
├── hackathon_id (FK)
├── organization_id (FK)
├── submission_id (FK)
├── team_id (FK)
├── place, prize_title, prize_value
├── category_id
├── announced_by
└── announcer_username, announcer_name  (denormalized)
```

## API Endpoints

### Authentication

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/auth/login` | GET | Initiate OAuth flow |
| `/api/auth/callback` | GET | Handle OAuth callback |
| `/api/auth/logout` | GET | Destroy session |
| `/api/auth/dev-login` | POST | Dev-only test login |

### Organizations

| Endpoint | Method | Auth | Description |
|----------|--------|------|-------------|
| `/api/organizations` | GET | Yes | List user's orgs |
| `/api/organizations` | POST | Yes | Create org |
| `/api/organizations/[id]` | GET | Yes | Get org details |
| `/api/organizations/[id]` | PATCH | Admin | Update org |
| `/api/organizations/[id]/members` | GET | Yes | List members |
| `/api/organizations/[id]/members` | POST | Admin | Add member |

### Hackathons

| Endpoint | Method | Auth | Description |
|----------|--------|------|-------------|
| `/api/hackathons` | GET | No* | List hackathons |
| `/api/hackathons` | POST | Yes | Create hackathon |
| `/api/hackathons/[id]` | GET | No* | Get details |
| `/api/hackathons/[id]` | PATCH | Manager | Update |
| `/api/hackathons/[id]/publish` | POST | Manager | Toggle publish |
| `/api/hackathons/[id]/register` | POST | Yes | Register user |

### Roles

| Endpoint | Method | Auth | Description |
|----------|--------|------|-------------|
| `/api/hackathons/[id]/roles` | GET | Manager | List roles |
| `/api/hackathons/[id]/roles` | POST | Manager | Assign role |
| `/api/hackathons/[id]/roles` | DELETE | Manager | Remove role |

### Teams

| Endpoint | Method | Auth | Description |
|----------|--------|------|-------------|
| `/api/hackathons/[id]/teams` | GET | Yes | List teams |
| `/api/hackathons/[id]/teams` | POST | Participant | Create team |
| `/api/hackathons/[id]/teams/join` | POST | Participant | Join via code |

### Submissions

| Endpoint | Method | Auth | Description |
|----------|--------|------|-------------|
| `/api/hackathons/[id]/submissions` | GET | Judge+ | List submissions |
| `/api/hackathons/[id]/submissions` | POST | Leader | Submit project |
| `/api/hackathons/[id]/submissions/[subId]/score` | POST | Judge | Score submission |

### Judging

| Endpoint | Method | Auth | Description |
|----------|--------|------|-------------|
| `/api/hackathons/[id]/criteria` | GET | Yes | List criteria |
| `/api/hackathons/[id]/criteria` | POST | Manager | Add criterion |

## Role-Based Access Control

### Organization Roles (from SSO)

| Role | Capabilities |
|------|--------------|
| `owner` | Full control, billing, delete org |
| `admin` | Manage members, settings |
| `manager` | Create hackathons, manage content |
| `member` | Participate in hackathons |

### Hackathon Roles (per-hackathon)

| Role | Capabilities |
|------|--------------|
| `organizer` | Full hackathon control |
| `manager` | Manage teams, criteria, roles |
| `judge` | Score submissions |
| `mentor` | Support teams (limited capacity) |
| `participant` | Join teams, submit projects |

### Permission Matrix

```
Action                    | Creator | Organizer | Manager | Judge | Mentor | Participant
--------------------------|---------|-----------|---------|-------|--------|------------
Edit hackathon            |    ✓    |     ✓     |    ✓    |       |        |
Publish/unpublish         |    ✓    |     ✓     |    ✓    |       |        |
Assign roles              |    ✓    |     ✓     |    ✓    |       |        |
View all submissions      |    ✓    |     ✓     |    ✓    |   ✓   |        |
Score submissions         |    ✓    |     ✓     |         |   ✓   |        |
View team details         |    ✓    |     ✓     |    ✓    |   ✓   |   ✓    |
Create team               |         |           |         |       |        |     ✓
Join team                 |         |           |         |       |        |     ✓
Submit project            |         |           |         |       |        |     ✓
```

## User Flows

### For Participants

1. **Registration**
   - Visit `/h/[slug]` (public hackathon page)
   - Click "Register" → redirected to login if needed
   - After login, registration confirmed
   - Can create or join a team

2. **Team Formation**
   - **Create team**: Set name, description → get invite code
   - **Join team**: Enter 8-character invite code
   - **Invite by username**: Type `@username` → send invitation
   - **Accept invitation**: View in dashboard → click accept

3. **Submission**
   - Team leader submits project details
   - Required: name, description, repository URL
   - Optional: demo URL, presentation URL
   - Select category (if applicable)

### For Organizers

1. **Create Hackathon**
   - Fill details (dates, team sizes, description)
   - Save as draft
   - Add judging criteria
   - Add roles (judges, mentors)
   - Publish when ready

2. **Manage Roles**
   - Type `@username` to add organizer/judge/mentor
   - User looked up from SSO → stored with denormalized data
   - User sees invitation in dashboard

3. **Judging Phase**
   - Change status to "judging"
   - Judges see assigned submissions
   - Score against criteria
   - View analytics dashboard

4. **Announce Winners**
   - Select winning submissions
   - Assign prizes
   - Publish results

### For Judges

1. **Access Submissions**
   - View assigned submissions in dashboard
   - Read project details, visit demo/repo

2. **Score Submissions**
   - Rate each criterion (1-10 scale)
   - Provide feedback (optional)
   - Submit scores

## M2M User Lookup

When adding users by username (roles, team invitations):

```typescript
// 1. Organizer types username
const username = "@johndoe";

// 2. Look up user from SSO
const result = await getUserByUsername(username);
// Returns: { id, username, name, email, image }

// 3. Store denormalized data
await db.insert(hackathonRoles).values({
  hackathonId,
  organizationId,
  userId: result.user.id,
  username: result.user.username,  // Preserved for display
  name: result.user.name,
  email: result.user.email,
  image: result.user.image,
  role: "judge",
});

// 4. Zero API calls for display thereafter
```

## Environment Variables

```bash
# Database
DATABASE_URL="postgresql://..."

# SSO OAuth
SSO_OAUTH_CLIENT_ID="hackathon-public-client"
SSO_OAUTH_REDIRECT_URI="http://localhost:3002/api/auth/callback"
SSO_OAUTH_AUTHORIZE_URL="http://localhost:3001/api/auth/oauth2/authorize"
SSO_OAUTH_TOKEN_URL="http://localhost:3001/api/auth/oauth2/token"
SSO_OAUTH_JWKS_URL="http://localhost:3001/api/auth/jwks"
SSO_OAUTH_ISSUER="http://localhost:3001"

# SSO URLs
NEXT_PUBLIC_SSO_URL="http://localhost:3001"

# M2M API (for user lookups)
SSO_API_KEY="your-sso-api-key"

# Session
SESSION_SECRET="32+ character secret"

# App
NEXT_PUBLIC_APP_URL="http://localhost:3002"
NEXT_PUBLIC_APP_NAME="Panaversity Hackathon Platform"
```

## SSO Integration Requirements

### JWT Claims Required

The SSO must include these claims in the ID token:

```typescript
{
  sub: string;         // User ID
  email: string;       // Email address
  name: string;        // Display name
  username: string;    // Username (for @mentions)
  image?: string;      // Profile image URL
  tenant_id: string;   // Organization ID
  tenant_name?: string;// Organization name
}
```

### M2M API Endpoint

SSO must expose:

```
GET /api/m2m/users/{username}
Headers: x-api-key: {SSO_API_KEY}

Response:
{
  "id": "user-uuid",
  "username": "johndoe",
  "displayUsername": "JohnDoe",
  "name": "John Doe",
  "email": "john@example.com",
  "image": "https://...",
  "createdAt": "2024-01-01T00:00:00Z"
}
```

## Development

### Running Locally

```bash
# Install dependencies
pnpm install

# Start database (or use Neon)
# Set DATABASE_URL in .env.local

# Start SSO (port 3001)
cd ../sso && pnpm dev

# Start hackathon app (port 3002)
pnpm dev
```

### Database Migrations

```bash
# Generate migration from schema changes
npx drizzle-kit generate

# Apply migrations
npx drizzle-kit push

# View database
npx drizzle-kit studio
```

### File Structure

```
apps/hackathon/
├── drizzle/              # Migrations
├── src/
│   ├── app/
│   │   ├── (auth)/       # Login pages
│   │   ├── (dashboard)/  # Protected pages
│   │   ├── (public)/     # Public pages (/h/[id], /explore)
│   │   └── api/          # API routes
│   ├── components/
│   │   ├── hackathons/   # Hackathon-specific components
│   │   ├── layout/       # Navbar, sidebar
│   │   ├── teams/        # Team components
│   │   └── ui/           # shadcn/ui components
│   ├── db/
│   │   ├── index.ts      # Database connection
│   │   ├── schema.ts     # Drizzle schema
│   │   └── queries/      # Query functions
│   ├── lib/
│   │   ├── auth/         # Auth utilities
│   │   ├── validation/   # Zod schemas
│   │   ├── cache.ts      # Cache invalidation
│   │   ├── sso-user-lookup.ts  # M2M user lookup
│   │   └── utils.ts      # Utilities
│   ├── proxy.ts          # Next.js 16 proxy (routing protection)
│   └── types/            # TypeScript types
└── docs/
    └── ARCHITECTURE.md   # This file
```

## Route Protection (Next.js 16 Proxy)

As of Next.js 16, `middleware.ts` is deprecated in favor of `proxy.ts`.

**Key Differences:**
- Runs on Node.js runtime (not Edge)
- Function name changed from `middleware` to `proxy`
- Designed for routing decisions only (redirects, rewrites, headers)
- Authentication verification should happen in Layouts or Route Handlers

**Location:** `src/proxy.ts`

```typescript
import { NextResponse } from "next/server";
import type { NextRequest } from "next/server";

export function proxy(request: NextRequest) {
  const { pathname } = request.nextUrl;

  // Check for session cookie presence
  const sessionCookie = request.cookies.get("hackathon-session");

  if (requiresAuth && !sessionCookie?.value) {
    return NextResponse.redirect(new URL("/login", request.url));
  }

  return NextResponse.next();
}

export const config = {
  matcher: ["/((?!_next/static|_next/image|favicon.ico|public).*))"],
};
```

**Important:** Full session validation (JWT verification, expiry checks) happens in:
- Route Handlers (`app/api/**/route.ts`)
- Server Components (layouts/pages)
- **NOT** in proxy (which only checks cookie presence for routing)

See: [Next.js 16 Proxy Documentation](https://nextjs.org/docs/app/api-reference/file-conventions/proxy)

## Security Considerations

1. **PKCE Flow**: Public client uses code verifier to prevent code interception
2. **State Parameter**: Prevents CSRF attacks on OAuth flow
3. **HTTP-Only Cookies**: Session not accessible via JavaScript
4. **JWT Verification**: All tokens verified against SSO JWKS
5. **Role Checks**: Every API endpoint validates user roles
6. **Org Isolation**: All queries scoped to organization_id
7. **Proxy vs Auth**: Proxy handles routing; auth happens in handlers (prevents CVE-2025-29927)

## Performance Optimizations

1. **Denormalized Data**: Zero API calls for user display
2. **Tag-Based Cache**: Invalidation by entity type
3. **Database Indexes**: All foreign keys and common query paths indexed
4. **JWKS Caching**: JWKS fetched once, cached in memory

## Future Considerations

1. **Row-Level Security**: PostgreSQL RLS using organization_id
2. **Real-time Updates**: WebSocket for team chat, live scoring
3. **File Uploads**: S3 for project assets, presentations
4. **Email Notifications**: Invitations, deadline reminders
5. **API Rate Limiting**: Per-user and per-org limits
