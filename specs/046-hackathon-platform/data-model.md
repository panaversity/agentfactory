# Database Schema Design: Hackathon Platform

**Feature**: 046-hackathon-platform
**Database**: Neon PostgreSQL (shared with SSO or separate project)
**ORM**: Drizzle ORM
**Date**: 2025-12-22

---

## 1. Schema Overview

```
┌─────────────┐       ┌──────────────┐       ┌─────────────┐
│ hackathons  │◄──────│hackathon_    │──────►│ users (SSO) │
└─────────────┘       │roles         │       └─────────────┘
       │              └──────────────┘              │
       │                                            │
       │              ┌──────────────┐              │
       ├─────────────►│ teams        │◄─────────────┤
       │              └──────────────┘              │
       │                     │                      │
       │                     │                      │
       │              ┌──────────────┐              │
       │              │ team_members │──────────────┤
       │              └──────────────┘              │
       │                                            │
       │              ┌──────────────┐              │
       ├─────────────►│ submissions  │◄─────────────┤
       │              └──────────────┘              │
       │                     │                      │
       │              ┌──────────────┐              │
       │              │ scores       │──────────────┤
       │              └──────────────┘              │
       │                                            │
       │              ┌──────────────┐              │
       └─────────────►│ judging_     │              │
                      │ criteria     │              │
                      └──────────────┘              │
                                                    │
                      ┌──────────────┐              │
                      │ team_        │──────────────┤
                      │ messages     │              │
                      └──────────────┘
```

**Multi-tenancy**: All tables include `organization_id` (FK to SSO organizations table).

---

## 2. Table Definitions (Drizzle Schema)

### 2.1 Users (Reference Only - Stored in SSO DB)

**Note**: Users are managed by SSO. Hackathon app reads user data from JWT claims.

```typescript
// Reference type only - not created by hackathon app
export interface UserFromJWT {
  id: string;
  email: string;
  name: string;
  organizationId: string; // From JWT tenant_id claim
  role: string; // SSO role (admin/user)
}
```

---

### 2.2 Hackathons

**Purpose**: Core hackathon events with configuration

```typescript
import { pgTable, text, timestamp, integer, boolean } from "drizzle-orm/pg-core";

export const hackathons = pgTable("hackathons", {
  id: text("id").primaryKey().$defaultFn(() => crypto.randomUUID()),
  organizationId: text("organization_id").notNull(), // Multi-tenant isolation

  // Basic Info
  title: text("title").notNull(),
  description: text("description").notNull(),
  slug: text("slug").notNull(), // URL-friendly identifier

  // Dates
  startDate: timestamp("start_date").notNull(),
  endDate: timestamp("end_date").notNull(),
  registrationDeadline: timestamp("registration_deadline").notNull(),
  submissionDeadline: timestamp("submission_deadline").notNull(),

  // Team Configuration
  minTeamSize: integer("min_team_size").notNull().default(1),
  maxTeamSize: integer("max_team_size").notNull().default(5),

  // Status
  status: text("status").notNull().default("draft"), // draft, open, active, judging, completed
  published: boolean("published").notNull().default(false),

  // Metadata
  createdAt: timestamp("created_at").defaultNow().notNull(),
  updatedAt: timestamp("updated_at").defaultNow().$onUpdate(() => new Date()).notNull(),
  createdBy: text("created_by").notNull(), // User ID from SSO
});

// Indexes
export const hackathonsOrgIndex = index("hackathons_org_idx")
  .on(hackathons.organizationId);

export const hackathonsSlugIndex = index("hackathons_slug_idx")
  .on(hackathons.slug);

export const hackathonsStatusIndex = index("hackathons_status_idx")
  .on(hackathons.status, hackathons.published);
```

**Validation Rules**:
- `endDate` must be after `startDate`
- `registrationDeadline` must be before `startDate`
- `submissionDeadline` must be before or equal to `endDate`
- `maxTeamSize` must be >= `minTeamSize`
- `slug` must be unique within organization

**Status Transitions**:
```
draft → open → active → judging → completed
         ↓
    (unpublished)
```

---

### 2.3 Judging Criteria

**Purpose**: Define scoring criteria for hackathons

```typescript
export const judgingCriteria = pgTable("judging_criteria", {
  id: text("id").primaryKey().$defaultFn(() => crypto.randomUUID()),
  hackathonId: text("hackathon_id").notNull()
    .references(() => hackathons.id, { onDelete: "cascade" }),

  name: text("name").notNull(), // e.g., "Innovation", "Technical Execution"
  description: text("description"), // Guidance for judges
  weight: integer("weight").notNull().default(1), // Relative importance (1-5)
  maxScore: integer("max_score").notNull().default(10), // Max points (usually 10)
  order: integer("order").notNull(), // Display order

  createdAt: timestamp("created_at").defaultNow().notNull(),
});

// Indexes
export const criteriaHackathonIndex = index("criteria_hackathon_idx")
  .on(judgingCriteria.hackathonId, judgingCriteria.order);

// Relations
export const judgingCriteriaRelations = relations(judgingCriteria, ({ one }) => ({
  hackathon: one(hackathons, {
    fields: [judgingCriteria.hackathonId],
    references: [hackathons.id],
  }),
}));
```

**Default Criteria** (Created on hackathon creation):
1. Innovation (weight: 2, max: 10)
2. Technical Execution (weight: 2, max: 10)
3. Impact/Usefulness (weight: 1, max: 10)
4. Presentation (weight: 1, max: 10)

---

### 2.4 Hackathon Roles

**Purpose**: Per-hackathon role assignments (organizer, judge, mentor, etc.)

```typescript
export const hackathonRoles = pgTable("hackathon_roles", {
  id: text("id").primaryKey().$defaultFn(() => crypto.randomUUID()),

  hackathonId: text("hackathon_id").notNull()
    .references(() => hackathons.id, { onDelete: "cascade" }),
  userId: text("user_id").notNull(), // User ID from SSO JWT

  role: text("role").notNull(), // organizer, manager, judge, mentor, participant

  // Mentor-specific (optional)
  maxTeamsAssigned: integer("max_teams_assigned"), // Mentor workload limit

  createdAt: timestamp("created_at").defaultNow().notNull(),
  assignedBy: text("assigned_by"), // User ID who assigned this role
});

// Indexes
export const rolesUserHackathonIndex = index("roles_user_hackathon_idx")
  .on(hackathonRoles.userId, hackathonRoles.hackathonId);

export const rolesHackathonRoleIndex = index("roles_hackathon_role_idx")
  .on(hackathonRoles.hackathonId, hackathonRoles.role);

// Composite unique constraint: one role per user per hackathon
export const rolesUnique = unique("roles_user_hackathon_role_unique")
  .on(hackathonRoles.userId, hackathonRoles.hackathonId, hackathonRoles.role);
```

**Role Permissions**:
| Role | Create Hackathon | Assign Roles | Edit Teams | Judge Submissions | Mentor Teams | Submit Projects |
|------|-----------------|--------------|------------|-------------------|--------------|-----------------|
| organizer | ✓ | ✓ | ✓ | ✓ | ✓ | ✗ |
| manager | ✗ | ✓ | ✓ | ✓ | ✓ | ✗ |
| judge | ✗ | ✗ | ✗ | ✓ | ✗ | ✗ |
| mentor | ✗ | ✗ | ✗ | ✗ | ✓ | ✗ |
| participant | ✗ | ✗ | ✗ | ✗ | ✗ | ✓ |

---

### 2.5 Teams

**Purpose**: Participant teams for hackathons

```typescript
import { customAlphabet } from "nanoid";

const generateInviteCode = customAlphabet("23456789ABCDEFGHJKLMNPQRSTUVWXYZ", 8);

export const teams = pgTable("teams", {
  id: text("id").primaryKey().$defaultFn(() => crypto.randomUUID()),

  hackathonId: text("hackathon_id").notNull()
    .references(() => hackathons.id, { onDelete: "cascade" }),

  name: text("name").notNull(),
  description: text("description"),
  inviteCode: text("invite_code").notNull().unique()
    .$defaultFn(() => generateInviteCode()),

  leaderId: text("leader_id").notNull(), // User ID from SSO

  // Status
  status: text("status").notNull().default("forming"), // forming, ready, submitted, disqualified

  createdAt: timestamp("created_at").defaultNow().notNull(),
  updatedAt: timestamp("updated_at").defaultNow().$onUpdate(() => new Date()).notNull(),
});

// Indexes
export const teamsHackathonIndex = index("teams_hackathon_idx")
  .on(teams.hackathonId);

export const teamsInviteCodeIndex = index("teams_invite_code_idx")
  .on(teams.inviteCode);

export const teamsLeaderIndex = index("teams_leader_idx")
  .on(teams.leaderId);

// Composite unique: team name per hackathon
export const teamsNameUnique = unique("teams_name_hackathon_unique")
  .on(teams.hackathonId, teams.name);
```

**Status Transitions**:
```
forming → ready → submitted
   ↓
disqualified
```

**Business Logic**:
- Team is "ready" when size >= minTeamSize
- Team leader can remove members, update team info
- Invite code regenerates if team leader changes (security)

---

### 2.6 Team Members

**Purpose**: Junction table linking users to teams

```typescript
export const teamMembers = pgTable("team_members", {
  id: text("id").primaryKey().$defaultFn(() => crypto.randomUUID()),

  teamId: text("team_id").notNull()
    .references(() => teams.id, { onDelete: "cascade" }),
  userId: text("user_id").notNull(), // User ID from SSO

  role: text("role").notNull().default("member"), // leader, member

  joinedAt: timestamp("joined_at").defaultNow().notNull(),
});

// Indexes
export const membersTeamIndex = index("members_team_idx")
  .on(teamMembers.teamId);

export const membersUserIndex = index("members_user_idx")
  .on(teamMembers.userId);

// Composite unique: one membership per user per team
export const membersUnique = unique("members_user_team_unique")
  .on(teamMembers.userId, teamMembers.teamId);

// Relations
export const teamMembersRelations = relations(teamMembers, ({ one }) => ({
  team: one(teams, {
    fields: [teamMembers.teamId],
    references: [teams.id],
  }),
}));
```

**Constraints** (Enforced in application logic):
- User can only be on ONE team per hackathon
- Team size <= maxTeamSize (checked before adding member)
- Team must have exactly 1 leader

---

### 2.7 Submissions

**Purpose**: Team project submissions

```typescript
export const submissions = pgTable("submissions", {
  id: text("id").primaryKey().$defaultFn(() => crypto.randomUUID()),

  teamId: text("team_id").notNull()
    .references(() => teams.id, { onDelete: "cascade" }),
  hackathonId: text("hackathon_id").notNull()
    .references(() => hackathons.id, { onDelete: "cascade" }),

  // Project Details
  projectName: text("project_name").notNull(),
  description: text("description").notNull(),
  repositoryUrl: text("repository_url").notNull(),
  demoUrl: text("demo_url"), // Optional live demo
  presentationUrl: text("presentation_url"), // Optional slides/video

  // Status
  status: text("status").notNull().default("submitted"), // submitted, under_review, scored

  // Metadata
  submittedAt: timestamp("submitted_at").defaultNow().notNull(),
  submittedBy: text("submitted_by").notNull(), // User ID who submitted
  updatedAt: timestamp("updated_at").defaultNow().$onUpdate(() => new Date()).notNull(),
});

// Indexes
export const submissionsTeamIndex = index("submissions_team_idx")
  .on(submissions.teamId);

export const submissionsHackathonIndex = index("submissions_hackathon_idx")
  .on(submissions.hackathonId, submissions.status);

// Composite unique: one submission per team per hackathon
export const submissionsUnique = unique("submissions_team_hackathon_unique")
  .on(submissions.teamId, submissions.hackathonId);

// Relations
export const submissionsRelations = relations(submissions, ({ one, many }) => ({
  team: one(teams, {
    fields: [submissions.teamId],
    references: [teams.id],
  }),
  hackathon: one(hackathons, {
    fields: [submissions.hackathonId],
    references: [hackathons.id],
  }),
  scores: many(scores),
}));
```

**Validation**:
- `repositoryUrl` must be valid URL (GitHub, GitLab, etc.)
- `demoUrl` must be valid URL if provided
- Submission only allowed if team.status === "ready"
- Submission only allowed before hackathon.submissionDeadline

---

### 2.8 Scores

**Purpose**: Judge evaluations of submissions

```typescript
export const scores = pgTable("scores", {
  id: text("id").primaryKey().$defaultFn(() => crypto.randomUUID()),

  submissionId: text("submission_id").notNull()
    .references(() => submissions.id, { onDelete: "cascade" }),
  judgeId: text("judge_id").notNull(), // User ID from SSO
  criterionId: text("criterion_id").notNull()
    .references(() => judgingCriteria.id, { onDelete: "cascade" }),

  score: integer("score").notNull(), // 1-10 (or 1-maxScore from criterion)
  feedback: text("feedback"), // Optional written feedback

  createdAt: timestamp("created_at").defaultNow().notNull(),
  updatedAt: timestamp("updated_at").defaultNow().$onUpdate(() => new Date()).notNull(),
});

// Indexes
export const scoresSubmissionIndex = index("scores_submission_idx")
  .on(scores.submissionId);

export const scoresJudgeIndex = index("scores_judge_idx")
  .on(scores.judgeId);

// Composite unique: one score per judge per criterion per submission
export const scoresUnique = unique("scores_judge_criterion_submission_unique")
  .on(scores.judgeId, scores.criterionId, scores.submissionId);

// Relations
export const scoresRelations = relations(scores, ({ one }) => ({
  submission: one(submissions, {
    fields: [scores.submissionId],
    references: [submissions.id],
  }),
  criterion: one(judgingCriteria, {
    fields: [scores.criterionId],
    references: [judgingCriteria.id],
  }),
}));
```

**Aggregate Score Calculation**:
```sql
-- Get final scores for all submissions in a hackathon
WITH weighted_scores AS (
  SELECT
    s.submission_id,
    s.judge_id,
    SUM(s.score * jc.weight) / SUM(jc.weight) AS judge_score
  FROM scores s
  JOIN judging_criteria jc ON s.criterion_id = jc.id
  GROUP BY s.submission_id, s.judge_id
)
SELECT
  submission_id,
  AVG(judge_score) AS final_score,
  COUNT(DISTINCT judge_id) AS num_judges
FROM weighted_scores
GROUP BY submission_id
ORDER BY final_score DESC;
```

---

### 2.9 Team Messages (Communication)

**Purpose**: Team-internal messaging and mentor feedback

```typescript
export const teamMessages = pgTable("team_messages", {
  id: text("id").primaryKey().$defaultFn(() => crypto.randomUUID()),

  teamId: text("team_id").notNull()
    .references(() => teams.id, { onDelete: "cascade" }),
  senderId: text("sender_id").notNull(), // User ID from SSO

  message: text("message").notNull(),

  // Optional: distinguish mentor messages
  senderRole: text("sender_role"), // member, mentor

  createdAt: timestamp("created_at").defaultNow().notNull(),
});

// Indexes
export const messagesTeamIndex = index("messages_team_idx")
  .on(teamMessages.teamId, teamMessages.createdAt);

// Relations
export const teamMessagesRelations = relations(teamMessages, ({ one }) => ({
  team: one(teams, {
    fields: [teamMessages.teamId],
    references: [teams.id],
  }),
}));
```

**Access Control**:
- Team members can read/write team messages
- Mentors assigned to team can read/write messages
- Organizers/managers can read (moderation)

---

## 3. Drizzle Configuration

### 3.1 drizzle.config.ts

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

### 3.2 Database Connection (src/db/index.ts)

```typescript
import { drizzle } from "drizzle-orm/neon-serverless";
import { Pool } from "@neondatabase/serverless";
import * as schema from "./schema";

const pool = new Pool({ connectionString: process.env.DATABASE_URL });
export const db = drizzle(pool, { schema });
```

---

## 4. Migrations

### 4.1 Initial Migration

```bash
# Generate migration from schema
pnpm db:generate

# Apply migration
pnpm db:push
```

### 4.2 Migration Files

**drizzle/0000_initial.sql** (Auto-generated):
```sql
CREATE TABLE "hackathons" (
  "id" text PRIMARY KEY,
  "organization_id" text NOT NULL,
  "title" text NOT NULL,
  "description" text NOT NULL,
  "slug" text NOT NULL,
  -- ... all fields
);

CREATE INDEX "hackathons_org_idx" ON "hackathons" ("organization_id");
CREATE INDEX "hackathons_slug_idx" ON "hackathons" ("slug");

-- Repeat for all tables
```

---

## 5. Seed Data (Development)

### 5.1 Seed Script (scripts/seed-dev.ts)

```typescript
import { db } from "../src/db";
import { hackathons, judgingCriteria } from "../src/db/schema";

async function seed() {
  console.log("Seeding development data...");

  // Create sample hackathon
  const [hackathon] = await db.insert(hackathons).values({
    id: "test-hackathon-1",
    organizationId: "panaversity-default-org-id", // From SSO
    title: "AI Innovation Challenge 2025",
    description: "Build innovative AI applications that solve real-world problems",
    slug: "ai-innovation-2025",
    startDate: new Date("2025-02-01"),
    endDate: new Date("2025-02-28"),
    registrationDeadline: new Date("2025-01-25"),
    submissionDeadline: new Date("2025-02-28"),
    minTeamSize: 2,
    maxTeamSize: 5,
    status: "open",
    published: true,
    createdBy: "admin-user-id",
  }).returning();

  // Create judging criteria
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
      description: "Potential real-world usefulness and value",
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

  console.log("✅ Seed complete!");
}

seed().catch(console.error);
```

**Run with**: `pnpm run seed:dev`

---

## 6. Query Patterns

### 6.1 Get User's Hackathons (Multi-tenant Safe)

```typescript
async function getUserHackathons(userId: string, organizationId: string) {
  return await db
    .select({
      hackathon: hackathons,
      role: hackathonRoles.role,
    })
    .from(hackathons)
    .leftJoin(
      hackathonRoles,
      and(
        eq(hackathonRoles.hackathonId, hackathons.id),
        eq(hackathonRoles.userId, userId)
      )
    )
    .where(
      and(
        eq(hackathons.organizationId, organizationId),
        eq(hackathons.published, true)
      )
    );
}
```

### 6.2 Get Team with Members

```typescript
async function getTeamWithMembers(teamId: string) {
  return await db.query.teams.findFirst({
    where: eq(teams.id, teamId),
    with: {
      members: true,
      submission: true,
    },
  });
}
```

### 6.3 Get Submissions for Judging

```typescript
async function getSubmissionsForJudge(judgeId: string, hackathonId: string) {
  return await db
    .select({
      submission: submissions,
      team: teams,
      myScores: scores,
    })
    .from(submissions)
    .innerJoin(teams, eq(submissions.teamId, teams.id))
    .leftJoin(
      scores,
      and(
        eq(scores.submissionId, submissions.id),
        eq(scores.judgeId, judgeId)
      )
    )
    .where(eq(submissions.hackathonId, hackathonId));
}
```

### 6.4 Get Leaderboard

```typescript
async function getLeaderboard(hackathonId: string) {
  // Complex query - use raw SQL for performance
  return await db.execute(sql`
    WITH weighted_scores AS (
      SELECT
        s.submission_id,
        s.judge_id,
        SUM(s.score * jc.weight) / NULLIF(SUM(jc.weight), 0) AS judge_score
      FROM scores s
      JOIN judging_criteria jc ON s.criterion_id = jc.id
      JOIN submissions sub ON s.submission_id = sub.id
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
    JOIN submissions sub ON ws.submission_id = sub.id
    JOIN teams t ON sub.team_id = t.id
    GROUP BY sub.id, sub.project_name, t.name
    ORDER BY final_score DESC;
  `);
}
```

---

## 7. Data Integrity Rules

### 7.1 Foreign Key Constraints
✅ All foreign keys use `onDelete: "cascade"` for referential integrity
✅ User IDs stored as text (no FK to SSO DB for decoupling)

### 7.2 Unique Constraints
✅ Team name unique per hackathon
✅ One submission per team per hackathon
✅ One role assignment per user per hackathon
✅ One score per judge per criterion per submission

### 7.3 Check Constraints (Application-Level)
✅ Team size within min/max bounds
✅ Score within 1-maxScore range
✅ Deadline ordering (registration < start < submission <= end)

---

## 8. Performance Optimizations

### 8.1 Indexes Summary

| Table | Index | Purpose |
|-------|-------|---------|
| hackathons | organization_id | Multi-tenant queries |
| hackathons | slug | URL lookups |
| hackathons | (status, published) | Catalog filtering |
| teams | hackathon_id | List teams per hackathon |
| teams | invite_code | Join team by code |
| team_members | team_id | Get team roster |
| team_members | user_id | Check user's team |
| hackathon_roles | (user_id, hackathon_id) | Auth checks |
| submissions | hackathon_id | List submissions |
| scores | submission_id | Get scores for submission |

### 8.2 Query Optimization Tips

1. **Use select() sparingly**: Only fetch needed columns
2. **Leverage Drizzle query builder**: Automatic JOIN optimization
3. **Batch inserts**: Use array values for multiple rows
4. **Connection pooling**: Neon serverless handles automatically

---

## 9. Next Steps

1. ✅ Create `src/db/schema.ts` with all table definitions
2. ✅ Configure `drizzle.config.ts`
3. ✅ Generate and apply initial migration
4. ✅ Create seed script for development data
5. ✅ Write query utility functions in `src/db/queries/`
6. ✅ Add Zod validation schemas matching DB schema

---

**References**:
- [Drizzle ORM Schema Reference](https://orm.drizzle.team/docs/sql-schema-declaration)
- [Neon Serverless Docs](https://neon.tech/docs/serverless/serverless-driver)
- [PostgreSQL Index Types](https://www.postgresql.org/docs/current/indexes-types.html)
