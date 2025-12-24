import {
  pgTable,
  text,
  timestamp,
  integer,
  boolean,
  index,
  unique,
} from "drizzle-orm/pg-core";
import { relations } from "drizzle-orm";
import { customAlphabet } from "nanoid";

// Invite code generator (excludes ambiguous chars: 0, 1, O, I)
const generateInviteCode = customAlphabet(
  "23456789ABCDEFGHJKLMNPQRSTUVWXYZ",
  8
);

// =============================================================================
// HACKATHONS
// =============================================================================
// organization_id comes from SSO JWT (tenant_id) - no FK, just data isolation

export const hackathons = pgTable(
  "hackathons",
  {
    id: text("id")
      .primaryKey()
      .$defaultFn(() => crypto.randomUUID()),

    // Tenant ID from SSO (no FK - SSO is source of truth)
    organizationId: text("organization_id").notNull(),

    // Basic Info
    title: text("title").notNull(),
    description: text("description").notNull(),
    slug: text("slug").notNull(),

    // Dates
    startDate: timestamp("start_date").notNull(),
    endDate: timestamp("end_date").notNull(),
    registrationDeadline: timestamp("registration_deadline").notNull(),
    submissionDeadline: timestamp("submission_deadline").notNull(),

    // Team Configuration
    minTeamSize: integer("min_team_size").notNull().default(1),
    maxTeamSize: integer("max_team_size").notNull().default(5),

    // Prizes (JSON array of { place: "1st", title: "Grand Prize", value: "$5000" })
    prizes: text("prizes"),

    // Organizers (JSON array of { name: "PIAIC", logo?: "url", url?: "https://piaic.org" })
    organizers: text("organizers"),

    // Sponsors (JSON array of { name: "Panaversity", logo?: "url", url?: "https://panaversity.com", tier?: "platinum"|"gold"|"silver"|"bronze" })
    sponsors: text("sponsors"),

    // Categories/Tracks (JSON array of { id: "ai", name: "AI & Machine Learning", description?: "Build AI-powered solutions" })
    // If empty, hackathon has no categories (single track)
    categories: text("categories"),

    // Status
    status: text("status").notNull().default("draft"), // draft, open, active, judging, completed
    published: boolean("published").notNull().default(false),

    // Submission Configuration
    submissionMode: text("submission_mode").notNull().default("platform"), // platform, external, hybrid
    externalFormUrl: text("external_form_url"), // Google Form, Typeform, etc.

    // Metadata
    createdAt: timestamp("created_at").defaultNow().notNull(),
    updatedAt: timestamp("updated_at")
      .defaultNow()
      .$onUpdate(() => new Date())
      .notNull(),
    createdBy: text("created_by").notNull(),
  },
  (table) => [
    index("hackathons_org_idx").on(table.organizationId),
    index("hackathons_slug_idx").on(table.slug),
    index("hackathons_status_idx").on(table.status, table.published),
    unique("hackathons_org_slug_unique").on(table.organizationId, table.slug),
  ]
);

export const hackathonsRelations = relations(hackathons, ({ many }) => ({
  criteria: many(judgingCriteria),
  roles: many(hackathonRoles),
  teams: many(teams),
  submissions: many(submissions),
  submissionFields: many(submissionFields),
  submissionSyncs: many(submissionSyncs),
  events: many(hackathonEvents),
}));

// =============================================================================
// JUDGING CRITERIA
// =============================================================================

export const judgingCriteria = pgTable(
  "judging_criteria",
  {
    id: text("id")
      .primaryKey()
      .$defaultFn(() => crypto.randomUUID()),
    hackathonId: text("hackathon_id")
      .notNull()
      .references(() => hackathons.id, { onDelete: "cascade" }),

    name: text("name").notNull(),
    description: text("description"),
    weight: integer("weight").notNull().default(1),
    maxScore: integer("max_score").notNull().default(10),
    order: integer("order").notNull(),

    createdAt: timestamp("created_at").defaultNow().notNull(),
  },
  (table) => [
    index("criteria_hackathon_idx").on(table.hackathonId, table.order),
  ]
);

export const judgingCriteriaRelations = relations(
  judgingCriteria,
  ({ one }) => ({
    hackathon: one(hackathons, {
      fields: [judgingCriteria.hackathonId],
      references: [hackathons.id],
    }),
  })
);

// =============================================================================
// HACKATHON ROLES
// =============================================================================

export const hackathonRoles = pgTable(
  "hackathon_roles",
  {
    id: text("id")
      .primaryKey()
      .$defaultFn(() => crypto.randomUUID()),

    hackathonId: text("hackathon_id")
      .notNull()
      .references(() => hackathons.id, { onDelete: "cascade" }),

    // Tenant ID from SSO (no FK)
    organizationId: text("organization_id").notNull(),
    userId: text("user_id").notNull(),

    // Denormalized user data (from SSO)
    username: text("username").notNull(),
    name: text("name"),
    email: text("email"),
    image: text("image"),

    role: text("role").notNull(), // organizer, manager, judge, mentor, participant

    // Invitation status (for roles that require acceptance)
    status: text("status").notNull().default("active"), // invited, active, declined

    // Mentor-specific
    maxTeamsAssigned: integer("max_teams_assigned"),

    createdAt: timestamp("created_at").defaultNow().notNull(),
    updatedAt: timestamp("updated_at")
      .defaultNow()
      .$onUpdate(() => new Date())
      .notNull(),
    assignedBy: text("assigned_by"),
  },
  (table) => [
    index("roles_org_idx").on(table.organizationId),
    index("roles_user_hackathon_idx").on(table.userId, table.hackathonId),
    index("roles_hackathon_role_idx").on(table.hackathonId, table.role),
    index("roles_username_idx").on(table.username),
    unique("roles_user_hackathon_role_unique").on(
      table.userId,
      table.hackathonId,
      table.role
    ),
  ]
);

export const hackathonRolesRelations = relations(hackathonRoles, ({ one }) => ({
  hackathon: one(hackathons, {
    fields: [hackathonRoles.hackathonId],
    references: [hackathons.id],
  }),
}));

// =============================================================================
// TEAMS
// =============================================================================

export const teams = pgTable(
  "teams",
  {
    id: text("id")
      .primaryKey()
      .$defaultFn(() => crypto.randomUUID()),

    hackathonId: text("hackathon_id")
      .notNull()
      .references(() => hackathons.id, { onDelete: "cascade" }),

    // Tenant ID from SSO (no FK)
    organizationId: text("organization_id").notNull(),

    name: text("name").notNull(),
    description: text("description"),
    inviteCode: text("invite_code")
      .notNull()
      .unique()
      .$defaultFn(() => generateInviteCode()),

    // Leader info (denormalized from SSO)
    leaderId: text("leader_id").notNull(),
    leaderUsername: text("leader_username").notNull(),
    leaderName: text("leader_name"),
    leaderImage: text("leader_image"),

    // Status
    status: text("status").notNull().default("forming"), // forming, ready, submitted, disqualified

    createdAt: timestamp("created_at").defaultNow().notNull(),
    updatedAt: timestamp("updated_at")
      .defaultNow()
      .$onUpdate(() => new Date())
      .notNull(),
  },
  (table) => [
    index("teams_org_idx").on(table.organizationId),
    index("teams_hackathon_idx").on(table.hackathonId),
    index("teams_invite_code_idx").on(table.inviteCode),
    index("teams_leader_idx").on(table.leaderId),
    unique("teams_name_hackathon_unique").on(table.hackathonId, table.name),
  ]
);

export const teamsRelations = relations(teams, ({ one, many }) => ({
  hackathon: one(hackathons, {
    fields: [teams.hackathonId],
    references: [hackathons.id],
  }),
  members: many(teamMembers),
  submission: one(submissions),
  messages: many(teamMessages),
}));

// =============================================================================
// TEAM MEMBERS
// =============================================================================

export const teamMembers = pgTable(
  "team_members",
  {
    id: text("id")
      .primaryKey()
      .$defaultFn(() => crypto.randomUUID()),

    teamId: text("team_id")
      .notNull()
      .references(() => teams.id, { onDelete: "cascade" }),

    // Tenant ID from SSO (no FK)
    organizationId: text("organization_id").notNull(),
    userId: text("user_id").notNull(),

    // Denormalized user data (from SSO)
    username: text("username").notNull(),
    name: text("name"),
    email: text("email"),
    image: text("image"),

    role: text("role").notNull().default("member"), // leader, member

    // Invitation status
    status: text("status").notNull().default("accepted"), // invited, accepted, declined

    invitedAt: timestamp("invited_at").defaultNow().notNull(),
    joinedAt: timestamp("joined_at"), // null until accepted
  },
  (table) => [
    index("members_org_idx").on(table.organizationId),
    index("members_team_idx").on(table.teamId),
    index("members_user_idx").on(table.userId),
    index("members_username_idx").on(table.username),
    unique("members_user_team_unique").on(table.userId, table.teamId),
  ]
);

export const teamMembersRelations = relations(teamMembers, ({ one }) => ({
  team: one(teams, {
    fields: [teamMembers.teamId],
    references: [teams.id],
  }),
}));

// =============================================================================
// SUBMISSIONS
// =============================================================================

export const submissions = pgTable(
  "submissions",
  {
    id: text("id")
      .primaryKey()
      .$defaultFn(() => crypto.randomUUID()),

    teamId: text("team_id")
      .notNull()
      .references(() => teams.id, { onDelete: "cascade" }),
    hackathonId: text("hackathon_id")
      .notNull()
      .references(() => hackathons.id, { onDelete: "cascade" }),

    // Tenant ID from SSO (no FK)
    organizationId: text("organization_id").notNull(),

    // Project Details
    projectName: text("project_name").notNull(),
    description: text("description").notNull(),
    repositoryUrl: text("repository_url").notNull(),
    demoUrl: text("demo_url"),
    presentationUrl: text("presentation_url"),

    // Category (matches category id from hackathon.categories, null if no categories)
    categoryId: text("category_id"),

    // Status
    status: text("status").notNull().default("submitted"), // submitted, under_review, scored

    // Submitter info (denormalized from SSO)
    submittedBy: text("submitted_by").notNull(),
    submitterUsername: text("submitter_username").notNull(),
    submitterName: text("submitter_name"),
    submitterEmail: text("submitter_email"), // For external form matching

    // Custom form data (JSON - responses to custom fields)
    formData: text("form_data"), // JSON object with custom field responses

    // External sync tracking
    syncedFromExternal: boolean("synced_from_external").notNull().default(false),
    syncedAt: timestamp("synced_at"),

    // Metadata
    submittedAt: timestamp("submitted_at").defaultNow().notNull(),
    updatedAt: timestamp("updated_at")
      .defaultNow()
      .$onUpdate(() => new Date())
      .notNull(),
  },
  (table) => [
    index("submissions_org_idx").on(table.organizationId),
    index("submissions_team_idx").on(table.teamId),
    index("submissions_hackathon_idx").on(table.hackathonId, table.status),
    index("submissions_email_idx").on(table.submitterEmail), // For sync matching
    unique("submissions_team_hackathon_unique").on(
      table.teamId,
      table.hackathonId
    ),
  ]
);

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

// =============================================================================
// SCORES
// =============================================================================

export const scores = pgTable(
  "scores",
  {
    id: text("id")
      .primaryKey()
      .$defaultFn(() => crypto.randomUUID()),

    submissionId: text("submission_id")
      .notNull()
      .references(() => submissions.id, { onDelete: "cascade" }),

    // Tenant ID from SSO (no FK)
    organizationId: text("organization_id").notNull(),

    criterionId: text("criterion_id")
      .notNull()
      .references(() => judgingCriteria.id, { onDelete: "cascade" }),

    // Judge info (denormalized from SSO)
    judgeId: text("judge_id").notNull(),
    judgeUsername: text("judge_username").notNull(),
    judgeName: text("judge_name"),

    score: integer("score").notNull(),
    feedback: text("feedback"),

    createdAt: timestamp("created_at").defaultNow().notNull(),
    updatedAt: timestamp("updated_at")
      .defaultNow()
      .$onUpdate(() => new Date())
      .notNull(),
  },
  (table) => [
    index("scores_org_idx").on(table.organizationId),
    index("scores_submission_idx").on(table.submissionId),
    index("scores_judge_idx").on(table.judgeId),
    unique("scores_judge_criterion_submission_unique").on(
      table.judgeId,
      table.criterionId,
      table.submissionId
    ),
  ]
);

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

// =============================================================================
// TEAM MESSAGES
// =============================================================================

export const teamMessages = pgTable(
  "team_messages",
  {
    id: text("id")
      .primaryKey()
      .$defaultFn(() => crypto.randomUUID()),

    teamId: text("team_id")
      .notNull()
      .references(() => teams.id, { onDelete: "cascade" }),

    // Tenant ID from SSO (no FK)
    organizationId: text("organization_id").notNull(),

    // Sender info (denormalized from SSO)
    senderId: text("sender_id").notNull(),
    senderUsername: text("sender_username").notNull(),
    senderName: text("sender_name"),
    senderImage: text("sender_image"),
    senderRole: text("sender_role"), // member, mentor

    message: text("message").notNull(),

    createdAt: timestamp("created_at").defaultNow().notNull(),
  },
  (table) => [
    index("messages_org_idx").on(table.organizationId),
    index("messages_team_idx").on(table.teamId, table.createdAt),
  ]
);

export const teamMessagesRelations = relations(teamMessages, ({ one }) => ({
  team: one(teams, {
    fields: [teamMessages.teamId],
    references: [teams.id],
  }),
}));

// =============================================================================
// WINNERS
// =============================================================================

export const winners = pgTable(
  "winners",
  {
    id: text("id")
      .primaryKey()
      .$defaultFn(() => crypto.randomUUID()),

    hackathonId: text("hackathon_id")
      .notNull()
      .references(() => hackathons.id, { onDelete: "cascade" }),

    // Tenant ID from SSO (no FK)
    organizationId: text("organization_id").notNull(),

    submissionId: text("submission_id")
      .notNull()
      .references(() => submissions.id, { onDelete: "cascade" }),
    teamId: text("team_id")
      .notNull()
      .references(() => teams.id, { onDelete: "cascade" }),

    // Prize info (matches prize from hackathon.prizes)
    place: text("place").notNull(), // "1st", "2nd", "3rd", or custom like "Best UI"
    prizeTitle: text("prize_title"), // "Grand Prize", "Runner Up", etc.
    prizeValue: text("prize_value"), // "$5,000", "MacBook Pro", etc.

    // Category (null = overall winner, otherwise category-specific winner)
    categoryId: text("category_id"),

    // Announcer info (denormalized from SSO)
    announcedBy: text("announced_by").notNull(),
    announcerUsername: text("announcer_username").notNull(),
    announcerName: text("announcer_name"),

    announcedAt: timestamp("announced_at").defaultNow().notNull(),
  },
  (table) => [
    index("winners_org_idx").on(table.organizationId),
    index("winners_hackathon_idx").on(table.hackathonId),
    index("winners_submission_idx").on(table.submissionId),
    unique("winners_hackathon_place_category_unique").on(
      table.hackathonId,
      table.place,
      table.categoryId
    ),
  ]
);

export const winnersRelations = relations(winners, ({ one }) => ({
  hackathon: one(hackathons, {
    fields: [winners.hackathonId],
    references: [hackathons.id],
  }),
  submission: one(submissions, {
    fields: [winners.submissionId],
    references: [submissions.id],
  }),
  team: one(teams, {
    fields: [winners.teamId],
    references: [teams.id],
  }),
}));

// =============================================================================
// SUBMISSION FIELDS (Custom Form Builder)
// =============================================================================

export const submissionFields = pgTable(
  "submission_fields",
  {
    id: text("id")
      .primaryKey()
      .$defaultFn(() => crypto.randomUUID()),

    hackathonId: text("hackathon_id")
      .notNull()
      .references(() => hackathons.id, { onDelete: "cascade" }),

    // Tenant ID from SSO (no FK)
    organizationId: text("organization_id").notNull(),

    // Field definition
    name: text("name").notNull(), // Machine name: "youtube_demo", "tech_stack"
    label: text("label").notNull(), // Display label: "YouTube Demo Link"
    type: text("type").notNull(), // text, url, textarea, select, number
    placeholder: text("placeholder"), // Placeholder text
    description: text("description"), // Help text below field

    // For select type
    options: text("options"), // JSON array: ["Option 1", "Option 2"]

    // Validation
    required: boolean("required").notNull().default(false),
    order: integer("order").notNull().default(0),

    createdAt: timestamp("created_at").defaultNow().notNull(),
    updatedAt: timestamp("updated_at")
      .defaultNow()
      .$onUpdate(() => new Date())
      .notNull(),
  },
  (table) => [
    index("submission_fields_hackathon_idx").on(table.hackathonId, table.order),
    index("submission_fields_org_idx").on(table.organizationId),
    unique("submission_fields_name_hackathon_unique").on(
      table.hackathonId,
      table.name
    ),
  ]
);

export const submissionFieldsRelations = relations(
  submissionFields,
  ({ one }) => ({
    hackathon: one(hackathons, {
      fields: [submissionFields.hackathonId],
      references: [hackathons.id],
    }),
  })
);

// =============================================================================
// SUBMISSION SYNCS (External Form Sync History)
// =============================================================================

export const submissionSyncs = pgTable(
  "submission_syncs",
  {
    id: text("id")
      .primaryKey()
      .$defaultFn(() => crypto.randomUUID()),

    hackathonId: text("hackathon_id")
      .notNull()
      .references(() => hackathons.id, { onDelete: "cascade" }),

    // Tenant ID from SSO (no FK)
    organizationId: text("organization_id").notNull(),

    // Sync details
    fileName: text("file_name"), // "responses.csv"
    totalRows: integer("total_rows").notNull().default(0),
    matched: integer("matched").notNull().default(0), // Found in our registrations
    unmatched: integer("unmatched").notNull().default(0), // In CSV but not registered
    created: integer("created").notNull().default(0), // New submissions created
    updated: integer("updated").notNull().default(0), // Existing submissions updated

    // Sync metadata
    syncedBy: text("synced_by").notNull(),
    syncedByUsername: text("synced_by_username").notNull(),
    syncedByName: text("synced_by_name"),

    syncedAt: timestamp("synced_at").defaultNow().notNull(),
  },
  (table) => [
    index("submission_syncs_hackathon_idx").on(table.hackathonId),
    index("submission_syncs_org_idx").on(table.organizationId),
  ]
);

export const submissionSyncsRelations = relations(
  submissionSyncs,
  ({ one }) => ({
    hackathon: one(hackathons, {
      fields: [submissionSyncs.hackathonId],
      references: [hackathons.id],
    }),
  })
);

// =============================================================================
// HACKATHON EVENTS (Livestreams, Workshops, Schedule)
// =============================================================================

export const hackathonEvents = pgTable(
  "hackathon_events",
  {
    id: text("id")
      .primaryKey()
      .$defaultFn(() => crypto.randomUUID()),

    hackathonId: text("hackathon_id")
      .notNull()
      .references(() => hackathons.id, { onDelete: "cascade" }),

    // Tenant ID from SSO (no FK)
    organizationId: text("organization_id").notNull(),

    // Event Details
    title: text("title").notNull(),
    description: text("description"),

    // Event Type: kickoff, workshop, qa_session, submission_deadline, judging, awards, livestream, networking, other
    eventType: text("event_type").notNull().default("other"),

    // Timing (stored in UTC, frontend displays in user timezone)
    startTime: timestamp("start_time").notNull(),
    endTime: timestamp("end_time"),

    // Location
    locationType: text("location_type").notNull().default("online"), // online, in_person, hybrid
    locationDetails: text("location_details"), // "San Francisco, CA" or "Discord #general"

    // Streaming & Recording
    livestreamUrl: text("livestream_url"), // Zoom, Meet, YouTube, Twitch link
    recordingUrl: text("recording_url"), // After event, the recording link

    // Optional speaker (links to hackathon_roles - judge/mentor/organizer)
    speakerId: text("speaker_id"),
    speakerName: text("speaker_name"), // Denormalized for display
    speakerTitle: text("speaker_title"), // "CEO at NativelyAI"

    // Display order for same-time events
    order: integer("order").notNull().default(0),

    // Published status
    published: boolean("published").notNull().default(true),

    createdAt: timestamp("created_at").defaultNow().notNull(),
    updatedAt: timestamp("updated_at")
      .defaultNow()
      .$onUpdate(() => new Date())
      .notNull(),
    createdBy: text("created_by").notNull(),
  },
  (table) => [
    index("events_hackathon_idx").on(table.hackathonId, table.startTime),
    index("events_org_idx").on(table.organizationId),
    index("events_type_idx").on(table.hackathonId, table.eventType),
  ]
);

export const hackathonEventsRelations = relations(
  hackathonEvents,
  ({ one }) => ({
    hackathon: one(hackathons, {
      fields: [hackathonEvents.hackathonId],
      references: [hackathons.id],
    }),
  })
);

// =============================================================================
// TYPE EXPORTS
// =============================================================================

export type Hackathon = typeof hackathons.$inferSelect;
export type NewHackathon = typeof hackathons.$inferInsert;

export type JudgingCriterion = typeof judgingCriteria.$inferSelect;
export type NewJudgingCriterion = typeof judgingCriteria.$inferInsert;

export type HackathonRole = typeof hackathonRoles.$inferSelect;
export type NewHackathonRole = typeof hackathonRoles.$inferInsert;

export type Team = typeof teams.$inferSelect;
export type NewTeam = typeof teams.$inferInsert;

export type TeamMember = typeof teamMembers.$inferSelect;
export type NewTeamMember = typeof teamMembers.$inferInsert;

export type Submission = typeof submissions.$inferSelect;
export type NewSubmission = typeof submissions.$inferInsert;

export type Score = typeof scores.$inferSelect;
export type NewScore = typeof scores.$inferInsert;

export type TeamMessage = typeof teamMessages.$inferSelect;
export type NewTeamMessage = typeof teamMessages.$inferInsert;

export type Winner = typeof winners.$inferSelect;
export type NewWinner = typeof winners.$inferInsert;

export type SubmissionField = typeof submissionFields.$inferSelect;
export type NewSubmissionField = typeof submissionFields.$inferInsert;

export type SubmissionSync = typeof submissionSyncs.$inferSelect;
export type NewSubmissionSync = typeof submissionSyncs.$inferInsert;

export type HackathonEvent = typeof hackathonEvents.$inferSelect;
export type NewHackathonEvent = typeof hackathonEvents.$inferInsert;
