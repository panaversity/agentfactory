CREATE TABLE IF NOT EXISTS "hackathon_roles" (
	"id" text PRIMARY KEY NOT NULL,
	"hackathon_id" text NOT NULL,
	"organization_id" text NOT NULL,
	"user_id" text NOT NULL,
	"username" text NOT NULL,
	"name" text,
	"email" text,
	"image" text,
	"role" text NOT NULL,
	"status" text DEFAULT 'active' NOT NULL,
	"max_teams_assigned" integer,
	"created_at" timestamp DEFAULT now() NOT NULL,
	"updated_at" timestamp DEFAULT now() NOT NULL,
	"assigned_by" text,
	CONSTRAINT "roles_user_hackathon_role_unique" UNIQUE("user_id","hackathon_id","role")
);
--> statement-breakpoint
CREATE TABLE IF NOT EXISTS "hackathons" (
	"id" text PRIMARY KEY NOT NULL,
	"organization_id" text NOT NULL,
	"title" text NOT NULL,
	"description" text NOT NULL,
	"slug" text NOT NULL,
	"start_date" timestamp NOT NULL,
	"end_date" timestamp NOT NULL,
	"registration_deadline" timestamp NOT NULL,
	"submission_deadline" timestamp NOT NULL,
	"min_team_size" integer DEFAULT 1 NOT NULL,
	"max_team_size" integer DEFAULT 5 NOT NULL,
	"prizes" text,
	"organizers" text,
	"sponsors" text,
	"categories" text,
	"status" text DEFAULT 'draft' NOT NULL,
	"published" boolean DEFAULT false NOT NULL,
	"created_at" timestamp DEFAULT now() NOT NULL,
	"updated_at" timestamp DEFAULT now() NOT NULL,
	"created_by" text NOT NULL,
	CONSTRAINT "hackathons_org_slug_unique" UNIQUE("organization_id","slug")
);
--> statement-breakpoint
CREATE TABLE IF NOT EXISTS "judging_criteria" (
	"id" text PRIMARY KEY NOT NULL,
	"hackathon_id" text NOT NULL,
	"name" text NOT NULL,
	"description" text,
	"weight" integer DEFAULT 1 NOT NULL,
	"max_score" integer DEFAULT 10 NOT NULL,
	"order" integer NOT NULL,
	"created_at" timestamp DEFAULT now() NOT NULL
);
--> statement-breakpoint
CREATE TABLE IF NOT EXISTS "scores" (
	"id" text PRIMARY KEY NOT NULL,
	"submission_id" text NOT NULL,
	"organization_id" text NOT NULL,
	"criterion_id" text NOT NULL,
	"judge_id" text NOT NULL,
	"judge_username" text NOT NULL,
	"judge_name" text,
	"score" integer NOT NULL,
	"feedback" text,
	"created_at" timestamp DEFAULT now() NOT NULL,
	"updated_at" timestamp DEFAULT now() NOT NULL,
	CONSTRAINT "scores_judge_criterion_submission_unique" UNIQUE("judge_id","criterion_id","submission_id")
);
--> statement-breakpoint
CREATE TABLE IF NOT EXISTS "submissions" (
	"id" text PRIMARY KEY NOT NULL,
	"team_id" text NOT NULL,
	"hackathon_id" text NOT NULL,
	"organization_id" text NOT NULL,
	"project_name" text NOT NULL,
	"description" text NOT NULL,
	"repository_url" text NOT NULL,
	"demo_url" text,
	"presentation_url" text,
	"category_id" text,
	"status" text DEFAULT 'submitted' NOT NULL,
	"submitted_by" text NOT NULL,
	"submitter_username" text NOT NULL,
	"submitter_name" text,
	"submitted_at" timestamp DEFAULT now() NOT NULL,
	"updated_at" timestamp DEFAULT now() NOT NULL,
	CONSTRAINT "submissions_team_hackathon_unique" UNIQUE("team_id","hackathon_id")
);
--> statement-breakpoint
CREATE TABLE IF NOT EXISTS "team_members" (
	"id" text PRIMARY KEY NOT NULL,
	"team_id" text NOT NULL,
	"organization_id" text NOT NULL,
	"user_id" text NOT NULL,
	"username" text NOT NULL,
	"name" text,
	"email" text,
	"image" text,
	"role" text DEFAULT 'member' NOT NULL,
	"status" text DEFAULT 'accepted' NOT NULL,
	"invited_at" timestamp DEFAULT now() NOT NULL,
	"joined_at" timestamp,
	CONSTRAINT "members_user_team_unique" UNIQUE("user_id","team_id")
);
--> statement-breakpoint
CREATE TABLE IF NOT EXISTS "team_messages" (
	"id" text PRIMARY KEY NOT NULL,
	"team_id" text NOT NULL,
	"organization_id" text NOT NULL,
	"sender_id" text NOT NULL,
	"sender_username" text NOT NULL,
	"sender_name" text,
	"sender_image" text,
	"sender_role" text,
	"message" text NOT NULL,
	"created_at" timestamp DEFAULT now() NOT NULL
);
--> statement-breakpoint
CREATE TABLE IF NOT EXISTS "teams" (
	"id" text PRIMARY KEY NOT NULL,
	"hackathon_id" text NOT NULL,
	"organization_id" text NOT NULL,
	"name" text NOT NULL,
	"description" text,
	"invite_code" text NOT NULL,
	"leader_id" text NOT NULL,
	"leader_username" text NOT NULL,
	"leader_name" text,
	"leader_image" text,
	"status" text DEFAULT 'forming' NOT NULL,
	"created_at" timestamp DEFAULT now() NOT NULL,
	"updated_at" timestamp DEFAULT now() NOT NULL,
	CONSTRAINT "teams_invite_code_unique" UNIQUE("invite_code"),
	CONSTRAINT "teams_name_hackathon_unique" UNIQUE("hackathon_id","name")
);
--> statement-breakpoint
CREATE TABLE IF NOT EXISTS "winners" (
	"id" text PRIMARY KEY NOT NULL,
	"hackathon_id" text NOT NULL,
	"organization_id" text NOT NULL,
	"submission_id" text NOT NULL,
	"team_id" text NOT NULL,
	"place" text NOT NULL,
	"prize_title" text,
	"prize_value" text,
	"category_id" text,
	"announced_by" text NOT NULL,
	"announcer_username" text NOT NULL,
	"announcer_name" text,
	"announced_at" timestamp DEFAULT now() NOT NULL,
	CONSTRAINT "winners_hackathon_place_category_unique" UNIQUE("hackathon_id","place","category_id")
);
--> statement-breakpoint
DO $$ BEGIN
 ALTER TABLE "hackathon_roles" ADD CONSTRAINT "hackathon_roles_hackathon_id_hackathons_id_fk" FOREIGN KEY ("hackathon_id") REFERENCES "public"."hackathons"("id") ON DELETE cascade ON UPDATE no action;
EXCEPTION
 WHEN duplicate_object THEN null;
END $$;
--> statement-breakpoint
DO $$ BEGIN
 ALTER TABLE "judging_criteria" ADD CONSTRAINT "judging_criteria_hackathon_id_hackathons_id_fk" FOREIGN KEY ("hackathon_id") REFERENCES "public"."hackathons"("id") ON DELETE cascade ON UPDATE no action;
EXCEPTION
 WHEN duplicate_object THEN null;
END $$;
--> statement-breakpoint
DO $$ BEGIN
 ALTER TABLE "scores" ADD CONSTRAINT "scores_submission_id_submissions_id_fk" FOREIGN KEY ("submission_id") REFERENCES "public"."submissions"("id") ON DELETE cascade ON UPDATE no action;
EXCEPTION
 WHEN duplicate_object THEN null;
END $$;
--> statement-breakpoint
DO $$ BEGIN
 ALTER TABLE "scores" ADD CONSTRAINT "scores_criterion_id_judging_criteria_id_fk" FOREIGN KEY ("criterion_id") REFERENCES "public"."judging_criteria"("id") ON DELETE cascade ON UPDATE no action;
EXCEPTION
 WHEN duplicate_object THEN null;
END $$;
--> statement-breakpoint
DO $$ BEGIN
 ALTER TABLE "submissions" ADD CONSTRAINT "submissions_team_id_teams_id_fk" FOREIGN KEY ("team_id") REFERENCES "public"."teams"("id") ON DELETE cascade ON UPDATE no action;
EXCEPTION
 WHEN duplicate_object THEN null;
END $$;
--> statement-breakpoint
DO $$ BEGIN
 ALTER TABLE "submissions" ADD CONSTRAINT "submissions_hackathon_id_hackathons_id_fk" FOREIGN KEY ("hackathon_id") REFERENCES "public"."hackathons"("id") ON DELETE cascade ON UPDATE no action;
EXCEPTION
 WHEN duplicate_object THEN null;
END $$;
--> statement-breakpoint
DO $$ BEGIN
 ALTER TABLE "team_members" ADD CONSTRAINT "team_members_team_id_teams_id_fk" FOREIGN KEY ("team_id") REFERENCES "public"."teams"("id") ON DELETE cascade ON UPDATE no action;
EXCEPTION
 WHEN duplicate_object THEN null;
END $$;
--> statement-breakpoint
DO $$ BEGIN
 ALTER TABLE "team_messages" ADD CONSTRAINT "team_messages_team_id_teams_id_fk" FOREIGN KEY ("team_id") REFERENCES "public"."teams"("id") ON DELETE cascade ON UPDATE no action;
EXCEPTION
 WHEN duplicate_object THEN null;
END $$;
--> statement-breakpoint
DO $$ BEGIN
 ALTER TABLE "teams" ADD CONSTRAINT "teams_hackathon_id_hackathons_id_fk" FOREIGN KEY ("hackathon_id") REFERENCES "public"."hackathons"("id") ON DELETE cascade ON UPDATE no action;
EXCEPTION
 WHEN duplicate_object THEN null;
END $$;
--> statement-breakpoint
DO $$ BEGIN
 ALTER TABLE "winners" ADD CONSTRAINT "winners_hackathon_id_hackathons_id_fk" FOREIGN KEY ("hackathon_id") REFERENCES "public"."hackathons"("id") ON DELETE cascade ON UPDATE no action;
EXCEPTION
 WHEN duplicate_object THEN null;
END $$;
--> statement-breakpoint
DO $$ BEGIN
 ALTER TABLE "winners" ADD CONSTRAINT "winners_submission_id_submissions_id_fk" FOREIGN KEY ("submission_id") REFERENCES "public"."submissions"("id") ON DELETE cascade ON UPDATE no action;
EXCEPTION
 WHEN duplicate_object THEN null;
END $$;
--> statement-breakpoint
DO $$ BEGIN
 ALTER TABLE "winners" ADD CONSTRAINT "winners_team_id_teams_id_fk" FOREIGN KEY ("team_id") REFERENCES "public"."teams"("id") ON DELETE cascade ON UPDATE no action;
EXCEPTION
 WHEN duplicate_object THEN null;
END $$;
--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "roles_org_idx" ON "hackathon_roles" USING btree ("organization_id");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "roles_user_hackathon_idx" ON "hackathon_roles" USING btree ("user_id","hackathon_id");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "roles_hackathon_role_idx" ON "hackathon_roles" USING btree ("hackathon_id","role");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "roles_username_idx" ON "hackathon_roles" USING btree ("username");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "hackathons_org_idx" ON "hackathons" USING btree ("organization_id");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "hackathons_slug_idx" ON "hackathons" USING btree ("slug");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "hackathons_status_idx" ON "hackathons" USING btree ("status","published");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "criteria_hackathon_idx" ON "judging_criteria" USING btree ("hackathon_id","order");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "scores_org_idx" ON "scores" USING btree ("organization_id");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "scores_submission_idx" ON "scores" USING btree ("submission_id");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "scores_judge_idx" ON "scores" USING btree ("judge_id");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "submissions_org_idx" ON "submissions" USING btree ("organization_id");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "submissions_team_idx" ON "submissions" USING btree ("team_id");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "submissions_hackathon_idx" ON "submissions" USING btree ("hackathon_id","status");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "members_org_idx" ON "team_members" USING btree ("organization_id");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "members_team_idx" ON "team_members" USING btree ("team_id");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "members_user_idx" ON "team_members" USING btree ("user_id");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "members_username_idx" ON "team_members" USING btree ("username");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "messages_org_idx" ON "team_messages" USING btree ("organization_id");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "messages_team_idx" ON "team_messages" USING btree ("team_id","created_at");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "teams_org_idx" ON "teams" USING btree ("organization_id");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "teams_hackathon_idx" ON "teams" USING btree ("hackathon_id");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "teams_invite_code_idx" ON "teams" USING btree ("invite_code");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "teams_leader_idx" ON "teams" USING btree ("leader_id");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "winners_org_idx" ON "winners" USING btree ("organization_id");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "winners_hackathon_idx" ON "winners" USING btree ("hackathon_id");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "winners_submission_idx" ON "winners" USING btree ("submission_id");