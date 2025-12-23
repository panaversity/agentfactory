CREATE TABLE IF NOT EXISTS "submission_fields" (
	"id" text PRIMARY KEY NOT NULL,
	"hackathon_id" text NOT NULL,
	"organization_id" text NOT NULL,
	"name" text NOT NULL,
	"label" text NOT NULL,
	"type" text NOT NULL,
	"placeholder" text,
	"description" text,
	"options" text,
	"required" boolean DEFAULT false NOT NULL,
	"order" integer DEFAULT 0 NOT NULL,
	"created_at" timestamp DEFAULT now() NOT NULL,
	"updated_at" timestamp DEFAULT now() NOT NULL,
	CONSTRAINT "submission_fields_name_hackathon_unique" UNIQUE("hackathon_id","name")
);
--> statement-breakpoint
CREATE TABLE IF NOT EXISTS "submission_syncs" (
	"id" text PRIMARY KEY NOT NULL,
	"hackathon_id" text NOT NULL,
	"organization_id" text NOT NULL,
	"file_name" text,
	"total_rows" integer DEFAULT 0 NOT NULL,
	"matched" integer DEFAULT 0 NOT NULL,
	"unmatched" integer DEFAULT 0 NOT NULL,
	"created" integer DEFAULT 0 NOT NULL,
	"updated" integer DEFAULT 0 NOT NULL,
	"synced_by" text NOT NULL,
	"synced_by_username" text NOT NULL,
	"synced_by_name" text,
	"synced_at" timestamp DEFAULT now() NOT NULL
);
--> statement-breakpoint
ALTER TABLE "hackathons" ADD COLUMN "submission_mode" text DEFAULT 'platform' NOT NULL;--> statement-breakpoint
ALTER TABLE "hackathons" ADD COLUMN "external_form_url" text;--> statement-breakpoint
ALTER TABLE "submissions" ADD COLUMN "submitter_email" text;--> statement-breakpoint
ALTER TABLE "submissions" ADD COLUMN "form_data" text;--> statement-breakpoint
ALTER TABLE "submissions" ADD COLUMN "synced_from_external" boolean DEFAULT false NOT NULL;--> statement-breakpoint
ALTER TABLE "submissions" ADD COLUMN "synced_at" timestamp;--> statement-breakpoint
DO $$ BEGIN
 ALTER TABLE "submission_fields" ADD CONSTRAINT "submission_fields_hackathon_id_hackathons_id_fk" FOREIGN KEY ("hackathon_id") REFERENCES "public"."hackathons"("id") ON DELETE cascade ON UPDATE no action;
EXCEPTION
 WHEN duplicate_object THEN null;
END $$;
--> statement-breakpoint
DO $$ BEGIN
 ALTER TABLE "submission_syncs" ADD CONSTRAINT "submission_syncs_hackathon_id_hackathons_id_fk" FOREIGN KEY ("hackathon_id") REFERENCES "public"."hackathons"("id") ON DELETE cascade ON UPDATE no action;
EXCEPTION
 WHEN duplicate_object THEN null;
END $$;
--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "submission_fields_hackathon_idx" ON "submission_fields" USING btree ("hackathon_id","order");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "submission_fields_org_idx" ON "submission_fields" USING btree ("organization_id");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "submission_syncs_hackathon_idx" ON "submission_syncs" USING btree ("hackathon_id");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "submission_syncs_org_idx" ON "submission_syncs" USING btree ("organization_id");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "submissions_email_idx" ON "submissions" USING btree ("submitter_email");