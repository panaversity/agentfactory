CREATE TABLE IF NOT EXISTS "hackathon_events" (
	"id" text PRIMARY KEY NOT NULL,
	"hackathon_id" text NOT NULL,
	"organization_id" text NOT NULL,
	"title" text NOT NULL,
	"description" text,
	"event_type" text DEFAULT 'other' NOT NULL,
	"start_time" timestamp NOT NULL,
	"end_time" timestamp,
	"location_type" text DEFAULT 'online' NOT NULL,
	"location_details" text,
	"livestream_url" text,
	"recording_url" text,
	"speaker_id" text,
	"speaker_name" text,
	"speaker_title" text,
	"order" integer DEFAULT 0 NOT NULL,
	"published" boolean DEFAULT true NOT NULL,
	"created_at" timestamp DEFAULT now() NOT NULL,
	"updated_at" timestamp DEFAULT now() NOT NULL,
	"created_by" text NOT NULL
);
--> statement-breakpoint
DO $$ BEGIN
 ALTER TABLE "hackathon_events" ADD CONSTRAINT "hackathon_events_hackathon_id_hackathons_id_fk" FOREIGN KEY ("hackathon_id") REFERENCES "public"."hackathons"("id") ON DELETE cascade ON UPDATE no action;
EXCEPTION
 WHEN duplicate_object THEN null;
END $$;
--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "events_hackathon_idx" ON "hackathon_events" USING btree ("hackathon_id","start_time");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "events_org_idx" ON "hackathon_events" USING btree ("organization_id");--> statement-breakpoint
CREATE INDEX IF NOT EXISTS "events_type_idx" ON "hackathon_events" USING btree ("hackathon_id","event_type");