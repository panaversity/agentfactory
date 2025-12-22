import { neon } from '@neondatabase/serverless';
import { config } from 'dotenv';

config({ path: '.env.local' });

const sql = neon(process.env.DATABASE_URL!);

async function migrate() {
  console.log('Adding categories to hackathons...');
  await sql`ALTER TABLE hackathons ADD COLUMN IF NOT EXISTS categories text`;

  console.log('Adding category_id to submissions...');
  await sql`ALTER TABLE submissions ADD COLUMN IF NOT EXISTS category_id text`;

  console.log('Creating winners table...');
  await sql`
    CREATE TABLE IF NOT EXISTS winners (
      id text PRIMARY KEY,
      hackathon_id text NOT NULL REFERENCES hackathons(id) ON DELETE CASCADE,
      submission_id text NOT NULL REFERENCES submissions(id) ON DELETE CASCADE,
      team_id text NOT NULL REFERENCES teams(id) ON DELETE CASCADE,
      place text NOT NULL,
      prize_title text,
      prize_value text,
      category_id text,
      announced_at timestamp DEFAULT now() NOT NULL,
      announced_by text NOT NULL
    )
  `;

  console.log('Creating indexes for winners table...');
  await sql`CREATE INDEX IF NOT EXISTS winners_hackathon_idx ON winners(hackathon_id)`;
  await sql`CREATE INDEX IF NOT EXISTS winners_submission_idx ON winners(submission_id)`;
  await sql`CREATE UNIQUE INDEX IF NOT EXISTS winners_hackathon_place_category_unique ON winners(hackathon_id, place, category_id)`;

  console.log('Migration complete!');
}

migrate().catch(console.error);
