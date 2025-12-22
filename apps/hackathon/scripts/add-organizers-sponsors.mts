import { neon } from '@neondatabase/serverless';
import { config } from 'dotenv';

// Load from .env.local
config({ path: '.env.local' });

const sql = neon(process.env.DATABASE_URL!);

async function migrate() {
  console.log('Adding organizers and sponsors columns...');

  await sql`ALTER TABLE hackathons ADD COLUMN IF NOT EXISTS organizers text`;
  await sql`ALTER TABLE hackathons ADD COLUMN IF NOT EXISTS sponsors text`;

  console.log('Migration complete!');
}

migrate().catch(console.error);
