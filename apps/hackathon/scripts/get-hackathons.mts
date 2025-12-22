import { neon } from '@neondatabase/serverless';
import { config } from 'dotenv';

config({ path: '.env.local' });

const sql = neon(process.env.DATABASE_URL!);

async function main() {
  const result = await sql`SELECT id, title, published, organizers, sponsors FROM hackathons LIMIT 5`;
  console.log(JSON.stringify(result, null, 2));
}

main().catch(console.error);
