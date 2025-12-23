import { drizzle } from "drizzle-orm/neon-serverless";
import { Pool, neonConfig } from "@neondatabase/serverless";
import * as schema from "./schema";

// Enable connection pooling for serverless environments
// Use DATABASE_URL with Neon's connection pooler (-pooler suffix)
// Example: postgres://user:pass@ep-xxx-pooler.region.aws.neon.tech/dbname
neonConfig.fetchConnectionCache = true;

const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  max: 10, // Limit connections per serverless instance
  idleTimeoutMillis: 30000, // Close idle connections after 30s
  connectionTimeoutMillis: 10000, // Timeout after 10s if can't connect
});

export const db = drizzle(pool, { schema });
