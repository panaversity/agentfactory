import * as schema from "@/auth-schema"; // Use Better Auth generated schema

// Get database URL from environment
// Production: DATABASE_URL must be set
// Build time: Uses placeholder to prevent build failures
// Development: Falls back to local postgres for testing
const getDatabaseUrl = (): string => {
  if (process.env.DATABASE_URL) {
    return process.env.DATABASE_URL;
  }
  // During build, Next.js imports this file - allow placeholder
  if (process.env.NODE_ENV === "production" && !process.env.NEXT_PHASE) {
    throw new Error("DATABASE_URL must be set in production");
  }
  // Development/build placeholder
  return "postgresql://placeholder:placeholder@localhost:5432/placeholder";
};

/**
 * Initialize database connection with environment-aware driver selection
 *
 * - Production/Neon: Uses @neondatabase/serverless (HTTP/WebSockets)
 * - CI/Standard PostgreSQL: Uses postgres (TCP)
 *
 * Detection: If DATABASE_URL contains "localhost" or "127.0.0.1", use standard PostgreSQL
 */
function initDatabase() {
  const databaseUrl = getDatabaseUrl();
  const isLocalPostgres =
    databaseUrl.includes("localhost") || databaseUrl.includes("127.0.0.1");

  if (isLocalPostgres) {
    // Standard PostgreSQL (CI, local development with standard Postgres)
    const postgres = require("postgres");
    const { drizzle } = require("drizzle-orm/postgres-js");
    const sql = postgres(databaseUrl);
    return drizzle(sql, { schema });
  } else {
    // Neon serverless (production, staging)
    const { neon } = require("@neondatabase/serverless");
    const { drizzle } = require("drizzle-orm/neon-http");
    const sql = neon(databaseUrl);
    return drizzle(sql, { schema });
  }
}

export const db = initDatabase();

export type Database = typeof db;
