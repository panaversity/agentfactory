import { neon } from "@neondatabase/serverless";
import { drizzle } from "drizzle-orm/neon-http";
import * as schema from "./schema";

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

const sql = neon(getDatabaseUrl());

export const db = drizzle(sql, { schema });

export type Database = typeof db;
