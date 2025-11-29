import { neon } from "@neondatabase/serverless";
import { drizzle } from "drizzle-orm/neon-http";
import * as schema from "./schema";

// Handle missing DATABASE_URL gracefully during build
const databaseUrl = process.env.DATABASE_URL || "postgresql://placeholder:placeholder@localhost:5432/placeholder";
const sql = neon(databaseUrl);

export const db = drizzle(sql, { schema });

export type Database = typeof db;
