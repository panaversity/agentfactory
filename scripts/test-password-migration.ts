#!/usr/bin/env node
/**
 * Test Password Migration Script
 *
 * This script migrates a few test users from NextAuth DB to Better Auth DB
 * to verify that bcrypt password verification works correctly.
 *
 * Usage: npx tsx scripts/test-password-migration.ts
 */

import * as dotenv from 'dotenv';
import { pgTable, text, boolean, timestamp } from 'drizzle-orm/pg-core';
import { eq } from 'drizzle-orm';

// Load environment variables
dotenv.config();
dotenv.config({ path: '.env.local' });

// Schema definitions (inline to avoid import issues)
const user = pgTable("user", {
  id: text("id").primaryKey(),
  name: text("name").notNull(),
  email: text("email").notNull().unique(),
  emailVerified: boolean("email_verified").default(false).notNull(),
  image: text("image"),
  createdAt: timestamp("created_at").defaultNow().notNull(),
  updatedAt: timestamp("updated_at").defaultNow().notNull(),
  role: text("role"),
  banned: boolean("banned").default(false),
  username: text("username").unique(),
  givenName: text("given_name"),
  familyName: text("family_name"),
  phoneNumber: text("phone_number"),
  gender: text("gender"),
  fatherName: text("father_name"),
  city: text("city"),
  country: text("country"),
});

const account = pgTable("account", {
  id: text("id").primaryKey(),
  accountId: text("account_id").notNull(),
  providerId: text("provider_id").notNull(),
  userId: text("user_id").notNull(),
  password: text("password"),
  createdAt: timestamp("created_at").defaultNow().notNull(),
  updatedAt: timestamp("updated_at").defaultNow().notNull(),
});

const OLD_DB_URL = process.env.NEXT_AUTH_PROD_DB_MIRROR;
const NEW_DB_URL = process.env.BETTER_AUT_NEW_DB_MIRROR;

// Database connection holders (initialized in main)
let oldSql: any;
let newDb: any;

/**
 * Initialize database connections using dynamic imports
 */
async function initDatabases() {
  if (!OLD_DB_URL || !NEW_DB_URL) {
    console.error('Missing required environment variables:');
    console.error('- NEXT_AUTH_PROD_DB_MIRROR:', OLD_DB_URL ? 'set' : 'MISSING');
    console.error('- BETTER_AUT_NEW_DB_MIRROR:', NEW_DB_URL ? 'set' : 'MISSING');
    process.exit(1);
  }

  // Use Neon serverless for both DBs
  const { neon } = await import('@neondatabase/serverless');
  const { drizzle } = await import('drizzle-orm/neon-http');

  oldSql = neon(OLD_DB_URL);
  const newSql = neon(NEW_DB_URL);
  newDb = drizzle(newSql);

  console.log('Connected to both databases');
}

interface OldUser {
  id: string;
  name: string | null;
  email: string;
  password: string | null;
  phone_number: string | null;
  emailverified: boolean;
  image: string | null;
  country: string | null;
  role: string | null;
  username: string | null;
  created_at: Date;
  updated_at: Date;
}

interface OldProfile {
  city: string | null;
  gender: string | null;
  father_name: string | null;
}

async function getTestUsersFromOldDb(): Promise<(OldUser & OldProfile)[]> {
  // Get 3 users with passwords that don't exist in new DB
  const result = await oldSql`
    SELECT
      u.id, u.name, u.email, u.password, u.phone_number,
      u."emailVerified" as emailverified, u.image, u.country, u.role, u.username,
      u.created_at, u.updated_at,
      p.city, p.gender, p.father_name
    FROM public.users u
    LEFT JOIN public.profile p ON u.id = p.user_id
    WHERE u.password IS NOT NULL
      AND u.password != ''
    LIMIT 3
  `;

  return result as (OldUser & OldProfile)[];
}

async function checkUserExistsInNewDb(email: string): Promise<boolean> {
  try {
    // Debug: test raw query first
    const { neon } = await import('@neondatabase/serverless');
    const sql = neon(NEW_DB_URL!);
    const rawResult = await sql`SELECT COUNT(*) as count FROM public."user" WHERE email = ${email}`;
    console.log(`  [DEBUG] Raw query found: ${rawResult[0]?.count} users with email ${email}`);

    return Number(rawResult[0]?.count || 0) > 0;
  } catch (err: any) {
    console.error(`  [DEBUG] Error checking user: ${err.message}`);
    throw err;
  }
}

async function migrateUser(oldUser: OldUser & OldProfile): Promise<{ success: boolean; message: string }> {
  const email = oldUser.email;

  // Check if user already exists
  const exists = await checkUserExistsInNewDb(email);
  if (exists) {
    return { success: false, message: `User ${email} already exists in new DB` };
  }

  if (!oldUser.password) {
    return { success: false, message: `User ${email} has no password` };
  }

  try {
    const { neon } = await import('@neondatabase/serverless');
    const sql = neon(NEW_DB_URL!);

    const userId = oldUser.id;
    const accountId = crypto.randomUUID();
    const now = new Date().toISOString();

    // Handle date conversion (dates may come as strings from neon)
    const createdAt = oldUser.created_at
      ? (oldUser.created_at instanceof Date ? oldUser.created_at.toISOString() : String(oldUser.created_at))
      : now;
    const updatedAt = oldUser.updated_at
      ? (oldUser.updated_at instanceof Date ? oldUser.updated_at.toISOString() : String(oldUser.updated_at))
      : now;

    // Insert user into Better Auth user table using raw SQL
    // Note: city, gender, father_name, country columns not yet in test DB - will be added later
    await sql`
      INSERT INTO public."user" (
        id, name, email, email_verified, image, created_at, updated_at,
        role, username, phone_number
      ) VALUES (
        ${userId},
        ${oldUser.name || 'Unknown'},
        ${oldUser.email},
        ${oldUser.emailverified || false},
        ${oldUser.image || null},
        ${createdAt},
        ${updatedAt},
        ${oldUser.role || 'user'},
        ${oldUser.username || null},
        ${oldUser.phone_number || null}
      )
    `;

    // Insert credential account with bcrypt password
    await sql`
      INSERT INTO public.account (
        id, account_id, provider_id, user_id, password, created_at, updated_at
      ) VALUES (
        ${accountId},
        ${userId},
        ${'credential'},
        ${userId},
        ${oldUser.password},
        ${now},
        ${now}
      )
    `;

    return {
      success: true,
      message: `Migrated user ${email} (ID: ${userId}) with bcrypt password`
    };
  } catch (error: any) {
    return {
      success: false,
      message: `Failed to migrate ${email}: ${error.message}`
    };
  }
}

async function main() {
  console.log('=== Test Password Migration ===\n');
  console.log('Old DB (NextAuth):', OLD_DB_URL?.substring(0, 50) + '...');
  console.log('New DB (Better Auth):', NEW_DB_URL?.substring(0, 50) + '...');

  // Initialize database connections
  await initDatabases();

  console.log('\n--- Fetching test users from old DB ---\n');

  const testUsers = await getTestUsersFromOldDb();

  console.log(`Found ${testUsers.length} users to migrate:\n`);

  for (const oldUser of testUsers) {
    console.log(`- ${oldUser.email}`);
    console.log(`  ID: ${oldUser.id}`);
    console.log(`  Password hash: ${oldUser.password?.substring(0, 30)}...`);
    console.log(`  Profile: city=${oldUser.city || 'null'}, gender=${oldUser.gender || 'null'}`);
  }

  console.log('\n--- Starting migration ---\n');

  const results: { email: string; success: boolean; message: string }[] = [];

  for (const oldUser of testUsers) {
    const result = await migrateUser(oldUser);
    results.push({ email: oldUser.email, ...result });

    if (result.success) {
      console.log(`[OK] ${result.message}`);
    } else {
      console.log(`[SKIP] ${result.message}`);
    }
  }

  console.log('\n--- Migration Summary ---\n');
  const successful = results.filter(r => r.success);
  const skipped = results.filter(r => !r.success);

  console.log(`Migrated: ${successful.length}`);
  console.log(`Skipped: ${skipped.length}`);

  if (successful.length > 0) {
    console.log('\n--- Next Steps ---\n');
    console.log('1. Start the dev server: pnpm dev');
    console.log('2. Try logging in with one of these emails:');
    for (const r of successful) {
      console.log(`   - ${r.email}`);
    }
    console.log('3. Use the password from the old NextAuth system');
    console.log('\nThe bcrypt password should be verified by the custom verify function in auth.ts');
  }
}

main().catch(console.error);
