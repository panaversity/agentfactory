#!/usr/bin/env npx tsx
/**
 * Admin password update script
 *
 * Usage: DATABASE_URL=<url> npx tsx scripts/update-password.ts <email> <password>
 *
 * Security:
 * - Validates email format
 * - Requires minimum 8 character password
 * - Hashes password with bcrypt (cost 10)
 * - Requires DATABASE_URL environment variable
 */

import { neon } from "@neondatabase/serverless";
import bcrypt from "bcryptjs";

/**
 * Basic email format validation
 */
function isValidEmail(email: string): boolean {
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  return emailRegex.test(email);
}

async function main() {
  const [email, newPassword] = process.argv.slice(2);

  if (!email || !newPassword) {
    console.error("Usage: npx tsx scripts/update-password.ts <email> <password>");
    console.error("       Requires DATABASE_URL environment variable");
    process.exit(1);
  }

  // Validate email format
  if (!isValidEmail(email)) {
    console.error("Invalid email format");
    process.exit(1);
  }

  if (newPassword.length < 8) {
    console.error("Password must be at least 8 characters");
    process.exit(1);
  }

  const databaseUrl = process.env.DATABASE_URL;
  if (!databaseUrl) {
    console.error("DATABASE_URL environment variable not set");
    process.exit(1);
  }

  const sql = neon(databaseUrl);

  // Find user (parameterized query - safe from SQL injection)
  const users = await sql`SELECT id, email, role FROM public."user" WHERE email = ${email}`;
  if (users.length === 0) {
    console.error(`User not found: ${email}`);
    process.exit(1);
  }

  const user = users[0];
  console.log(`Found user: ${user.email} (role: ${user.role})`);

  // Hash new password with bcrypt
  const hashedPassword = await bcrypt.hash(newPassword, 10);

  // Update password using database timestamp for consistency
  const result = await sql`
    UPDATE public.account
    SET password = ${hashedPassword}, updated_at = NOW()
    WHERE user_id = ${user.id} AND provider_id = 'credential'
    RETURNING user_id
  `;

  if (result.length === 0) {
    console.error("No credential account found for user");
    process.exit(1);
  }

  console.log(`Password updated for ${email}`);
}

main().catch(console.error);
