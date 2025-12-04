import { db } from "../src/lib/db";
import { user, account } from "../auth-schema";
import { eq } from "drizzle-orm";

// Admin credentials - must match test files
const ADMIN_EMAIL = "admin@robolearn.io";
const ADMIN_PASSWORD = "admin@robolearn.io";
const ADMIN_NAME = "Test Admin";
const SSO_URL = process.env.BETTER_AUTH_URL || "http://localhost:3001";

async function recreateAdmin() {
  console.log("Recreating admin user with correct password hash...\n");

  try {
    // Step 1: Delete existing admin user (cascade will delete account)
    console.log("Step 1: Deleting existing admin user...");
    const deleted = await db.delete(user).where(eq(user.email, ADMIN_EMAIL));
    console.log("✅ Deleted existing admin (if any)\n");

    // Step 2: Create admin via sign-up API (uses correct scrypt hashing)
    console.log("Step 2: Creating admin via sign-up API...");
    const signUpResponse = await fetch(`${SSO_URL}/api/auth/sign-up/email`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        email: ADMIN_EMAIL,
        password: ADMIN_PASSWORD,
        name: ADMIN_NAME,
      }),
    });

    if (!signUpResponse.ok) {
      const error = await signUpResponse.json();
      throw new Error(`Sign-up failed: ${JSON.stringify(error)}`);
    }

    console.log("✅ Admin created via sign-up API\n");

    // Step 3: Update role to admin and verify email
    console.log("Step 3: Setting admin role and verifying email...");
    await db
      .update(user)
      .set({
        role: "admin",
        emailVerified: true
      })
      .where(eq(user.email, ADMIN_EMAIL));

    console.log("✅ Admin role and email verification set\n");

    console.log("========================================");
    console.log("Admin user recreated successfully!");
    console.log("========================================");
    console.log(`  Email: ${ADMIN_EMAIL}`);
    console.log(`  Password: ${ADMIN_PASSWORD}`);
    console.log(`  Role: admin`);
    console.log(`  Email Verified: true`);
    console.log("========================================\n");

  } catch (error) {
    console.error("❌ Error:", error);
    process.exit(1);
  }

  process.exit(0);
}

recreateAdmin();
