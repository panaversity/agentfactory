import { db } from "../src/lib/db";
import { user } from "../auth-schema";
import { eq } from "drizzle-orm";

// Admin credentials - must match seed-setup.ts and test files
const ADMIN_EMAIL = "admin@robolearn.io";
const ADMIN_PASSWORD = "admin@robolearn.io"; // Strong password (matches seed-setup.ts)
const ADMIN_NAME = "Test Admin";
const SSO_URL = process.env.BETTER_AUTH_URL || "http://localhost:3001";

/**
 * Creates admin user via Better Auth sign-up API.
 * This ensures password is hashed using Better Auth's exact implementation (scrypt).
 */
async function createAdminUser() {
  console.log("Creating admin user...");
  console.log(`SSO URL: ${SSO_URL}\n`);

  try {
    // Check if admin already exists
    const existingUser = await db.query.user.findFirst({
      where: eq(user.email, ADMIN_EMAIL),
    });

    if (existingUser) {
      console.log("Admin user already exists, updating role to admin...");
      await db
        .update(user)
        .set({ role: "admin", emailVerified: true })
        .where(eq(user.email, ADMIN_EMAIL));
      console.log("✅ Admin role and email verification updated!");
      console.log(`\nAdmin Credentials:`);
      console.log(`  Email: ${ADMIN_EMAIL}`);
      console.log(`  Password: (use existing password)`);
      process.exit(0);
    }

    // Create admin via sign-up API (uses correct scrypt hashing)
    console.log("Creating admin via sign-up API...");
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
      // If user already exists, just update role
      if (error.code === "USER_ALREADY_EXISTS_USE_ANOTHER_EMAIL" ||
          error.message?.includes("already exists")) {
        console.log("User already exists, updating role...");
        await db
          .update(user)
          .set({ role: "admin", emailVerified: true })
          .where(eq(user.email, ADMIN_EMAIL));
        console.log("✅ Admin role updated!");
      } else {
        throw new Error(`Sign-up failed: ${JSON.stringify(error)}`);
      }
    } else {
      console.log("✅ Admin created via sign-up API");

      // Set admin role and verify email
      await db
        .update(user)
        .set({ role: "admin", emailVerified: true })
        .where(eq(user.email, ADMIN_EMAIL));
      console.log("✅ Admin role and email verification set");
    }

    console.log("\n========================================");
    console.log("Admin user ready!");
    console.log("========================================");
    console.log(`  Email: ${ADMIN_EMAIL}`);
    console.log(`  Password: ${ADMIN_PASSWORD}`);
    console.log(`  Role: admin`);
    console.log("========================================\n");
  } catch (error) {
    console.error("❌ Error creating admin user:", error);
    console.error("\n💡 Make sure the server is running: pnpm dev");
    process.exit(1);
  }

  process.exit(0);
}

createAdminUser();
