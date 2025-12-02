import { db } from "../src/lib/db";
import { user, account } from "@/auth-schema";
import { eq } from "drizzle-orm";
import { randomUUID } from "crypto";

// Admin credentials
const ADMIN_EMAIL = "admin@robolearn.io";
const ADMIN_PASSWORD = "Admin123!"; // Will be hashed by Better Auth
const ADMIN_NAME = "Admin User";

async function createAdminUser() {
  console.log("Creating admin user...");

  try {
    // Check if admin already exists
    const existingUser = await db.query.user.findFirst({
      where: eq(user.email, ADMIN_EMAIL),
    });

    if (existingUser) {
      console.log("Admin user already exists, updating role to admin...");
      await db
        .update(user)
        .set({ role: "admin" })
        .where(eq(user.email, ADMIN_EMAIL));
      console.log("✅ Admin role updated!");
      console.log(`\nAdmin Credentials:`);
      console.log(`  Email: ${ADMIN_EMAIL}`);
      console.log(`  Password: (use existing password or reset)`);
      process.exit(0);
    }

    // Create new admin user
    const userId = randomUUID();
    const now = new Date();

    await db.insert(user).values({
      id: userId,
      email: ADMIN_EMAIL,
      name: ADMIN_NAME,
      emailVerified: true,
      role: "admin",
      createdAt: now,
      updatedAt: now,
    });

    // Create credential account with hashed password
    // Note: Better Auth uses bcrypt for password hashing
    const bcrypt = await import("bcryptjs");
    const hashedPassword = await bcrypt.hash(ADMIN_PASSWORD, 10);

    await db.insert(account).values({
      id: randomUUID(),
      userId: userId,
      accountId: userId,
      providerId: "credential",
      password: hashedPassword,
      createdAt: now,
      updatedAt: now,
    });

    console.log("✅ Admin user created successfully!");
    console.log(`\nAdmin Credentials:`);
    console.log(`  Email: ${ADMIN_EMAIL}`);
    console.log(`  Password: ${ADMIN_PASSWORD}`);
    console.log(`\n⚠️  Please change the password after first login!`);
  } catch (error) {
    console.error("❌ Error creating admin user:", error);
    process.exit(1);
  }

  process.exit(0);
}

createAdminUser();
