import * as dotenv from "dotenv";
dotenv.config({ path: ".env.local" });

import { neon } from "@neondatabase/serverless";
import { drizzle } from "drizzle-orm/neon-http";
import { pgTable, text, timestamp } from "drizzle-orm/pg-core";
import { eq } from "drizzle-orm";

// Schema definitions
const organization = pgTable("organization", {
  id: text("id").primaryKey(),
  name: text("name").notNull(),
  slug: text("slug").unique(),
  logo: text("logo"),
  metadata: text("metadata"),
  createdAt: timestamp("created_at").defaultNow().notNull(),
});

const member = pgTable("member", {
  id: text("id").primaryKey(),
  userId: text("user_id").notNull(),
  organizationId: text("organization_id").notNull(),
  role: text("role").notNull(),
  createdAt: timestamp("created_at").defaultNow().notNull(),
});

const user = pgTable("user", {
  id: text("id").primaryKey(),
  email: text("email").notNull().unique(),
});

const sql = neon(process.env.DATABASE_URL!);
const db = drizzle(sql);

const TEST_ORG = {
  id: "test-organization-id",
  name: "RoboLearn Test Organization",
  slug: "robolearn-test",
  logo: null,
  metadata: JSON.stringify({ plan: "pro", features: ["multi-tenant"] }),
};

async function seed() {
  console.log("Seeding test organization...");
  console.log("DATABASE_URL:", process.env.DATABASE_URL ? "Set" : "NOT SET");

  // Create or update organization
  const existingOrg = await db
    .select()
    .from(organization)
    .where(eq(organization.id, TEST_ORG.id));

  if (existingOrg.length > 0) {
    console.log("  Organization already exists, updating...");
    await db
      .update(organization)
      .set({
        name: TEST_ORG.name,
        slug: TEST_ORG.slug,
        metadata: TEST_ORG.metadata,
      })
      .where(eq(organization.id, TEST_ORG.id));
  } else {
    console.log("  Creating organization...");
    await db.insert(organization).values({
      ...TEST_ORG,
      createdAt: new Date(),
    });
  }

  // Find test user
  const testUser = await db
    .select()
    .from(user)
    .where(eq(user.email, "admin@robolearn.io"));

  if (testUser.length === 0) {
    console.log("  Test user not found (admin@robolearn.io)");
    process.exit(1);
  }

  const userId = testUser[0].id;
  console.log("  Found test user:", userId);

  // Add user to organization as owner
  const existingMember = await db
    .select()
    .from(member)
    .where(eq(member.userId, userId));

  if (existingMember.length > 0) {
    console.log("  User already a member, updating role...");
    await db
      .update(member)
      .set({ role: "owner" })
      .where(eq(member.userId, userId));
  } else {
    console.log("  Adding user as organization owner...");
    await db.insert(member).values({
      id: `member-${userId}-${TEST_ORG.id}`,
      userId,
      organizationId: TEST_ORG.id,
      role: "owner",
      createdAt: new Date(),
    });
  }

  // Verify
  const result = await db
    .select()
    .from(member)
    .where(eq(member.userId, userId));

  console.log("\n✅ Test organization seeded!");
  console.log("Organization:", TEST_ORG.name);
  console.log("User membership:", result[0]);
  process.exit(0);
}

seed().catch((err) => {
  console.error("Failed to seed:", err);
  process.exit(1);
});
