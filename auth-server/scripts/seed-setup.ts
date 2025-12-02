#!/usr/bin/env node
import * as dotenv from "dotenv";
dotenv.config({ path: ".env.local" });

import { neon } from "@neondatabase/serverless";
import { drizzle } from "drizzle-orm/neon-http";
import { pgTable, text, boolean, timestamp } from "drizzle-orm/pg-core";
import { eq, and } from "drizzle-orm";
import {
  TRUSTED_CLIENTS,
  DEFAULT_ORG_ID,
  DEFAULT_ORG_NAME,
  DEFAULT_ORG_SLUG,
} from "../src/lib/trusted-clients";
import bcrypt from "bcryptjs";

const TEST_ADMIN_EMAIL = "admin@robolearn.io";
const TEST_ADMIN_PASSWORD = "Admin123!@#"; // For local dev only
const TEST_ADMIN_NAME = "Admin User";

// Schema definitions
const user = pgTable("user", {
  id: text("id").primaryKey(),
  email: text("email").notNull().unique(),
  emailVerified: boolean("email_verified").default(false),
  name: text("name"),
  createdAt: timestamp("created_at"),
  updatedAt: timestamp("updated_at"),
});

const account = pgTable("account", {
  id: text("id").primaryKey(),
  userId: text("user_id").notNull(),
  accountId: text("account_id").notNull(),
  providerId: text("provider_id").notNull(),
  accessToken: text("access_token"),
  refreshToken: text("refresh_token"),
  idToken: text("id_token"),
  accessTokenExpiresAt: timestamp("access_token_expires_at"),
  refreshTokenExpiresAt: timestamp("refresh_token_expires_at"),
  scope: text("scope"),
  password: text("password"),
  createdAt: timestamp("created_at"),
  updatedAt: timestamp("updated_at"),
});

const oauthApplication = pgTable("oauth_application", {
  id: text("id").primaryKey(),
  name: text("name"),
  icon: text("icon"),
  metadata: text("metadata"),
  clientId: text("client_id").unique(),
  clientSecret: text("client_secret"),
  redirectUrls: text("redirect_urls"),
  type: text("type"),
  disabled: boolean("disabled").default(false),
  userId: text("user_id"),
  createdAt: timestamp("created_at"),
  updatedAt: timestamp("updated_at"),
});

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


const sql = neon(process.env.DATABASE_URL!);
const db = drizzle(sql);

// Default organization configuration (production-ready)
const DEFAULT_ORG = {
  id: DEFAULT_ORG_ID,
  name: DEFAULT_ORG_NAME,
  slug: DEFAULT_ORG_SLUG,
  logo: null,
  metadata: JSON.stringify({
    type: "default",
    description: "Default organization for all Panaversity users",
    plan: "platform",
    features: ["learning", "projects", "multi-tenant"]
  }),
};

// Test organization configuration (dev only)
const TEST_ORG = {
  id: "test-organization-id",
  name: "RoboLearn Test Organization",
  slug: "robolearn-test",
  logo: null,
  metadata: JSON.stringify({
    type: "test",
    plan: "pro",
    features: ["multi-tenant"]
  }),
};

/**
 * Upsert OAuth client from trusted-clients.ts configuration
 */
async function upsertClient(client: typeof TRUSTED_CLIENTS[0]) {
  const dbClient = {
    id: `${client.clientId}-id`,
    clientId: client.clientId,
    clientSecret: null, // Public clients have no secret
    name: client.name,
    redirectUrls: client.redirectUrls.join(","),
    type: client.type,
    disabled: client.disabled,
    metadata: JSON.stringify({
      token_endpoint_auth_method: "none",
      grant_types: ["authorization_code", "refresh_token"],
      skip_consent: client.skipConsent,
      ...client.metadata,
    }),
  };

  const existing = await db
    .select()
    .from(oauthApplication)
    .where(eq(oauthApplication.clientId, client.clientId));

  if (existing.length > 0) {
    console.log(`  ✅ ${client.name} (updating...)`);
    await db
      .update(oauthApplication)
      .set({
        name: dbClient.name,
        redirectUrls: dbClient.redirectUrls,
        type: dbClient.type,
        disabled: dbClient.disabled,
        metadata: dbClient.metadata,
        updatedAt: new Date(),
      })
      .where(eq(oauthApplication.clientId, client.clientId));
  } else {
    console.log(`  ✅ ${client.name} (creating...)`);
    await db.insert(oauthApplication).values({
      ...dbClient,
      createdAt: new Date(),
      updatedAt: new Date(),
    });
  }
}

/**
 * Create or get admin user
 */
async function createAdminUser() {
  // Check if admin user exists
  const existingUser = await db
    .select()
    .from(user)
    .where(eq(user.email, TEST_ADMIN_EMAIL));

  if (existingUser.length > 0) {
    console.log(`  ✅ Admin user exists: ${TEST_ADMIN_EMAIL}`);
    return existingUser[0].id;
  }

  // Create admin user
  console.log(`  ✅ Creating admin user: ${TEST_ADMIN_EMAIL}`);
  const userId = crypto.randomUUID();
  const hashedPassword = await bcrypt.hash(TEST_ADMIN_PASSWORD, 10);

  await db.insert(user).values({
    id: userId,
    email: TEST_ADMIN_EMAIL,
    emailVerified: true, // Skip email verification for local dev
    name: TEST_ADMIN_NAME,
    createdAt: new Date(),
    updatedAt: new Date(),
  });

  // Create account with password
  await db.insert(account).values({
    id: crypto.randomUUID(),
    userId: userId,
    accountId: userId,
    providerId: "credential",
    password: hashedPassword,
    createdAt: new Date(),
    updatedAt: new Date(),
  });

  console.log(`  📧 Email: ${TEST_ADMIN_EMAIL}`);
  console.log(`  🔑 Password: ${TEST_ADMIN_PASSWORD}`);

  return userId;
}

/**
 * Seed default organization (production + dev)
 */
async function seedDefaultOrganization(adminUserId: string) {
  console.log("\n📊 Seeding default organization...\n");

  // Create or update default organization
  const existingOrg = await db
    .select()
    .from(organization)
    .where(eq(organization.id, DEFAULT_ORG_ID));

  if (existingOrg.length > 0) {
    console.log(`  ✅ ${DEFAULT_ORG_NAME} (updating...)`);
    await db
      .update(organization)
      .set({
        name: DEFAULT_ORG.name,
        slug: DEFAULT_ORG.slug,
        metadata: DEFAULT_ORG.metadata,
      })
      .where(eq(organization.id, DEFAULT_ORG_ID));
  } else {
    console.log(`  ✅ ${DEFAULT_ORG_NAME} (creating...)`);
    await db.insert(organization).values({
      ...DEFAULT_ORG,
      createdAt: new Date(),
    });
  }

  // Add admin user as owner
  const existingMember = await db
    .select()
    .from(member)
    .where(
      and(
        eq(member.userId, adminUserId),
        eq(member.organizationId, DEFAULT_ORG_ID)
      )
    );

  if (existingMember.length > 0) {
    console.log(`  ✅ Admin already owner of ${DEFAULT_ORG_NAME}`);
  } else {
    console.log(`  ✅ Adding admin as owner`);
    await db.insert(member).values({
      id: `member-${adminUserId}-${DEFAULT_ORG_ID}`,
      userId: adminUserId,
      organizationId: DEFAULT_ORG_ID,
      role: "owner",
      createdAt: new Date(),
    });
  }
}

/**
 * Seed test organization (dev only)
 */
async function seedTestOrganization(adminUserId: string) {
  console.log("\n📊 Seeding test organization...\n");

  // Create or update test organization
  const existingOrg = await db
    .select()
    .from(organization)
    .where(eq(organization.id, TEST_ORG.id));

  if (existingOrg.length > 0) {
    console.log(`  ✅ ${TEST_ORG.name} (updating...)`);
    await db
      .update(organization)
      .set({
        name: TEST_ORG.name,
        slug: TEST_ORG.slug,
        metadata: TEST_ORG.metadata,
      })
      .where(eq(organization.id, TEST_ORG.id));
  } else {
    console.log(`  ✅ ${TEST_ORG.name} (creating...)`);
    await db.insert(organization).values({
      ...TEST_ORG,
      createdAt: new Date(),
    });
  }

  // Add admin user as owner
  const existingMember = await db
    .select()
    .from(member)
    .where(
      and(
        eq(member.userId, adminUserId),
        eq(member.organizationId, TEST_ORG.id)
      )
    );

  if (existingMember.length > 0) {
    console.log(`  ✅ Admin already member of ${TEST_ORG.name}`);
  } else {
    console.log(`  ✅ Adding admin as owner`);
    await db.insert(member).values({
      id: `member-${adminUserId}-${TEST_ORG.id}`,
      userId: adminUserId,
      organizationId: TEST_ORG.id,
      role: "owner",
      createdAt: new Date(),
    });
  }
}

/**
 * Main seed function
 */
async function seed() {
  const args = process.argv.slice(2);
  const isProd = args.includes("--prod");

  console.log("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  console.log(`  🔐 OAuth Setup ${isProd ? "(Production Mode)" : "(Development Mode)"}`);
  console.log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

  // Verify database connection
  if (!process.env.DATABASE_URL) {
    console.error("❌ ERROR: DATABASE_URL not set!");
    console.error("   Please set DATABASE_URL in .env.local");
    process.exit(1);
  }

  const dbHost = process.env.DATABASE_URL.split("@")[1]?.split("/")[0] || "Connected";
  console.log(`📊 Database: ${dbHost}`);
  console.log(`📝 Source: src/lib/trusted-clients.ts`);
  console.log(`\n🔐 Seeding OAuth clients...\n`);

  // Filter clients based on mode
  const clientsToSeed = isProd
    ? TRUSTED_CLIENTS.filter((c) => c.clientId !== "robolearn-public-client")
    : TRUSTED_CLIENTS;

  // Seed OAuth clients
  for (const client of clientsToSeed) {
    await upsertClient(client);
  }

  // Create admin user (local dev only)
  let adminUserId: string;
  if (!isProd) {
    console.log("\n👤 Setting up admin user...\n");
    adminUserId = await createAdminUser();
  } else {
    // In production, admin must be created manually
    const existingAdmin = await db
      .select()
      .from(user)
      .where(eq(user.email, TEST_ADMIN_EMAIL));

    if (existingAdmin.length === 0) {
      console.log("\n⚠️  WARNING: No admin user found!");
      console.log("   Create admin user manually first, then run this script again.");
      process.exit(1);
    }
    adminUserId = existingAdmin[0].id;
    console.log(`\n✅ Found admin user: ${TEST_ADMIN_EMAIL}`);
  }

  // Seed default organization (ALWAYS - both dev and prod)
  await seedDefaultOrganization(adminUserId);

  // Seed test organization (only in dev mode)
  if (!isProd) {
    await seedTestOrganization(adminUserId);
  }

  // Display results
  const allClients = await db.select().from(oauthApplication);
  const seededClientIds = clientsToSeed.map((c) => c.clientId);
  const seededClients = allClients.filter((c) =>
    seededClientIds.includes(c.clientId!)
  );

  console.log("\n✅ Successfully configured!\n");
  console.log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  console.log(`  ${isProd ? "Production" : "Development"} Clients`);
  console.log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

  seededClients.forEach((client, index) => {
    if (index > 0) console.log("\n" + "─".repeat(60) + "\n");
    console.log(`Client ID:       ${client.clientId}`);
    console.log(`Client Name:     ${client.name}`);
    console.log(`Client Type:     ${client.type} (PKCE flow)`);
    console.log(`Status:          ${client.disabled ? "DISABLED" : "ENABLED"}`);
    console.log(`\nRedirect URLs:`);
    client.redirectUrls?.split(",").forEach((url) => {
      console.log(`  - ${url}`);
    });
  });

  console.log("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
  console.log("📝 Next Steps:\n");
  console.log("1. Use these client IDs in your frontend applications");
  console.log("2. To manage redirect URLs, visit:");
  console.log(`   ${process.env.BETTER_AUTH_URL || "http://localhost:3001"}/admin/clients\n`);

  console.log("🏢 Organizations:\n");
  console.log(`   - Default Organization: ${DEFAULT_ORG_NAME}`);
  console.log(`   - ID: ${DEFAULT_ORG_ID} (hardcoded in auth.ts)`);
  console.log(`   - All new users auto-join this organization\n`);

  if (!isProd) {
    console.log("👤 Admin Credentials (Local Dev):\n");
    console.log(`   - Email: ${TEST_ADMIN_EMAIL}`);
    console.log(`   - Password: ${TEST_ADMIN_PASSWORD}\n`);

    console.log("💡 Test Organization:\n");
    console.log("   - Additional test org for multi-tenant testing");
    console.log(`   - Name: ${TEST_ORG.name}\n`);
  }

  console.log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

  process.exit(0);
}

seed().catch((err) => {
  console.error("\n❌ Failed to seed:");
  console.error(err);
  process.exit(1);
});
