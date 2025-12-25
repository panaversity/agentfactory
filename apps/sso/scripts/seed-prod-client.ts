import * as dotenv from "dotenv";
// Load environment variables FIRST before any imports that use them
dotenv.config({ path: ".env.local" });

import { neon } from "@neondatabase/serverless";
import { drizzle } from "drizzle-orm/neon-http";
import { pgTable, text, boolean, timestamp } from "drizzle-orm/pg-core";
import { eq } from "drizzle-orm";

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

const sql = neon(process.env.DATABASE_URL!);
const db = drizzle(sql);

/**
 * Production-ready OAuth Clients
 *
 * These are PUBLIC clients (PKCE flow - no client secret required)
 * Safe for frontend applications (browser, mobile apps)
 */

const PANAVERSITY_SSO_CLIENT = {
  id: "panaversity-sso-public-client-id",
  clientId: "panaversity-sso-public-client",
  clientSecret: null,
  name: "Panaversity SSO",
  redirectUrls: [
    "http://localhost:3000/auth/callback",
    "https://panaversity.org/auth/callback",
  ].join(","),
  type: "public",
  disabled: false,
  metadata: JSON.stringify({
    token_endpoint_auth_method: "none",
    grant_types: ["authorization_code", "refresh_token"],
    description: "Panaversity SSO public client",
  }),
};

const AI_NATIVE_CLIENT = {
  id: "ai-native-public-client-id",
  clientId: "ai-native-public-client",
  clientSecret: null,
  name: "AI Native Platform (Legacy)",
  redirectUrls: [
    "http://localhost:3000/auth/callback",
    "https://ai-native.panaversity.org/auth/callback",
  ].join(","),
  type: "public",
  disabled: false,
  metadata: JSON.stringify({
    token_endpoint_auth_method: "none",
    grant_types: ["authorization_code", "refresh_token"],
    description: "AI Native platform public client (Legacy - use agent-factory-public-client)",
  }),
};

const AGENT_FACTORY_CLIENT = {
  id: "agent-factory-public-client-id",
  clientId: "agent-factory-public-client",
  clientSecret: null,
  name: "The AI Agent Factory",
  redirectUrls: [
    "http://localhost:3000/auth/callback",
    "https://agent-factory-interface.vercel.app/auth/callback",
    "https://agentfactory.panaversity.org/auth/callback",
  ].join(","),
  type: "public",
  disabled: false,
  metadata: JSON.stringify({
    token_endpoint_auth_method: "none",
    grant_types: ["authorization_code", "refresh_token"],
    description: "The AI Agent Factory - Spec-Driven Blueprint for Digital FTEs",
  }),
};

async function upsertClient(client: typeof PANAVERSITY_SSO_CLIENT) {
  const existing = await db
    .select()
    .from(oauthApplication)
    .where(eq(oauthApplication.clientId, client.clientId));

  if (existing.length > 0) {
    console.log(`✅ Client already exists: ${client.name}`);
    console.log(`   Updating configuration...`);
    await db
      .update(oauthApplication)
      .set({
        name: client.name,
        clientSecret: client.clientSecret,
        redirectUrls: client.redirectUrls,
        type: client.type,
        disabled: client.disabled,
        metadata: client.metadata,
        updatedAt: new Date(),
      })
      .where(eq(oauthApplication.clientId, client.clientId));
  } else {
    console.log(`✅ Creating new client: ${client.name}`);
    await db.insert(oauthApplication).values({
      ...client,
      createdAt: new Date(),
      updatedAt: new Date(),
    });
  }
}

async function seed() {
  console.log("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  console.log("  Panaversity - Production Clients Setup");
  console.log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

  // Verify database connection
  if (!process.env.DATABASE_URL) {
    console.error("❌ ERROR: DATABASE_URL not set!");
    console.error("   Please set DATABASE_URL in .env.local");
    process.exit(1);
  }

  console.log("📊 Database:", process.env.DATABASE_URL.split("@")[1]?.split("/")[0] || "Connected");
  console.log("\n🔐 Seeding OAuth clients...\n");

  await upsertClient(PANAVERSITY_SSO_CLIENT);
  await upsertClient(AI_NATIVE_CLIENT);
  await upsertClient(AGENT_FACTORY_CLIENT);

  // Verify and display all production clients
  const allClients = await db.select().from(oauthApplication);
  const prodClients = allClients.filter(c =>
    c.clientId === PANAVERSITY_SSO_CLIENT.clientId ||
    c.clientId === AI_NATIVE_CLIENT.clientId ||
    c.clientId === AGENT_FACTORY_CLIENT.clientId
  );

  console.log("\n✅ Successfully configured!\n");
  console.log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  console.log("  Production Clients");
  console.log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

  prodClients.forEach((client, index) => {
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
  console.log("2. To manage redirect URLs, visit the admin panel:");
  console.log(`   ${process.env.BETTER_AUTH_URL || "http://localhost:3001"}/admin/clients\n`);
  console.log("3. Example authorization URL:");
  console.log(`   ${process.env.BETTER_AUTH_URL || "http://localhost:3001"}/api/auth/oauth2/authorize?`);
  console.log(`     client_id=panaversity-sso-public-client`);
  console.log(`     &redirect_uri=http://localhost:3000/auth/callback`);
  console.log(`     &response_type=code`);
  console.log(`     &scope=openid+profile+email`);
  console.log(`     &code_challenge=YOUR_PKCE_CHALLENGE`);
  console.log(`     &code_challenge_method=S256\n`);
  console.log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

  process.exit(0);
}

seed().catch((err) => {
  console.error("\n❌ Failed to seed client:");
  console.error(err);
  process.exit(1);
});
