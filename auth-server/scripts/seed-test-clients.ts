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

// Public client (PKCE flow - no secret)
const PUBLIC_CLIENT = {
  id: "robolearn-public-client-id",
  clientId: "robolearn-public-client",
  clientSecret: null,
  name: "RoboLearn Public Client",
  redirectUrls: [
    "http://localhost:3000/auth/callback",
    "http://localhost:4000/callback",
  ].join(","),
  type: "public",
  disabled: false,
  metadata: JSON.stringify({
    token_endpoint_auth_method: "none",
    grant_types: ["authorization_code", "refresh_token"],
  }),
};

// Confidential client (client_secret_basic flow)
const CONFIDENTIAL_CLIENT = {
  id: "robolearn-confidential-client-id",
  clientId: "robolearn-confidential-client",
  clientSecret: "robolearn-confidential-secret-for-testing-only",
  name: "RoboLearn Confidential Client",
  redirectUrls: [
    "http://localhost:8000/auth/callback",
    "http://localhost:5000/callback",
  ].join(","),
  type: "confidential",
  disabled: false,
  metadata: JSON.stringify({
    token_endpoint_auth_method: "client_secret_basic",
    grant_types: ["authorization_code", "refresh_token"],
  }),
};

async function upsertClient(
  client: typeof PUBLIC_CLIENT | typeof CONFIDENTIAL_CLIENT
) {
  const existing = await db
    .select()
    .from(oauthApplication)
    .where(eq(oauthApplication.clientId, client.clientId));

  if (existing.length > 0) {
    console.log(`  Updating ${client.name}...`);
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
    console.log(`  Creating ${client.name}...`);
    await db.insert(oauthApplication).values({
      ...client,
      createdAt: new Date(),
      updatedAt: new Date(),
    });
  }
}

async function seed() {
  console.log("Seeding OAuth test clients...");
  console.log("DATABASE_URL:", process.env.DATABASE_URL ? "Set" : "NOT SET");

  await upsertClient(PUBLIC_CLIENT);
  await upsertClient(CONFIDENTIAL_CLIENT);

  // Verify
  const clients = await db.select().from(oauthApplication);
  console.log("\n✅ OAuth clients in DB:");
  clients.forEach((c) => {
    console.log(`  - ${c.clientId} (${c.type}) secret=${c.clientSecret ? "YES" : "NO"}`);
  });

  process.exit(0);
}

seed().catch((err) => {
  console.error("Failed to seed:", err);
  process.exit(1);
});
