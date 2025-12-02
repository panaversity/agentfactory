import * as dotenv from "dotenv";
// Load environment variables FIRST before any imports that use them
dotenv.config({ path: ".env.local" });

import { neon } from "@neondatabase/serverless";
import { drizzle } from "drizzle-orm/neon-http";
import { pgTable, text, boolean, timestamp, index } from "drizzle-orm/pg-core";
import { eq } from "drizzle-orm";

const oauthApplication = pgTable(
  "oauth_application",
  {
    id: text("id").primaryKey(),
    name: text("name"),
    icon: text("icon"),
    metadata: text("metadata"),
    clientId: text("client_id").unique(),
    clientSecret: text("client_secret"),
    redirectUrls: text("redirect_urls"), // Better Auth 1.4.x expects redirectUrls (camelCase)
    type: text("type"),
    disabled: boolean("disabled").default(false),
    userId: text("user_id"),
    createdAt: timestamp("created_at"),
    updatedAt: timestamp("updated_at"),
  }
);

const sql = neon(process.env.DATABASE_URL!);
const db = drizzle(sql);

const PUBLIC_CLIENT = {
  id: "robolearn-public-client-id",
  clientId: "robolearn-public-client",
  clientSecret: null, // No secret for public client
  name: "RoboLearn Public Client",
  redirectUrls: [ // Better Auth 1.4.x expects redirectUrls (camelCase)
    "http://localhost:3000/auth/callback",
    // Add production URL here
  ].join(","),
  type: "public",
  disabled: false,
  metadata: JSON.stringify({
    token_endpoint_auth_method: "none",
    grant_types: ["authorization_code", "refresh_token"],
  }),
};

async function seed() {
  console.log("Checking for existing public client...");
  console.log("DATABASE_URL:", process.env.DATABASE_URL ? "Set" : "NOT SET");

  const existing = await db.select()
    .from(oauthApplication)
    .where(eq(oauthApplication.clientId, PUBLIC_CLIENT.clientId));

  if (existing.length > 0) {
    console.log("Public client already exists, updating...");
    await db.update(oauthApplication)
      .set({
        name: PUBLIC_CLIENT.name,
        redirectUrls: PUBLIC_CLIENT.redirectUrls,
        type: PUBLIC_CLIENT.type,
        disabled: PUBLIC_CLIENT.disabled,
        metadata: PUBLIC_CLIENT.metadata,
        updatedAt: new Date(),
      })
      .where(eq(oauthApplication.clientId, PUBLIC_CLIENT.clientId));
    console.log("✅ Public client updated!");
  } else {
    console.log("Creating public client...");
    await db.insert(oauthApplication).values({
      ...PUBLIC_CLIENT,
      createdAt: new Date(),
      updatedAt: new Date(),
    });
    console.log("✅ Public client created!");
  }

  // Verify
  const result = await db.select()
    .from(oauthApplication)
    .where(eq(oauthApplication.clientId, PUBLIC_CLIENT.clientId));

  console.log("Client in DB:", result[0]);
  process.exit(0);
}

seed().catch((err) => {
  console.error("Failed to seed:", err);
  process.exit(1);
});
