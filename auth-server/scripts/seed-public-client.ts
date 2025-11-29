import { db } from "../src/lib/db";
import { oauthApplication } from "../src/lib/db/schema";
import { eq } from "drizzle-orm";

const PUBLIC_CLIENT = {
  id: "robolearn-public-client-id",
  clientId: "robolearn-public-client",
  clientSecret: null, // No secret for public client
  name: "RoboLearn Public Client",
  redirectUrls: JSON.stringify([
    "http://localhost:3000/auth/callback",
    // Add production URL here
  ]),
  type: "public",
  disabled: false,
  metadata: JSON.stringify({
    token_endpoint_auth_method: "none",
    grant_types: ["authorization_code", "refresh_token"],
  }),
};

async function seed() {
  console.log("Checking for existing public client...");
  
  const existing = await db.select()
    .from(oauthApplication)
    .where(eq(oauthApplication.clientId, PUBLIC_CLIENT.clientId));
  
  if (existing.length > 0) {
    console.log("Public client already exists, updating...");
    await db.update(oauthApplication)
      .set(PUBLIC_CLIENT)
      .where(eq(oauthApplication.clientId, PUBLIC_CLIENT.clientId));
    console.log("✅ Public client updated!");
  } else {
    console.log("Creating public client...");
    await db.insert(oauthApplication).values(PUBLIC_CLIENT);
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
