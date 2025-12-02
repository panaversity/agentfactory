import { db } from "../src/lib/db";
import { user } from "../auth-schema";
import { eq } from "drizzle-orm";

async function verifyTestUser() {
  try {
    const result = await db
      .update(user)
      .set({ emailVerified: true })
      .where(eq(user.email, "testuser@example.com"))
      .returning();

    if (result.length > 0) {
      console.log("✅ Email verified for testuser@example.com");
      console.log("User:", result[0]);
    } else {
      console.log("❌ User not found");
    }
  } catch (error) {
    console.error("Error:", error);
  }
  process.exit(0);
}

verifyTestUser();
