import { NextResponse } from "next/server";
import { auth } from "@/lib/auth";
import { headers } from "next/headers";
import { db } from "@/lib/db";
import { oauthApplication } from "@/lib/db/schema";
import { eq } from "drizzle-orm";

const PUBLIC_CLIENT = {
  id: "robolearn-public-client-id",
  clientId: "robolearn-public-client",
  clientSecret: "", // Empty string for public client (PKCE only)
  name: "RoboLearn Public Client",
  redirectUrls: JSON.stringify([
    "http://localhost:3000/auth/callback",
    // Production URLs will be added via env var
    ...(process.env.ROBOLEARN_INTERFACE_CALLBACK_URL
      ? [process.env.ROBOLEARN_INTERFACE_CALLBACK_URL]
      : []),
  ]),
  type: "public",
  disabled: false,
  metadata: JSON.stringify({
    token_endpoint_auth_method: "none",
    grant_types: ["authorization_code", "refresh_token"],
  }),
};

export async function POST() {
  try {
    // Check if user is authenticated and is admin
    const session = await auth.api.getSession({
      headers: await headers(),
    });

    if (!session?.user) {
      return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
    }

    if (session.user.role !== "admin") {
      return NextResponse.json({ error: "Forbidden - admin only" }, { status: 403 });
    }

    // Check if client already exists
    const existing = await db.select()
      .from(oauthApplication)
      .where(eq(oauthApplication.clientId, PUBLIC_CLIENT.clientId));

    if (existing.length > 0) {
      // Update existing
      await db.update(oauthApplication)
        .set({
          ...PUBLIC_CLIENT,
          updatedAt: new Date(),
        })
        .where(eq(oauthApplication.clientId, PUBLIC_CLIENT.clientId));

      return NextResponse.json({
        message: "Public client updated",
        clientId: PUBLIC_CLIENT.clientId,
      });
    }

    // Create new
    await db.insert(oauthApplication).values({
      ...PUBLIC_CLIENT,
      createdAt: new Date(),
      updatedAt: new Date(),
    });

    return NextResponse.json({
      message: "Public client created",
      clientId: PUBLIC_CLIENT.clientId,
    });

  } catch (error) {
    console.error("Failed to seed public client:", error);
    return NextResponse.json(
      { error: "Failed to seed public client", details: String(error) },
      { status: 500 }
    );
  }
}
