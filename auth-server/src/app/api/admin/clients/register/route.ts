import { NextRequest, NextResponse } from "next/server";
import { auth } from "@/lib/auth";
import { headers } from "next/headers";
import { db } from "@/lib/db";
import { oauthApplication } from "@/lib/db/schema";
import crypto from "crypto";

// Generate a random client ID
function generateClientId(): string {
  return crypto.randomBytes(24).toString("base64url");
}

// Generate a random client secret
function generateClientSecret(): string {
  return crypto.randomBytes(32).toString("base64url");
}

export async function POST(request: NextRequest) {
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

    const body = await request.json();
    const { name, redirectUrls, scope, clientType } = body;

    if (!name || !redirectUrls || !Array.isArray(redirectUrls) || redirectUrls.length === 0) {
      return NextResponse.json(
        { error: "name and redirectUrls are required" },
        { status: 400 }
      );
    }

    const isPublic = clientType === "public";
    const clientId = generateClientId();
    // For public clients, clientSecret should be null (not empty string)
    // Better Auth uses this to determine if PKCE is required
    const clientSecret = isPublic ? null : generateClientSecret();

    const newClient = {
      id: crypto.randomUUID(),
      clientId,
      clientSecret: clientSecret || null, // Ensure null instead of empty string
      name,
      redirectURLs: redirectUrls.join(","), // Database field is redirectURLs (capital URLs) - Better Auth documented field name
      type: isPublic ? "public" : "confidential",
      disabled: false,
      metadata: JSON.stringify({
        token_endpoint_auth_method: isPublic ? "none" : "client_secret_post",
        grant_types: ["authorization_code", "refresh_token"],
        scope: scope || "openid profile email",
      }),
      createdAt: new Date(),
      updatedAt: new Date(),
    };

    await db.insert(oauthApplication).values(newClient);

    return NextResponse.json({
      success: true,
      client_id: clientId,
      client_secret: isPublic ? null : clientSecret,
      client_type: isPublic ? "public" : "confidential",
      name,
      redirect_uris: redirectUrls,
    });

  } catch (error) {
    console.error("Failed to register client:", error);
    return NextResponse.json(
      { error: "Failed to register client", details: String(error) },
      { status: 500 }
    );
  }
}
