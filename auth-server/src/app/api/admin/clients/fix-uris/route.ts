import { NextRequest, NextResponse } from "next/server";
import { auth } from "@/lib/auth";
import { headers } from "next/headers";
import { db } from "@/lib/db";
import { oauthApplication } from "@/auth-schema";
import { eq } from "drizzle-orm";

const CORRECT_REDIRECT_URIS = [
  "http://localhost:3000/auth/callback",
  "http://localhost:3000/robolearn/auth/callback",
];

// POST /api/admin/clients/fix-uris - Fix redirect URIs for a client
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
    const { clientId } = body;

    if (!clientId) {
      return NextResponse.json(
        { error: "clientId is required" },
        { status: 400 }
      );
    }

    // Check if client exists
    const [client] = await db
      .select()
      .from(oauthApplication)
      .where(eq(oauthApplication.clientId, clientId))
      .limit(1);

    if (!client) {
      return NextResponse.json(
        { error: "Client not found" },
        { status: 404 }
      );
    }

    // Update redirect URIs - comma-separated format
    const updated = await db
      .update(oauthApplication)
      .set({
        redirectUrls: CORRECT_REDIRECT_URIS.join(","),
        updatedAt: new Date(),
      })
      .where(eq(oauthApplication.clientId, clientId))
      .returning();

    if (updated.length === 0) {
      return NextResponse.json(
        { error: "Failed to update client" },
        { status: 500 }
      );
    }

    return NextResponse.json({
      success: true,
      clientId,
      name: updated[0].name,
      old_redirect_uris: client.redirectUrls?.split(",") || [],
      new_redirect_uris: CORRECT_REDIRECT_URIS,
    });

  } catch (error) {
    console.error("Failed to fix client redirect URIs:", error);
    return NextResponse.json(
      { error: "Failed to fix client", details: String(error) },
      { status: 500 }
    );
  }
}
