import { NextRequest, NextResponse } from "next/server";
import { auth } from "@/lib/auth";
import { headers } from "next/headers";
import { db } from "@/lib/db";
import { oauthApplication } from "@/auth-schema";
import { eq } from "drizzle-orm";
import { TRUSTED_CLIENT_IDS } from "@/lib/trusted-clients";

// Trusted clients are defined in src/lib/trusted-clients.ts
// They are:
// 1. Configured in auth.ts for skipConsent behavior
// 2. Stored in database for token storage (FK requirement)
// 3. Protected from editing/deletion via admin UI (see TRUSTED_CLIENT_IDS)

export async function GET() {
  try {
    // Check if user is authenticated and is admin
    const session = await auth.api.getSession({
      headers: await headers(),
    });

    if (!session?.user) {
      return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
    }

    if (session.user.role !== "admin") {
      return NextResponse.json({ error: "Forbidden" }, { status: 403 });
    }

    // Fetch all OAuth clients from database
    const clients = await db.select({
      id: oauthApplication.id,
      clientId: oauthApplication.clientId,
      name: oauthApplication.name,
      redirectUrls: oauthApplication.redirectUrls,
      type: oauthApplication.type,
      disabled: oauthApplication.disabled,
      metadata: oauthApplication.metadata,
      createdAt: oauthApplication.createdAt,
    }).from(oauthApplication);

    // Parse redirectUrls and metadata - handle both JSON strings and plain strings
    const parsedClients = clients.map((client: typeof clients[number]) => {
      let redirectUrls: string[] = [];
      let metadata: Record<string, unknown> = {};

      // Handle redirectUrls - could be JSON array or plain string
      if (client.redirectUrls) {
        try {
          const parsed = JSON.parse(client.redirectUrls);
          redirectUrls = Array.isArray(parsed) ? parsed : [client.redirectUrls];
        } catch {
          // Not JSON, treat as single URL or comma-separated
          redirectUrls = client.redirectUrls.includes(',')
            ? client.redirectUrls.split(',').map((u: string) => u.trim())
            : [client.redirectUrls];
        }
      }

      // Handle metadata - should be JSON
      if (client.metadata) {
        try {
          metadata = JSON.parse(client.metadata);
        } catch {
          metadata = {};
        }
      }

      return {
        ...client,
        redirectUrls,
        metadata,
        isTrusted: false,
      };
    });

    // Return all clients from database
    // Trusted clients (defined in auth.ts) are also stored in DB for token storage
    return NextResponse.json({ clients: parsedClients });
  } catch (error) {
    console.error("Failed to fetch OAuth clients:", error);
    return NextResponse.json(
      { error: "Failed to fetch clients" },
      { status: 500 }
    );
  }
}

export async function PATCH(request: NextRequest) {
  try {
    // Check if user is authenticated and is admin
    const session = await auth.api.getSession({
      headers: await headers(),
    });

    if (!session?.user) {
      return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
    }

    if (session.user.role !== "admin") {
      return NextResponse.json({ error: "Forbidden" }, { status: 403 });
    }

    const body = await request.json();
    const { clientId, redirectUrls } = body;

    if (!clientId) {
      return NextResponse.json({ error: "Client ID required" }, { status: 400 });
    }

    if (!redirectUrls || !Array.isArray(redirectUrls) || redirectUrls.length === 0) {
      return NextResponse.json(
        { error: "At least one redirect URL is required" },
        { status: 400 }
      );
    }

    // Prevent editing of trusted clients (first-party apps defined in auth.ts)
    if (TRUSTED_CLIENT_IDS.includes(clientId)) {
      return NextResponse.json(
        { error: "Cannot edit trusted first-party client via UI. Edit auth.ts trustedClients[] and restart server." },
        { status: 400 }
      );
    }

    // Update the OAuth client's redirect URLs
    await db
      .update(oauthApplication)
      .set({
        redirectUrls: redirectUrls.join(","),
        updatedAt: new Date(),
      })
      .where(eq(oauthApplication.clientId, clientId));

    return NextResponse.json({ success: true });
  } catch (error) {
    console.error("Failed to update OAuth client:", error);
    return NextResponse.json(
      { error: "Failed to update client" },
      { status: 500 }
    );
  }
}

export async function DELETE(request: NextRequest) {
  try {
    // Check if user is authenticated and is admin
    const session = await auth.api.getSession({
      headers: await headers(),
    });

    if (!session?.user) {
      return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
    }

    if (session.user.role !== "admin") {
      return NextResponse.json({ error: "Forbidden" }, { status: 403 });
    }

    const { searchParams } = new URL(request.url);
    const clientId = searchParams.get("clientId");

    if (!clientId) {
      return NextResponse.json({ error: "Client ID required" }, { status: 400 });
    }

    // Prevent deletion of trusted clients (first-party apps defined in auth.ts)
    if (TRUSTED_CLIENT_IDS.includes(clientId)) {
      return NextResponse.json(
        { error: "Cannot delete trusted first-party client. Remove from auth.ts first." },
        { status: 400 }
      );
    }

    // Delete the OAuth client
    await db.delete(oauthApplication).where(eq(oauthApplication.clientId, clientId));

    return NextResponse.json({ success: true });
  } catch (error) {
    console.error("Failed to delete OAuth client:", error);
    return NextResponse.json(
      { error: "Failed to delete client" },
      { status: 500 }
    );
  }
}
