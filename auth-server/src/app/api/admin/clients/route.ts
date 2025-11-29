import { NextRequest, NextResponse } from "next/server";
import { auth } from "@/lib/auth";
import { headers } from "next/headers";
import { db } from "@/lib/db";
import { oauthApplication } from "@/lib/db/schema";
import { eq } from "drizzle-orm";

// Pre-configured trusted client (from auth.ts)
const TRUSTED_CLIENT = {
  id: "trusted-robolearn-public",
  clientId: "robolearn-public-client",
  name: "RoboLearn Public Client",
  redirectUrls: process.env.ROBOLEARN_INTERFACE_CALLBACK_URL
    ? [process.env.ROBOLEARN_INTERFACE_CALLBACK_URL]
    : (process.env.NODE_ENV === "development"
        ? ["http://localhost:3000/auth/callback"]
        : []),
  type: "public",
  disabled: false,
  isTrusted: true, // Flag to indicate this is a pre-configured trusted client
  metadata: { token_endpoint_auth_method: "none" }, // Public client uses PKCE
  createdAt: null,
};

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
    const parsedClients = clients.map(client => {
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
            ? client.redirectUrls.split(',').map(u => u.trim())
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

    // Add trusted client at the beginning
    return NextResponse.json({ clients: [TRUSTED_CLIENT, ...parsedClients] });
  } catch (error) {
    console.error("Failed to fetch OAuth clients:", error);
    return NextResponse.json(
      { error: "Failed to fetch clients" },
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

    // Prevent deletion of trusted client
    if (clientId === "robolearn-public-client") {
      return NextResponse.json(
        { error: "Cannot delete pre-configured trusted client" },
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
