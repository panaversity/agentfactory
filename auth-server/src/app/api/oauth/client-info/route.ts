import { NextRequest, NextResponse } from "next/server";
import { db } from "@/lib/db";
import { oauthApplication } from "@/auth-schema";
import { eq } from "drizzle-orm";

// GET /api/oauth/client-info?client_id=xxx - Get public client info for consent screen
export async function GET(request: NextRequest) {
  try {
    const { searchParams } = new URL(request.url);
    const clientId = searchParams.get("client_id");

    if (!clientId) {
      return NextResponse.json(
        { error: "client_id is required" },
        { status: 400 }
      );
    }

    // Fetch client from database
    const [client] = await db
      .select({
        name: oauthApplication.name,
        clientId: oauthApplication.clientId,
      })
      .from(oauthApplication)
      .where(eq(oauthApplication.clientId, clientId))
      .limit(1);

    if (!client) {
      return NextResponse.json(
        { error: "Client not found" },
        { status: 404 }
      );
    }

    return NextResponse.json({
      name: client.name || "Unknown Application",
      clientId: client.clientId,
    });
  } catch (error) {
    console.error("Failed to fetch client info:", error);
    return NextResponse.json(
      { error: "Failed to fetch client info" },
      { status: 500 }
    );
  }
}
