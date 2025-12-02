import { NextRequest, NextResponse } from "next/server";
import { auth } from "@/lib/auth";
import { headers } from "next/headers";
import { db } from "@/lib/db";
import { oauthApplication } from "@/auth-schema";
import { eq } from "drizzle-orm";

// PATCH /api/admin/clients/[clientId] - Update client redirect URIs
export async function PATCH(
  request: NextRequest,
  { params }: { params: Promise<{ clientId: string }> }
) {
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

    const { clientId } = await params;
    const body = await request.json();
    const { redirectUrls, name } = body;

    // Validate redirect URLs if provided
    if (redirectUrls !== undefined) {
      if (!Array.isArray(redirectUrls) || redirectUrls.length === 0) {
        return NextResponse.json(
          { error: "redirectUrls must be a non-empty array" },
          { status: 400 }
        );
      }
    }

    // Build update object
    const updateData: {
      redirectUrls?: string;
      name?: string;
      updatedAt: Date;
    } = {
      updatedAt: new Date(),
    };

    if (redirectUrls) {
      updateData.redirectUrls = redirectUrls.join(",");
    }

    if (name) {
      updateData.name = name;
    }

    // Update the client
    const updated = await db
      .update(oauthApplication)
      .set(updateData)
      .where(eq(oauthApplication.clientId, clientId))
      .returning();

    if (updated.length === 0) {
      return NextResponse.json(
        { error: "Client not found" },
        { status: 404 }
      );
    }

    return NextResponse.json({
      success: true,
      clientId,
      name: updated[0].name,
      redirectUrls: redirectUrls || updated[0].redirectUrls?.split(","),
    });

  } catch (error) {
    console.error("Failed to update OAuth client:", error);
    return NextResponse.json(
      { error: "Failed to update client", details: String(error) },
      { status: 500 }
    );
  }
}
