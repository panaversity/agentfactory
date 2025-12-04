import { NextRequest, NextResponse } from "next/server";
import { auth } from "@/lib/auth";
import { headers } from "next/headers";

/**
 * POST /api/admin/api-keys/create
 * Admin-only endpoint to create a new API key
 *
 * Body:
 * - name: string (required) - Human-readable name for the key
 * - expiresIn: number (optional) - Expiration time in seconds
 * - metadata: object (optional) - Additional metadata
 */
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
      return NextResponse.json({ error: "Forbidden - Admin access required" }, { status: 403 });
    }

    const body = await request.json();
    const { name, expiresIn, metadata } = body;

    if (!name || typeof name !== "string" || name.trim().length === 0) {
      return NextResponse.json(
        { error: "Name is required" },
        { status: 400 }
      );
    }

    // Use Better Auth's internal API to create the key
    const response = await auth.api.createApiKey({
      headers: await headers(),
      body: {
        name: name.trim(),
        ...(expiresIn && { expiresIn }),
        ...(metadata && { metadata }),
      },
    });

    return NextResponse.json(response);
  } catch (error: any) {
    console.error("[Admin API Keys] Failed to create API key:", error);
    return NextResponse.json(
      { error: error.message || "Failed to create API key" },
      { status: 500 }
    );
  }
}
