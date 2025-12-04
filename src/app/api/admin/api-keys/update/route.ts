import { NextRequest, NextResponse } from "next/server";
import { auth } from "@/lib/auth";
import { headers } from "next/headers";

/**
 * POST /api/admin/api-keys/update
 * Admin-only endpoint to update an API key (e.g., revoke/enable)
 *
 * Body:
 * - keyId: string (required) - ID of the key to update
 * - enabled: boolean (optional) - Enable/disable the key
 * - name: string (optional) - Update the key name
 * - metadata: object (optional) - Update metadata
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
    const { keyId, enabled, name, metadata } = body;

    if (!keyId || typeof keyId !== "string") {
      return NextResponse.json(
        { error: "Key ID is required" },
        { status: 400 }
      );
    }

    // Use Better Auth's internal API to update the key
    const response = await auth.api.updateApiKey({
      headers: await headers(),
      body: {
        keyId,
        ...(typeof enabled === "boolean" && { enabled }),
        ...(name && { name: name.trim() }),
        ...(metadata && { metadata }),
      },
    });

    return NextResponse.json(response);
  } catch (error: any) {
    console.error("[Admin API Keys] Failed to update API key:", error);
    return NextResponse.json(
      { error: error.message || "Failed to update API key" },
      { status: 500 }
    );
  }
}
