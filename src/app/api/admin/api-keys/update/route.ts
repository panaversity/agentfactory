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
 * - permissions: Record<string, string[]> (optional) - Update scopes/permissions
 *   Example: { "users": ["read"], "projects": ["read", "write"] }
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
    const { keyId, enabled, name, metadata, permissions } = body;

    if (!keyId || typeof keyId !== "string") {
      return NextResponse.json(
        { error: "Key ID is required" },
        { status: 400 }
      );
    }

    // Validate permissions format if provided
    if (permissions !== undefined) {
      if (typeof permissions !== "object" || permissions === null || Array.isArray(permissions)) {
        return NextResponse.json(
          { error: "Permissions must be an object with string keys and string[] values" },
          { status: 400 }
        );
      }
      // Validate each permission entry
      for (const [resource, actions] of Object.entries(permissions)) {
        if (!Array.isArray(actions) || !actions.every((a) => typeof a === "string")) {
          return NextResponse.json(
            { error: `Invalid permissions for resource "${resource}": must be an array of strings` },
            { status: 400 }
          );
        }
      }
    }

    // Use Better Auth's internal API to update the key
    const response = await auth.api.updateApiKey({
      headers: await headers(),
      body: {
        keyId,
        ...(typeof enabled === "boolean" && { enabled }),
        ...(name && { name: name.trim() }),
        ...(metadata && { metadata }),
        ...(permissions && { permissions }),
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
