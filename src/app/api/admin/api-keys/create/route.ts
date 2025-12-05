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
 * - permissions: Record<string, string[]> (optional) - Scopes/permissions for the key
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
    const { name, expiresIn, metadata, permissions } = body;

    if (!name || typeof name !== "string" || name.trim().length === 0) {
      return NextResponse.json(
        { error: "Name is required" },
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

    // Use Better Auth's internal API to create the key
    // Note: permissions is server-only, so we must pass userId directly (not headers)
    const response = await auth.api.createApiKey({
      body: {
        userId: session.user.id,
        name: name.trim(),
        ...(expiresIn && { expiresIn }),
        ...(metadata && { metadata }),
        ...(permissions && { permissions }),
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
