import { NextResponse } from "next/server";
import { auth } from "@/lib/auth";
import { headers } from "next/headers";

/**
 * GET /api/admin/api-keys/list
 * Admin-only endpoint to list API keys for the authenticated admin user
 */
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
      return NextResponse.json({ error: "Forbidden - Admin access required" }, { status: 403 });
    }

    // Use Better Auth's internal API to list keys
    // Pass the headers so Better Auth can identify the user
    const response = await auth.api.listApiKeys({
      headers: await headers(),
    });

    return NextResponse.json(response);
  } catch (error: any) {
    console.error("[Admin API Keys] Failed to list API keys:", error);
    return NextResponse.json(
      { error: error.message || "Failed to list API keys" },
      { status: 500 }
    );
  }
}
