import { NextRequest, NextResponse } from "next/server";
import { auth } from "@/lib/auth";
import { db } from "@/lib/db";
import { user } from "@/auth-schema";
import { eq } from "drizzle-orm";

/**
 * GET /api/m2m/users/{username}
 *
 * M2M endpoint to fetch public user profile by username.
 * Used by client applications for public profile pages (/p/{username}).
 *
 * Authentication: API Key (x-api-key header)
 * Required scope: users:public:read OR users:read
 *
 * Response (success):
 * {
 *   "user": {
 *     "id": "...",
 *     "username": "johndoe",
 *     "displayUsername": "JohnDoe",
 *     "name": "John Doe",
 *     "image": "https://...",
 *     "createdAt": "2024-01-01T00:00:00.000Z"
 *   }
 * }
 *
 * Response (not found):
 * { "error": { "code": "USER_NOT_FOUND", "message": "..." } }
 *
 * Caching: Responses can be cached for 1-24 hours (public profile data is stable)
 */
export async function GET(
  request: NextRequest,
  { params }: { params: Promise<{ username: string }> }
) {
  try {
    // Get API key from header
    const apiKey = request.headers.get("x-api-key");

    if (!apiKey) {
      return NextResponse.json(
        {
          error: {
            code: "MISSING_API_KEY",
            message: "API key is required in x-api-key header",
          },
        },
        { status: 401 }
      );
    }

    // Verify API key
    const verifyResult = await auth.api.verifyApiKey({
      body: { key: apiKey },
    });

    if (!verifyResult.valid) {
      return NextResponse.json(
        {
          error: verifyResult.error || {
            code: "INVALID_API_KEY",
            message: "API key is invalid or expired",
          },
        },
        { status: 401 }
      );
    }

    // Check permissions - accept users:public:read, users:read, or admin:*
    const keyPermissions = verifyResult.key?.permissions as Record<string, string[]> | null;
    const hasPublicReadScope = keyPermissions?.["users-public"]?.includes("read");
    const hasFullReadScope = keyPermissions?.["users"]?.includes("read");
    const hasAdminScope = keyPermissions?.["admin"]?.includes("*");

    if (!hasPublicReadScope && !hasFullReadScope && !hasAdminScope) {
      return NextResponse.json(
        {
          error: {
            code: "INSUFFICIENT_PERMISSIONS",
            message: "API key requires users:public:read or users:read scope",
          },
        },
        { status: 403 }
      );
    }

    // Get username from path params
    const { username } = await params;

    if (!username || username.trim().length === 0) {
      return NextResponse.json(
        {
          error: {
            code: "MISSING_USERNAME",
            message: "Username is required",
          },
        },
        { status: 400 }
      );
    }

    // Normalize username (lowercase for lookup)
    const normalizedUsername = username.toLowerCase().trim();

    // Look up user by username - PUBLIC FIELDS ONLY
    const [foundUser] = await db
      .select({
        id: user.id,
        username: user.username,
        displayUsername: user.displayUsername,
        name: user.name,
        image: user.image,
        picture: user.picture,
        createdAt: user.createdAt,
      })
      .from(user)
      .where(eq(user.username, normalizedUsername))
      .limit(1);

    if (!foundUser) {
      return NextResponse.json(
        {
          error: {
            code: "USER_NOT_FOUND",
            message: `User with username "${username}" not found`,
          },
        },
        { status: 404 }
      );
    }

    // Return public profile data with cache headers
    const response = NextResponse.json({
      user: {
        id: foundUser.id,
        username: foundUser.username,
        displayUsername: foundUser.displayUsername || foundUser.username,
        name: foundUser.name,
        image: foundUser.image || foundUser.picture || null,
        createdAt: foundUser.createdAt,
      },
    });

    // Cache for 1 hour, serve stale for up to 24 hours
    response.headers.set("Cache-Control", "public, max-age=3600, stale-while-revalidate=86400");

    return response;
  } catch (error: unknown) {
    console.error("[M2M] Error fetching user by username:", error);

    const message = error instanceof Error ? error.message : "Internal server error";

    return NextResponse.json(
      {
        error: {
          code: "INTERNAL_ERROR",
          message,
        },
      },
      { status: 500 }
    );
  }
}

// CORS preflight
export async function OPTIONS() {
  return new NextResponse(null, {
    status: 204,
    headers: {
      "Access-Control-Allow-Origin": "*",
      "Access-Control-Allow-Methods": "GET, OPTIONS",
      "Access-Control-Allow-Headers": "Content-Type, x-api-key",
    },
  });
}
