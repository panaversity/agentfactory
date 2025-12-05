import { NextRequest, NextResponse } from "next/server";
import { auth } from "@/lib/auth";

/**
 * POST /api/api-key/verify
 *
 * Verify an API key for M2M authentication.
 * This endpoint is used by external services (FastAPI, NestJS, etc.)
 * to validate API keys before processing requests.
 *
 * Request body:
 * {
 *   "key": "pana_xxx...", // The full API key to verify
 *   "permissions": { "users": ["read"] } // Optional: required permissions to check
 * }
 *
 * Response (success):
 * {
 *   "valid": true,
 *   "key": {
 *     "id": "...",
 *     "name": "...",
 *     "userId": "...",
 *     "enabled": true,
 *     "expiresAt": "..." | null,
 *     "metadata": {...} | null,
 *     "permissions": {...} | null
 *   }
 * }
 *
 * Response (failure):
 * {
 *   "valid": false,
 *   "error": {
 *     "code": "INVALID_API_KEY" | "EXPIRED_API_KEY" | "DISABLED_API_KEY" | "INSUFFICIENT_PERMISSIONS",
 *     "message": "..."
 *   }
 * }
 */
export async function POST(request: NextRequest) {
  try {
    const body = await request.json();
    const { key, permissions } = body;

    if (!key || typeof key !== "string") {
      return NextResponse.json(
        {
          valid: false,
          error: {
            code: "MISSING_KEY",
            message: "API key is required in request body",
          },
        },
        { status: 400 }
      );
    }

    // Validate permissions format if provided
    if (permissions !== undefined) {
      if (typeof permissions !== "object" || permissions === null || Array.isArray(permissions)) {
        return NextResponse.json(
          {
            valid: false,
            error: {
              code: "INVALID_PERMISSIONS_FORMAT",
              message: "Permissions must be an object with string keys and string[] values",
            },
          },
          { status: 400 }
        );
      }
    }

    // Use Better Auth's server-side verification with optional permissions check
    const result = await auth.api.verifyApiKey({
      body: {
        key,
        ...(permissions && { permissions }),
      },
    });

    if (result.valid) {
      return NextResponse.json({
        valid: true,
        key: {
          id: result.key?.id,
          name: result.key?.name,
          userId: result.key?.userId,
          enabled: result.key?.enabled,
          expiresAt: result.key?.expiresAt,
          metadata: result.key?.metadata,
          permissions: result.key?.permissions,
        },
      });
    } else {
      return NextResponse.json(
        {
          valid: false,
          error: result.error || {
            code: "INVALID_API_KEY",
            message: "API key is invalid",
          },
        },
        { status: 401 }
      );
    }
  } catch (error: unknown) {
    console.error("[API Key Verify] Error:", error);

    const message = error instanceof Error ? error.message : "Internal server error";

    return NextResponse.json(
      {
        valid: false,
        error: {
          code: "INTERNAL_ERROR",
          message,
        },
      },
      { status: 500 }
    );
  }
}

// Handle CORS preflight
export async function OPTIONS() {
  return new NextResponse(null, {
    status: 204,
    headers: {
      "Access-Control-Allow-Origin": "*",
      "Access-Control-Allow-Methods": "POST, OPTIONS",
      "Access-Control-Allow-Headers": "Content-Type",
    },
  });
}
