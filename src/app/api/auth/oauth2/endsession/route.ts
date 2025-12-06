import { auth, AUTH_COOKIE_PREFIX } from "@/lib/auth";
import { TRUSTED_CLIENTS } from "@/lib/trusted-clients";
import { cookies } from "next/headers";
import { NextRequest, NextResponse } from "next/server";

/**
 * Better Auth session cookie names
 * Derived from AUTH_COOKIE_PREFIX for consistency with auth config
 */
const SESSION_COOKIE_NAMES = [
  `${AUTH_COOKIE_PREFIX}.session_token`,
  `${AUTH_COOKIE_PREFIX}.session_data`,
];

/**
 * Allowed post-logout redirect origins
 * Derived from trusted clients + SSO's own origin
 * Security: Prevents open redirect attacks (CWE-601)
 */
function getAllowedPostLogoutOrigins(): string[] {
  const origins = new Set<string>();

  // Add SSO's own origin
  const ssoUrl = process.env.BETTER_AUTH_URL || "http://localhost:3001";
  try {
    origins.add(new URL(ssoUrl).origin);
  } catch {
    // Invalid URL, skip
  }

  // Add all trusted client redirect URL origins
  for (const client of TRUSTED_CLIENTS) {
    for (const redirectUrl of client.redirectUrls) {
      try {
        origins.add(new URL(redirectUrl).origin);
      } catch {
        // Invalid URL, skip
      }
    }
  }

  // Add allowed CORS origins
  const allowedOrigins = process.env.ALLOWED_ORIGINS?.split(",") || [];
  for (const origin of allowedOrigins) {
    const trimmed = origin.trim();
    if (trimmed && trimmed.startsWith("http")) {
      try {
        origins.add(new URL(trimmed).origin);
      } catch {
        // Invalid URL, skip
      }
    }
  }

  return Array.from(origins);
}

/**
 * Validate post_logout_redirect_uri against allowed origins
 * Security: Prevents open redirect attacks (CWE-601) and dangerous protocols
 */
function isValidPostLogoutUri(uri: string): boolean {
  try {
    const parsedUri = new URL(uri);

    // Security: Explicitly reject non-HTTP(S) protocols
    // Prevents javascript:, data:, file: and other dangerous protocols
    if (parsedUri.protocol !== "http:" && parsedUri.protocol !== "https:") {
      return false;
    }

    const allowedOrigins = getAllowedPostLogoutOrigins();

    // Check if origin matches any allowed origin
    return allowedOrigins.includes(parsedUri.origin);
  } catch {
    return false;
  }
}

/**
 * OIDC RP-Initiated Logout Endpoint
 * Handles session termination and optionally redirects to post-logout URI
 *
 * Spec: https://openid.net/specs/openid-connect-rpinitiated-1_0.html
 *
 * Security:
 * - Validates post_logout_redirect_uri against allowed origins
 * - Prevents open redirect attacks (CWE-601)
 * - Clears session cookies securely
 */
async function handleEndSession(request: NextRequest) {
  const url = new URL(request.url);

  // Get query parameters (OIDC RP-Initiated Logout spec)
  // Note: id_token_hint and client_id are optional per spec - captured for logging/future use
  const _idTokenHint = url.searchParams.get("id_token_hint");
  const postLogoutRedirectUri = url.searchParams.get("post_logout_redirect_uri");
  const state = url.searchParams.get("state");
  const _clientId = url.searchParams.get("client_id");

  // Validate post_logout_redirect_uri if provided
  if (postLogoutRedirectUri && !isValidPostLogoutUri(postLogoutRedirectUri)) {
    console.warn("[EndSession] Rejected invalid post_logout_redirect_uri:", postLogoutRedirectUri);
    return NextResponse.json(
      { error: "invalid_request", error_description: "Invalid post_logout_redirect_uri" },
      { status: 400 }
    );
  }

  try {
    // Clear session using Better Auth's sign-out
    // This removes the session from the database and clears cookies
    const cookieStore = await cookies();

    // Get all cookies to forward to Better Auth
    const cookieHeader = cookieStore.getAll()
      .map(c => `${c.name}=${c.value}`)
      .join("; ");

    // Call Better Auth's sign-out endpoint internally
    const signOutResponse = await auth.api.signOut({
      headers: {
        cookie: cookieHeader,
      },
    });

    // Check for signOut errors (Better Auth returns success: true on success)
    if (!signOutResponse?.success) {
      console.warn("[EndSession] SignOut returned non-success:", signOutResponse);
      // Continue anyway - we'll still clear cookies explicitly
    }

    // Build redirect response (URI already validated above)
    if (postLogoutRedirectUri) {
      const redirectUrl = new URL(postLogoutRedirectUri);
      if (state) {
        redirectUrl.searchParams.set("state", state);
      }

      const response = NextResponse.redirect(redirectUrl.toString(), 302);

      // Clear auth cookies explicitly using dynamic prefix
      for (const name of SESSION_COOKIE_NAMES) {
        response.cookies.set(name, "", {
          expires: new Date(0),
          path: "/",
          httpOnly: true,
          secure: process.env.NODE_ENV === "production",
          sameSite: "lax",
        });
      }

      return response;
    }

    // No redirect URI - return success JSON with cookies cleared
    const response = NextResponse.json(
      { success: true, message: "Session terminated" },
      { status: 200 }
    );

    // Clear auth cookies explicitly for consistency with redirect case
    for (const name of SESSION_COOKIE_NAMES) {
      response.cookies.set(name, "", {
        expires: new Date(0),
        path: "/",
        httpOnly: true,
        secure: process.env.NODE_ENV === "production",
        sameSite: "lax",
      });
    }

    return response;
  } catch (error) {
    console.error("[EndSession] Error:", error);

    // Design decision: Redirect even on error for better UX
    // The user initiated logout, so redirect them to a sensible destination
    // rather than showing an error page. The URI was already validated above.
    if (postLogoutRedirectUri) {
      const redirectUrl = new URL(postLogoutRedirectUri);
      if (state) {
        redirectUrl.searchParams.set("state", state);
      }
      const response = NextResponse.redirect(redirectUrl.toString(), 302);

      // Clear cookies even on error to ensure logout completes
      for (const name of SESSION_COOKIE_NAMES) {
        response.cookies.set(name, "", {
          expires: new Date(0),
          path: "/",
          httpOnly: true,
          secure: process.env.NODE_ENV === "production",
          sameSite: "lax",
        });
      }

      return response;
    }

    const response = NextResponse.json(
      { success: false, error: "Failed to end session" },
      { status: 500 }
    );

    // Clear cookies even on error to ensure logout completes
    for (const name of SESSION_COOKIE_NAMES) {
      response.cookies.set(name, "", {
        expires: new Date(0),
        path: "/",
        httpOnly: true,
        secure: process.env.NODE_ENV === "production",
        sameSite: "lax",
      });
    }

    return response;
  }
}

export async function GET(request: NextRequest) {
  return handleEndSession(request);
}

export async function POST(request: NextRequest) {
  return handleEndSession(request);
}
