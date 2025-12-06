import { auth, AUTH_COOKIE_PREFIX } from "@/lib/auth";
import { TRUSTED_CLIENTS } from "@/lib/trusted-clients";
import { cookies } from "next/headers";
import { NextRequest, NextResponse } from "next/server";

/**
 * Better Auth session cookie names
 * Derived from AUTH_COOKIE_PREFIX for consistency with auth config
 *
 * Note: Better Auth uses chunked cookies for large session data, so we need to
 * clear both the base cookie and any chunked variants (e.g., .session_data.0, .session_data.1)
 */
const SESSION_COOKIE_NAMES = [
  `${AUTH_COOKIE_PREFIX}.session_token`,
  `${AUTH_COOKIE_PREFIX}.session_data`,
  `${AUTH_COOKIE_PREFIX}.dont_remember`,
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

  // Get the cookie store first - we'll need it for both success and error paths
  const cookieStore = await cookies();
  const allCookies = cookieStore.getAll();

  // Cookie clearing options - must match how Better Auth sets cookies
  const cookieClearOptions = {
    expires: new Date(0),
    maxAge: 0,
    path: "/",
    httpOnly: true,
    secure: process.env.NODE_ENV === "production",
    sameSite: "lax" as const,
  };

  try {
    // Get all cookies to forward to Better Auth for session lookup
    const cookieHeader = allCookies
      .map(c => `${c.name}=${c.value}`)
      .join("; ");

    // Call Better Auth's sign-out API to invalidate session in database
    // This deletes the session from the database
    await auth.api.signOut({
      headers: {
        cookie: cookieHeader,
      },
    });
  } catch (error) {
    // Log but continue - we still need to clear cookies even if DB deletion fails
    console.error("[EndSession] Error calling signOut:", error);
  }

  // Build response - redirect or JSON based on whether post_logout_redirect_uri was provided
  let response: NextResponse;

  if (postLogoutRedirectUri) {
    const redirectUrl = new URL(postLogoutRedirectUri);
    if (state) {
      redirectUrl.searchParams.set("state", state);
    }
    response = NextResponse.redirect(redirectUrl.toString(), 302);
  } else {
    response = NextResponse.json(
      { success: true, message: "Session terminated" },
      { status: 200 }
    );
  }

  // Clear all known session cookies
  // This is the critical part - we MUST clear cookies regardless of signOut API success
  for (const name of SESSION_COOKIE_NAMES) {
    response.cookies.set(name, "", cookieClearOptions);
  }

  // Also clear any chunked cookies that Better Auth may have created
  // Better Auth uses chunking for large session data: cookie.0, cookie.1, etc.
  for (const cookie of allCookies) {
    if (cookie.name.startsWith(`${AUTH_COOKIE_PREFIX}.`)) {
      response.cookies.set(cookie.name, "", cookieClearOptions);
    }
  }

  return response;
}

export async function GET(request: NextRequest) {
  return handleEndSession(request);
}

export async function POST(request: NextRequest) {
  return handleEndSession(request);
}
