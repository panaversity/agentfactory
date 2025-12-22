import { NextRequest, NextResponse } from "next/server";
import { exchangeCodeForTokens } from "@/lib/auth/oauth-client";
import { getSession, createSession } from "@/lib/auth/session";
import { verifyIdToken, extractUserFromToken } from "@/lib/auth/jwt-verify";

export async function GET(request: NextRequest) {
  try {
    const searchParams = request.nextUrl.searchParams;
    const code = searchParams.get("code");
    const state = searchParams.get("state");
    const error = searchParams.get("error");

    // Handle OAuth errors
    if (error) {
      console.error("OAuth error:", error, searchParams.get("error_description"));
      return NextResponse.redirect(
        new URL(`/login?error=${encodeURIComponent(error)}`, request.url)
      );
    }

    // Validate required parameters
    if (!code || !state) {
      return NextResponse.redirect(
        new URL("/login?error=missing_params", request.url)
      );
    }

    // Get session and verify state
    const session = await getSession();
    if (state !== session.oauthState) {
      console.error("State mismatch:", { received: state, expected: session.oauthState });
      return NextResponse.redirect(
        new URL("/login?error=state_mismatch", request.url)
      );
    }

    // Get code verifier
    const codeVerifier = session.oauthCodeVerifier;
    if (!codeVerifier) {
      return NextResponse.redirect(
        new URL("/login?error=missing_verifier", request.url)
      );
    }

    // Exchange code for tokens
    const tokens = await exchangeCodeForTokens(code, codeVerifier);

    // Get ID token from response
    const idToken = tokens.idToken();
    const accessToken = tokens.accessToken();
    const expiresAt = tokens.accessTokenExpiresAt()?.getTime() ?? Date.now() + 3600 * 1000;

    if (!idToken) {
      return NextResponse.redirect(
        new URL("/login?error=no_id_token", request.url)
      );
    }

    // Verify ID token and extract user info
    const payload = await verifyIdToken(idToken);
    const user = extractUserFromToken(payload);

    // Get returnUrl before creating session (which clears it)
    const returnUrl = session.returnUrl;

    // Create session with user data
    await createSession(user, {
      accessToken,
      idToken,
      expiresAt,
    });

    // Redirect to returnUrl or dashboard
    const redirectTo = returnUrl && returnUrl.startsWith("/") ? returnUrl : "/dashboard";
    return NextResponse.redirect(new URL(redirectTo, request.url));
  } catch (error) {
    console.error("Callback error:", error);
    return NextResponse.redirect(
      new URL("/login?error=auth_failed", request.url)
    );
  }
}
