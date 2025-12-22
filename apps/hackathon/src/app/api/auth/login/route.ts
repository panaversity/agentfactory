import { NextRequest, NextResponse } from "next/server";
import { createAuthorizationUrl } from "@/lib/auth/oauth-client";
import { getSession } from "@/lib/auth/session";

export async function GET(request: NextRequest) {
  try {
    // Get returnUrl from query params
    const returnUrl = request.nextUrl.searchParams.get("returnUrl");

    // Generate PKCE authorization URL
    const { url, state, codeVerifier } = createAuthorizationUrl();

    // Store state, code verifier, and returnUrl in session for callback
    const session = await getSession();
    session.oauthState = state;
    session.oauthCodeVerifier = codeVerifier;
    if (returnUrl) {
      session.returnUrl = returnUrl;
    }
    await session.save();

    // Redirect to SSO authorization endpoint
    return NextResponse.redirect(url);
  } catch (error) {
    console.error("Login error:", error);
    return NextResponse.json(
      { error: "Failed to initiate login" },
      { status: 500 }
    );
  }
}
