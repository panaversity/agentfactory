import { NextRequest, NextResponse } from "next/server";
import { destroySession } from "@/lib/auth/session";

export async function GET(request: NextRequest) {
  try {
    // Destroy local session
    await destroySession();

    // Redirect to home page after logout
    // Note: For full SSO logout, would need to call SSO endsession endpoint
    return NextResponse.redirect(new URL("/", request.url));
  } catch (error) {
    console.error("Logout error:", error);
    // Even if there's an error, redirect to home
    return NextResponse.redirect(new URL("/", request.url));
  }
}

export async function POST(request: NextRequest) {
  // Also support POST for CSRF-safe logout
  return GET(request);
}
