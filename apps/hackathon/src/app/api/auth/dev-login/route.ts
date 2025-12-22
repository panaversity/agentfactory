import { cookies } from "next/headers";
import { getIronSession } from "iron-session";
import { NextResponse } from "next/server";
import { SessionData, sessionOptions } from "@/lib/auth/session";

/**
 * DEV ONLY: Creates a mock session for testing without SSO
 * This should be disabled in production
 */
export async function GET() {
  if (process.env.NODE_ENV === "production") {
    return NextResponse.json({ error: "Not available in production" }, { status: 403 });
  }

  const session = await getIronSession<SessionData>(
    await cookies(),
    sessionOptions
  );

  // Create a mock user session
  session.isLoggedIn = true;
  session.user = {
    id: "dev-user-001",
    email: "dev@panaversity.org",
    name: "Dev User",
    username: "devuser",
    image: undefined,
    organizationId: "org-001",
    organizationName: "Panaversity",
    organizationRole: "admin",
  };

  await session.save();

  return NextResponse.redirect(new URL("/dashboard", process.env.NEXT_PUBLIC_APP_URL || "http://localhost:3002"));
}
