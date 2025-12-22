import { getIronSession, IronSession, SessionOptions } from "iron-session";
import { cookies } from "next/headers";

/**
 * Session data stored in encrypted cookie
 */
export interface SessionData {
  // OAuth flow state (temporary)
  oauthState?: string;
  oauthCodeVerifier?: string;
  returnUrl?: string;

  // User data after successful login
  isLoggedIn: boolean;
  user?: {
    id: string;
    email: string;
    name: string;
    username: string; // Required for user lookups
    image?: string; // Profile image URL
    organizationId: string;
    organizationName?: string;
    organizationRole?: "admin" | "manager" | "member";
  };

  // Token data
  accessToken?: string;
  idToken?: string;
  expiresAt?: number;
}

export const sessionOptions: SessionOptions = {
  password: process.env.SESSION_SECRET!,
  cookieName: "hackathon-session",
  cookieOptions: {
    secure: process.env.NODE_ENV === "production",
    httpOnly: true,
    sameSite: "lax",
    maxAge: 60 * 60 * 24 * 7, // 7 days
  },
};

/**
 * Get the current session from request cookies
 */
export async function getSession(): Promise<IronSession<SessionData>> {
  const cookieStore = await cookies();
  return getIronSession<SessionData>(cookieStore, sessionOptions);
}

/**
 * Create a new session with user data
 */
export async function createSession(userData: SessionData["user"], tokens: { accessToken: string; idToken: string; expiresAt: number }) {
  const session = await getSession();

  // Clear OAuth state
  session.oauthState = undefined;
  session.oauthCodeVerifier = undefined;

  // Set user data
  session.isLoggedIn = true;
  session.user = userData;
  session.accessToken = tokens.accessToken;
  session.idToken = tokens.idToken;
  session.expiresAt = tokens.expiresAt;

  await session.save();
  return session;
}

/**
 * Destroy the current session (logout)
 */
export async function destroySession() {
  const session = await getSession();
  session.destroy();
}

/**
 * Get the current user from session (returns undefined if not logged in)
 */
export async function getCurrentUser() {
  const session = await getSession();
  if (!session.isLoggedIn || !session.user) {
    return undefined;
  }
  return session.user;
}

/**
 * Require authentication - throws if not logged in
 */
export async function requireAuth() {
  const user = await getCurrentUser();
  if (!user) {
    throw new Error("Unauthorized");
  }
  return user;
}
