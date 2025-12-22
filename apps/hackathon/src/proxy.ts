import { NextResponse } from "next/server";
import type { NextRequest } from "next/server";

// Routes that require authentication
const protectedPaths = ["/dashboard", "/hackathons"];

// Public paths that should bypass auth checks
const publicPaths = [
  "/",
  "/login",
  "/api/auth",
  "/h/", // Public hackathon pages
  "/explore", // Public explore page
];

/**
 * Next.js 16 Proxy - replaces middleware.ts
 *
 * Runs on Node.js runtime (not Edge) before routes are rendered.
 * Used for routing decisions: redirects, rewrites, header/cookie manipulation.
 *
 * @see https://nextjs.org/docs/app/api-reference/file-conventions/proxy
 */
export function proxy(request: NextRequest) {
  const { pathname } = request.nextUrl;

  // Skip proxy for public paths
  if (publicPaths.some((path) => pathname.startsWith(path))) {
    return NextResponse.next();
  }

  // Check if path requires authentication
  const requiresAuth = protectedPaths.some((path) => pathname.startsWith(path));

  if (requiresAuth) {
    // Check for session cookie presence
    // Note: Full session validation happens in route handlers/layouts
    // Proxy handles the redirect to login for unauthenticated requests
    const sessionCookie = request.cookies.get("hackathon-session");

    if (!sessionCookie?.value) {
      // No session cookie - redirect to login
      const loginUrl = new URL("/login", request.url);
      loginUrl.searchParams.set("returnUrl", pathname);
      return NextResponse.redirect(loginUrl);
    }
  }

  return NextResponse.next();
}

export const config = {
  matcher: [
    /*
     * Match all request paths except:
     * - _next/static (static files)
     * - _next/image (image optimization files)
     * - favicon.ico (favicon file)
     * - public folder assets
     * - sitemap.xml and robots.txt
     */
    "/((?!_next/static|_next/image|favicon.ico|public|sitemap.xml|robots.txt).*)",
  ],
};
