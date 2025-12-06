import { auth } from "@/lib/auth";

/**
 * OIDC Discovery endpoint - /.well-known/openid-configuration
 *
 * Proxies to Better Auth's OIDC Provider which dynamically generates
 * discovery metadata from the auth configuration. No hardcoded values.
 *
 * Spec: https://openid.net/specs/openid-connect-discovery-1_0.html
 */
export async function GET(request: Request) {
  // Proxy to Better Auth's OIDC discovery endpoint
  // The OIDC provider plugin exposes this at /.well-known/openid-configuration
  const response = await auth.handler(
    new Request(
      new URL("/api/auth/.well-known/openid-configuration", request.url),
      { method: "GET", headers: request.headers }
    )
  );

  if (!response.ok) {
    console.error("[OIDC Discovery] Failed to fetch from Better Auth:", {
      status: response.status,
      statusText: response.statusText,
    });
    return Response.json(
      { error: "Failed to fetch OIDC configuration" },
      { status: 500 }
    );
  }

  const metadata = await response.json();

  return Response.json(metadata, {
    headers: {
      "Cache-Control": "public, max-age=3600",
      "Content-Type": "application/json",
    },
  });
}
