import { createRemoteJWKSet, jwtVerify, JWTPayload } from "jose";

// Cached JWKS
let jwks: ReturnType<typeof createRemoteJWKSet> | null = null;

function getJWKS() {
  if (!jwks) {
    jwks = createRemoteJWKSet(new URL(process.env.SSO_OAUTH_JWKS_URL!));
  }
  return jwks;
}

/**
 * Expected JWT claims from SSO
 * Note: SSO uses OIDC standard claim names (preferred_username, picture)
 */
export interface SSOJWTPayload extends JWTPayload {
  sub: string;
  email: string;
  name: string;
  preferred_username: string; // OIDC standard - Required for user lookup
  picture?: string; // OIDC standard - Profile image URL
  tenant_id: string; // Organization ID
  tenant_name?: string;
  org_role?: string; // Organization role (member/admin/owner)
}

/**
 * Verify ID token from SSO
 * @param token - JWT ID token
 * @returns Verified payload with user claims
 */
export async function verifyIdToken(token: string): Promise<SSOJWTPayload> {
  const jwks = getJWKS();

  const { payload } = await jwtVerify(token, jwks, {
    issuer: process.env.SSO_OAUTH_ISSUER,
    audience: process.env.SSO_OAUTH_CLIENT_ID,
  });

  return payload as SSOJWTPayload;
}

/**
 * Verify access token (for API calls if needed)
 * @param token - JWT access token
 * @returns Verified payload
 */
export async function verifyAccessToken(token: string): Promise<JWTPayload> {
  const jwks = getJWKS();

  const { payload } = await jwtVerify(token, jwks, {
    issuer: process.env.SSO_OAUTH_ISSUER,
  });

  return payload;
}

/**
 * Extract user info from verified ID token
 * Throws if required fields are missing
 */
export function extractUserFromToken(payload: SSOJWTPayload) {
  // Validate required fields
  if (!payload.sub) {
    throw new Error("Missing user ID (sub) in token");
  }
  if (!payload.preferred_username) {
    throw new Error("Missing username in token - user must complete SSO profile");
  }
  if (!payload.email) {
    throw new Error("Missing email in token");
  }
  if (!payload.tenant_id) {
    throw new Error("Missing organization (tenant_id) in token");
  }

  return {
    id: payload.sub,
    email: payload.email,
    name: payload.name || payload.preferred_username, // Fallback to username if name missing
    username: payload.preferred_username, // Map OIDC preferred_username to internal username
    image: payload.picture, // Map OIDC picture to internal image
    organizationId: payload.tenant_id,
    organizationName: payload.tenant_name,
  };
}
