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
 */
export interface SSOJWTPayload extends JWTPayload {
  sub: string;
  email: string;
  name: string;
  username: string; // Required for user lookup
  image?: string; // Profile image URL
  tenant_id: string; // Organization ID
  tenant_name?: string;
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
 */
export function extractUserFromToken(payload: SSOJWTPayload) {
  return {
    id: payload.sub,
    email: payload.email,
    name: payload.name,
    username: payload.username,
    image: payload.image,
    organizationId: payload.tenant_id,
    organizationName: payload.tenant_name,
  };
}
