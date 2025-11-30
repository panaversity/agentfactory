/**
 * JWT Verification Utility using JWKS
 * 
 * Verifies ID tokens client-side using JWKS public keys from the auth server.
 * This reduces server load by eliminating per-request userinfo calls.
 */

interface JWKSKey {
  kid: string;
  kty: string;
  use?: string;
  alg: string;
  n?: string; // RSA modulus
  e?: string; // RSA exponent
  crv?: string; // Elliptic curve
  x?: string; // EC x coordinate
  y?: string; // EC y coordinate
}

interface JWKSResponse {
  keys: JWKSKey[];
}

interface JWTPayload {
  sub: string;
  email?: string;
  name?: string;
  role?: string;
  software_background?: string;
  exp: number;
  iat: number;
  iss: string;
  aud: string;
}

// Cache JWKS keys (they don't change often)
let jwksCache: JWKSKey[] | null = null;
let jwksCacheExpiry: number = 0;
const JWKS_CACHE_TTL = 60 * 60 * 1000; // 1 hour

/**
 * Fetch JWKS from auth server
 */
async function fetchJWKS(authUrl: string): Promise<JWKSKey[]> {
  const now = Date.now();
  
  // Return cached JWKS if still valid
  if (jwksCache && now < jwksCacheExpiry) {
    return jwksCache;
  }

  try {
    const response = await fetch(`${authUrl}/api/auth/jwks`);
    if (!response.ok) {
      throw new Error(`Failed to fetch JWKS: ${response.status}`);
    }

    const jwks: JWKSResponse = await response.json();
    jwksCache = jwks.keys;
    jwksCacheExpiry = now + JWKS_CACHE_TTL;
    return jwks.keys;
  } catch (error) {
    console.error('Failed to fetch JWKS:', error);
    // Return cached keys if available, even if expired
    if (jwksCache) {
      return jwksCache;
    }
    throw error;
  }
}

/**
 * Base64 URL decode
 */
function base64UrlDecode(str: string): Uint8Array {
  // Add padding if needed
  str = str.replace(/-/g, '+').replace(/_/g, '/');
  while (str.length % 4) {
    str += '=';
  }
  
  const binary = atob(str);
  const bytes = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i++) {
    bytes[i] = binary.charCodeAt(i);
  }
  return bytes;
}

/**
 * Parse JWT without verification (for extracting header and payload)
 */
function parseJWT(token: string): { header: any; payload: JWTPayload; signature: string } {
  const parts = token.split('.');
  if (parts.length !== 3) {
    throw new Error('Invalid JWT format');
  }

  const [headerB64, payloadB64, signatureB64] = parts;

  const header = JSON.parse(new TextDecoder().decode(base64UrlDecode(headerB64)));
  const payload = JSON.parse(new TextDecoder().decode(base64UrlDecode(payloadB64)));

  return { header, payload, signature: signatureB64 };
}

/**
 * Convert JWK to CryptoKey for verification
 */
async function jwkToCryptoKey(jwk: JWKSKey): Promise<CryptoKey> {
  if (jwk.kty === 'RSA') {
    // RSA key (RS256)
    const keyData = {
      kty: jwk.kty,
      n: jwk.n,
      e: jwk.e,
      alg: jwk.alg || 'RS256',
      use: jwk.use || 'sig',
    };

    return await crypto.subtle.importKey(
      'jwk',
      keyData as any,
      {
        name: 'RSASSA-PKCS1-v1_5',
        hash: 'SHA-256',
      },
      false,
      ['verify']
    );
  } else if (jwk.kty === 'EC') {
    // Elliptic curve key (ES256)
    const keyData = {
      kty: jwk.kty,
      crv: jwk.crv,
      x: jwk.x,
      y: jwk.y,
      alg: jwk.alg || 'ES256',
      use: jwk.use || 'sig',
    };

    return await crypto.subtle.importKey(
      'jwk',
      keyData as any,
      {
        name: 'ECDSA',
        namedCurve: jwk.crv || 'P-256',
      },
      false,
      ['verify']
    );
  } else {
    throw new Error(`Unsupported key type: ${jwk.kty}`);
  }
}

/**
 * Verify JWT signature using JWKS
 */
async function verifyJWTSignature(
  token: string,
  signature: string,
  header: any,
  jwks: JWKSKey[]
): Promise<boolean> {
  // Find the key by kid (key ID)
  const key = jwks.find(k => k.kid === header.kid);
  if (!key) {
    throw new Error(`Key with kid ${header.kid} not found in JWKS`);
  }

  // Convert JWK to CryptoKey
  const cryptoKey = await jwkToCryptoKey(key);

  // Prepare data for verification (header.payload)
  const parts = token.split('.');
  const data = new TextEncoder().encode(`${parts[0]}.${parts[1]}`);
  const signatureBytes = base64UrlDecode(signature);

  // Verify signature
  return await crypto.subtle.verify(
    {
      name: key.kty === 'RSA' ? 'RSASSA-PKCS1-v1_5' : 'ECDSA',
      hash: header.alg === 'RS256' || header.alg === 'ES256' ? 'SHA-256' : 'SHA-384',
    },
    cryptoKey,
    signatureBytes,
    data
  );
}

/**
 * Verify ID token and extract user info
 * 
 * @param idToken - The ID token from OAuth flow
 * @param authUrl - Auth server URL
 * @param expectedAudience - Expected audience (client ID)
 * @returns User info if token is valid, null otherwise
 */
export async function verifyIDToken(
  idToken: string,
  authUrl: string,
  expectedAudience: string
): Promise<JWTPayload | null> {
  try {
    // Parse JWT
    const { header, payload, signature } = parseJWT(idToken);

    // Check algorithm
    if (header.alg !== 'RS256' && header.alg !== 'ES256') {
      console.warn(`Unsupported algorithm: ${header.alg}`);
      return null;
    }

    // Check expiration
    const now = Math.floor(Date.now() / 1000);
    if (payload.exp && payload.exp < now) {
      console.warn('Token expired');
      return null;
    }

    // Check audience
    if (payload.aud !== expectedAudience) {
      console.warn(`Token audience mismatch: expected ${expectedAudience}, got ${payload.aud}`);
      return null;
    }

    // Fetch JWKS
    const jwks = await fetchJWKS(authUrl);

    // Verify signature
    const isValid = await verifyJWTSignature(idToken, signature, header, jwks);
    if (!isValid) {
      console.warn('Token signature verification failed');
      return null;
    }

    return payload;
  } catch (error) {
    console.error('ID token verification failed:', error);
    return null;
  }
}

/**
 * Extract user info from verified ID token payload
 */
export function extractUserFromToken(payload: JWTPayload) {
  return {
    id: payload.sub,
    email: payload.email,
    name: payload.name,
    role: payload.role,
    softwareBackground: payload.software_background,
  };
}

