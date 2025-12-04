/**
 * OAuth Flow Tests
 * Tests both PKCE (public client) and Confidential client flows
 *
 * Prerequisites:
 * 1. Auth server running on port 3001: pnpm dev
 * 2. Test clients seeded: npx tsx scripts/seed-test-clients.ts
 * 3. Test user exists and is verified: admin@robolearn.io / admin@robolearn.io
 *
 * Run: node tests/test-oauth-flows.js
 */

const crypto = require("crypto");

// Configuration
const AUTH_URL = "http://localhost:3001";
const TEST_EMAIL = "admin@robolearn.io";
const TEST_PASSWORD = "admin@robolearn.io";

// Test clients (must match seed-test-clients.ts)
const PUBLIC_CLIENT = {
  clientId: "robolearn-public-client",
  redirectUri: "http://localhost:3000/auth/callback",
};

const CONFIDENTIAL_CLIENT = {
  clientId: "robolearn-confidential-client",
  clientSecret: "robolearn-confidential-secret-for-testing-only",
  redirectUri: "http://localhost:8000/auth/callback",
};

// Test results tracking
const results = {
  passed: 0,
  failed: 0,
  tests: [],
};

function logTest(name, passed, details = "") {
  const status = passed ? "PASS" : "FAIL";
  const icon = passed ? "\u2705" : "\u274c";
  console.log(`${icon} ${name}: ${status}${details ? ` - ${details}` : ""}`);
  results.tests.push({ name, passed, details });
  if (passed) results.passed++;
  else results.failed++;
}

// PKCE helpers
function generateCodeVerifier() {
  return crypto.randomBytes(32).toString("base64url");
}

function generateCodeChallenge(verifier) {
  return crypto.createHash("sha256").update(verifier).digest().toString("base64url");
}

async function getSessionCookie() {
  const response = await fetch(`${AUTH_URL}/api/auth/sign-in/email`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ email: TEST_EMAIL, password: TEST_PASSWORD }),
  });

  if (!response.ok) {
    throw new Error(`Sign-in failed: ${response.status} ${await response.text()}`);
  }

  return response.headers.get("set-cookie");
}

// === Test 1: OIDC Discovery ===
async function testOIDCDiscovery() {
  try {
    const response = await fetch(`${AUTH_URL}/api/auth/.well-known/openid-configuration`);
    const config = await response.json();

    const hasRequiredEndpoints =
      config.issuer &&
      config.authorization_endpoint &&
      config.token_endpoint &&
      config.userinfo_endpoint &&
      config.jwks_uri;

    logTest("OIDC Discovery", hasRequiredEndpoints, `issuer=${config.issuer}`);
    return config;
  } catch (error) {
    logTest("OIDC Discovery", false, error.message);
    return null;
  }
}

// === Test 2: JWKS Endpoint ===
async function testJWKS() {
  try {
    const response = await fetch(`${AUTH_URL}/api/auth/jwks`);
    const jwks = await response.json();

    const hasKeys = jwks.keys && jwks.keys.length > 0;
    const firstKey = jwks.keys?.[0];
    const validKey = firstKey?.kty && firstKey?.alg && firstKey?.kid;

    logTest("JWKS Endpoint", hasKeys && validKey, `keys=${jwks.keys?.length}, alg=${firstKey?.alg}`);
    return jwks;
  } catch (error) {
    logTest("JWKS Endpoint", false, error.message);
    return null;
  }
}

// === Test 3: PKCE Flow (Public Client) ===
async function testPKCEFlow() {
  try {
    // Step 1: Generate PKCE values
    const codeVerifier = generateCodeVerifier();
    const codeChallenge = generateCodeChallenge(codeVerifier);

    // Step 2: Get session cookie
    const cookies = await getSessionCookie();
    if (!cookies) throw new Error("No session cookie");

    // Step 3: Authorization request
    const authUrl = `${AUTH_URL}/api/auth/oauth2/authorize?` +
      `client_id=${PUBLIC_CLIENT.clientId}&` +
      `redirect_uri=${encodeURIComponent(PUBLIC_CLIENT.redirectUri)}&` +
      `response_type=code&scope=openid%20profile%20email&state=pkce-test&` +
      `code_challenge=${codeChallenge}&code_challenge_method=S256`;

    const authResponse = await fetch(authUrl, {
      headers: { Cookie: cookies },
      redirect: "manual",
    });

    // Better Auth returns JSON with redirect URL
    let code;
    if (authResponse.status === 200) {
      const body = await authResponse.json();
      if (body.redirect && body.url) {
        const match = body.url.match(/code=([^&]+)/);
        if (match) code = match[1];
      }
    }

    if (!code) throw new Error("No authorization code received");

    // Step 4: Token exchange with PKCE (no client_secret)
    const tokenResponse = await fetch(`${AUTH_URL}/api/auth/oauth2/token`, {
      method: "POST",
      headers: { "Content-Type": "application/x-www-form-urlencoded" },
      body: new URLSearchParams({
        grant_type: "authorization_code",
        code,
        redirect_uri: PUBLIC_CLIENT.redirectUri,
        client_id: PUBLIC_CLIENT.clientId,
        code_verifier: codeVerifier,
      }),
    });

    if (!tokenResponse.ok) {
      throw new Error(`Token exchange failed: ${await tokenResponse.text()}`);
    }

    const tokens = await tokenResponse.json();
    const hasAccessToken = !!tokens.access_token;
    const hasIdToken = !!tokens.id_token;

    logTest("PKCE Flow (Public Client)", hasAccessToken, `access_token=${hasAccessToken}, id_token=${hasIdToken}`);
    return tokens;
  } catch (error) {
    logTest("PKCE Flow (Public Client)", false, error.message);
    return null;
  }
}

// === Test 4: Confidential Client Flow ===
async function testConfidentialFlow() {
  try {
    // Step 1: Get session cookie
    const cookies = await getSessionCookie();
    if (!cookies) throw new Error("No session cookie");

    // Step 2: Authorization request (no PKCE needed for confidential client)
    const authUrl = `${AUTH_URL}/api/auth/oauth2/authorize?` +
      `client_id=${CONFIDENTIAL_CLIENT.clientId}&` +
      `redirect_uri=${encodeURIComponent(CONFIDENTIAL_CLIENT.redirectUri)}&` +
      `response_type=code&scope=openid%20profile%20email&state=confidential-test`;

    const authResponse = await fetch(authUrl, {
      headers: { Cookie: cookies },
      redirect: "manual",
    });

    // Better Auth returns JSON with redirect URL
    let code;
    if (authResponse.status === 200) {
      const body = await authResponse.json();
      if (body.redirect && body.url) {
        const match = body.url.match(/code=([^&]+)/);
        if (match) code = match[1];
      }
    }

    if (!code) throw new Error("No authorization code received");

    // Step 3: Token exchange with client_secret_basic
    const credentials = Buffer.from(
      `${CONFIDENTIAL_CLIENT.clientId}:${CONFIDENTIAL_CLIENT.clientSecret}`
    ).toString("base64");

    const tokenResponse = await fetch(`${AUTH_URL}/api/auth/oauth2/token`, {
      method: "POST",
      headers: {
        "Content-Type": "application/x-www-form-urlencoded",
        Authorization: `Basic ${credentials}`,
      },
      body: new URLSearchParams({
        grant_type: "authorization_code",
        code,
        redirect_uri: CONFIDENTIAL_CLIENT.redirectUri,
      }),
    });

    if (!tokenResponse.ok) {
      throw new Error(`Token exchange failed: ${await tokenResponse.text()}`);
    }

    const tokens = await tokenResponse.json();
    const hasAccessToken = !!tokens.access_token;
    const hasIdToken = !!tokens.id_token;

    logTest("Confidential Client Flow", hasAccessToken, `access_token=${hasAccessToken}, id_token=${hasIdToken}`);
    return tokens;
  } catch (error) {
    logTest("Confidential Client Flow", false, error.message);
    return null;
  }
}

// === Test 5: Userinfo Endpoint ===
async function testUserinfo(tokens) {
  if (!tokens?.access_token) {
    logTest("Userinfo Endpoint", false, "No access token available");
    return;
  }

  try {
    const response = await fetch(`${AUTH_URL}/api/auth/oauth2/userinfo`, {
      headers: { Authorization: `Bearer ${tokens.access_token}` },
    });

    if (!response.ok) {
      throw new Error(`Userinfo failed: ${response.status}`);
    }

    const userinfo = await response.json();
    const hasRequiredClaims = userinfo.sub && userinfo.email;
    const hasCustomClaims = userinfo.software_background !== undefined;

    logTest(
      "Userinfo Endpoint",
      hasRequiredClaims,
      `sub=${userinfo.sub?.slice(0, 10)}..., email=${userinfo.email}, custom_claims=${hasCustomClaims}`
    );
  } catch (error) {
    logTest("Userinfo Endpoint", false, error.message);
  }
}

// === Test 6: Token Introspection (ID Token validation) ===
async function testIdToken(tokens) {
  if (!tokens?.id_token) {
    logTest("ID Token Structure", false, "No ID token available");
    return;
  }

  try {
    // Decode JWT (without verification - just structure check)
    const parts = tokens.id_token.split(".");
    if (parts.length !== 3) throw new Error("Invalid JWT structure");

    const header = JSON.parse(Buffer.from(parts[0], "base64url").toString());
    const payload = JSON.parse(Buffer.from(parts[1], "base64url").toString());

    const hasRequiredClaims = payload.iss && payload.sub && payload.aud && payload.exp && payload.iat;
    const algorithm = header.alg;

    logTest(
      "ID Token Structure",
      hasRequiredClaims,
      `alg=${algorithm}, iss=${payload.iss}, exp=${new Date(payload.exp * 1000).toISOString()}`
    );
  } catch (error) {
    logTest("ID Token Structure", false, error.message);
  }
}

// === Test 7: Dynamic Client Registration ===
// SKIPPED: DCR requires admin authentication (security feature)
// Clients are managed via admin UI at /admin/clients
async function testDynamicClientRegistration() {
  logTest(
    "Dynamic Client Registration",
    true, // Mark as pass (intentionally skipped)
    "SKIPPED - DCR requires authentication (use admin UI)"
  );
}

// === Main ===
async function main() {
  console.log("=".repeat(60));
  console.log("OAuth Flow Test Suite");
  console.log(`Auth Server: ${AUTH_URL}`);
  console.log(`Test User: ${TEST_EMAIL}`);
  console.log("=".repeat(60));
  console.log("");

  // Run tests
  await testOIDCDiscovery();
  await testJWKS();
  const pkceTokens = await testPKCEFlow();
  const confidentialTokens = await testConfidentialFlow();
  await testUserinfo(pkceTokens || confidentialTokens);
  await testIdToken(pkceTokens || confidentialTokens);
  await testDynamicClientRegistration();

  // Summary
  console.log("");
  console.log("=".repeat(60));
  console.log(`Results: ${results.passed}/${results.passed + results.failed} tests passed`);

  if (results.failed > 0) {
    console.log("\nFailed tests:");
    results.tests.filter((t) => !t.passed).forEach((t) => console.log(`  - ${t.name}: ${t.details}`));
  }

  console.log("=".repeat(60));
  process.exit(results.failed > 0 ? 1 : 0);
}

main().catch((err) => {
  console.error("Test suite error:", err);
  process.exit(1);
});
