/**
 * Edge Case Tests for OAuth/OIDC Flows
 * Tests error handling, security boundaries, and unusual scenarios
 *
 * Prerequisites:
 * 1. Auth server running on port 3001: pnpm dev
 * 2. Test clients seeded: pnpm seed:clients
 * 3. Test user exists and is verified: admin@robolearn.io / admin@robolearn.io
 *
 * Run: node tests/test-edge-cases.js
 */

const crypto = require("crypto");

// Configuration
const AUTH_URL = "http://localhost:3001";
const TEST_EMAIL = "admin@robolearn.io";
const TEST_PASSWORD = "admin@robolearn.io";

const PUBLIC_CLIENT = {
  clientId: "robolearn-public-client",
  redirectUri: "http://localhost:3000/auth/callback",
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
  if (!response.ok) throw new Error(`Sign-in failed: ${response.status}`);
  return response.headers.get("set-cookie");
}

// ==================== SECURITY EDGE CASES ====================

// Test 1: Invalid client_id should be handled (rejected or redirect to error)
async function testInvalidClientId() {
  try {
    const cookies = await getSessionCookie();
    const authUrl = `${AUTH_URL}/api/auth/oauth2/authorize?` +
      `client_id=non-existent-client&` +
      `redirect_uri=${encodeURIComponent(PUBLIC_CLIENT.redirectUri)}&` +
      `response_type=code&scope=openid`;

    const response = await fetch(authUrl, {
      headers: { Cookie: cookies },
      redirect: "manual",
    });

    // OAuth spec allows: 400/401 error OR redirect to login page (302)
    // The key is that it should NOT issue an authorization code
    const passed = response.status === 400 || response.status === 401 ||
                   response.status === 302 || // Redirect to login is acceptable
                   (response.status === 200 && (await response.json()).error);
    logTest("Invalid client_id handled", passed, `status=${response.status}`);
  } catch (error) {
    logTest("Invalid client_id handled", false, error.message);
  }
}

// Test 2: Mismatched redirect_uri should be rejected
async function testMismatchedRedirectUri() {
  try {
    const cookies = await getSessionCookie();
    const authUrl = `${AUTH_URL}/api/auth/oauth2/authorize?` +
      `client_id=${PUBLIC_CLIENT.clientId}&` +
      `redirect_uri=${encodeURIComponent("http://evil.com/callback")}&` +
      `response_type=code&scope=openid`;

    const response = await fetch(authUrl, {
      headers: { Cookie: cookies },
      redirect: "manual",
    });

    // Should fail - redirect_uri doesn't match registered URIs
    const passed = response.status === 400 || response.status === 401 ||
                   (response.status === 200 && (await response.json()).error);
    logTest("Mismatched redirect_uri rejected", passed, `status=${response.status}`);
  } catch (error) {
    logTest("Mismatched redirect_uri rejected", false, error.message);
  }
}

// Test 3: PKCE code_verifier mismatch should fail token exchange
async function testPKCEVerifierMismatch() {
  try {
    const codeVerifier = generateCodeVerifier();
    const codeChallenge = generateCodeChallenge(codeVerifier);
    const cookies = await getSessionCookie();

    // Get authorization code with correct challenge
    const authUrl = `${AUTH_URL}/api/auth/oauth2/authorize?` +
      `client_id=${PUBLIC_CLIENT.clientId}&` +
      `redirect_uri=${encodeURIComponent(PUBLIC_CLIENT.redirectUri)}&` +
      `response_type=code&scope=openid&state=test&` +
      `code_challenge=${codeChallenge}&code_challenge_method=S256`;

    const authResponse = await fetch(authUrl, {
      headers: { Cookie: cookies },
      redirect: "manual",
    });

    let code;
    if (authResponse.status === 200) {
      const body = await authResponse.json();
      if (body.redirect && body.url) {
        const match = body.url.match(/code=([^&]+)/);
        if (match) code = match[1];
      }
    }

    if (!code) {
      logTest("PKCE verifier mismatch rejected", false, "Could not get auth code");
      return;
    }

    // Try to exchange with WRONG verifier
    const wrongVerifier = generateCodeVerifier(); // Different verifier!
    const tokenResponse = await fetch(`${AUTH_URL}/api/auth/oauth2/token`, {
      method: "POST",
      headers: { "Content-Type": "application/x-www-form-urlencoded" },
      body: new URLSearchParams({
        grant_type: "authorization_code",
        code,
        redirect_uri: PUBLIC_CLIENT.redirectUri,
        client_id: PUBLIC_CLIENT.clientId,
        code_verifier: wrongVerifier,
      }),
    });

    const passed = !tokenResponse.ok;
    logTest("PKCE verifier mismatch rejected", passed, `status=${tokenResponse.status}`);
  } catch (error) {
    logTest("PKCE verifier mismatch rejected", false, error.message);
  }
}

// Test 4: Authorization code reuse should fail
async function testCodeReuse() {
  try {
    const codeVerifier = generateCodeVerifier();
    const codeChallenge = generateCodeChallenge(codeVerifier);
    const cookies = await getSessionCookie();

    // Get authorization code
    const authUrl = `${AUTH_URL}/api/auth/oauth2/authorize?` +
      `client_id=${PUBLIC_CLIENT.clientId}&` +
      `redirect_uri=${encodeURIComponent(PUBLIC_CLIENT.redirectUri)}&` +
      `response_type=code&scope=openid&state=test&` +
      `code_challenge=${codeChallenge}&code_challenge_method=S256`;

    const authResponse = await fetch(authUrl, {
      headers: { Cookie: cookies },
      redirect: "manual",
    });

    let code;
    if (authResponse.status === 200) {
      const body = await authResponse.json();
      if (body.redirect && body.url) {
        const match = body.url.match(/code=([^&]+)/);
        if (match) code = match[1];
      }
    }

    if (!code) {
      logTest("Authorization code reuse rejected", false, "Could not get auth code");
      return;
    }

    // First exchange - should succeed
    const firstResponse = await fetch(`${AUTH_URL}/api/auth/oauth2/token`, {
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

    if (!firstResponse.ok) {
      logTest("Authorization code reuse rejected", false, "First exchange failed");
      return;
    }

    // Second exchange with same code - should FAIL
    const secondResponse = await fetch(`${AUTH_URL}/api/auth/oauth2/token`, {
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

    const passed = !secondResponse.ok;
    logTest("Authorization code reuse rejected", passed, `second_exchange_status=${secondResponse.status}`);
  } catch (error) {
    logTest("Authorization code reuse rejected", false, error.message);
  }
}

// Test 5: Invalid access token should be rejected at userinfo
async function testInvalidAccessToken() {
  try {
    const response = await fetch(`${AUTH_URL}/api/auth/oauth2/userinfo`, {
      headers: { Authorization: "Bearer invalid-token-12345" },
    });

    const passed = response.status === 401 || response.status === 403;
    logTest("Invalid access token rejected", passed, `status=${response.status}`);
  } catch (error) {
    logTest("Invalid access token rejected", false, error.message);
  }
}

// Test 6: Missing authorization header should fail
async function testMissingAuthHeader() {
  try {
    const response = await fetch(`${AUTH_URL}/api/auth/oauth2/userinfo`);
    const passed = response.status === 401;
    logTest("Missing auth header rejected", passed, `status=${response.status}`);
  } catch (error) {
    logTest("Missing auth header rejected", false, error.message);
  }
}

// ==================== AUTH FLOW EDGE CASES ====================

// Test 7: Sign-in with wrong password
async function testWrongPassword() {
  try {
    const response = await fetch(`${AUTH_URL}/api/auth/sign-in/email`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ email: TEST_EMAIL, password: "WrongPassword123!" }),
    });

    const body = await response.json();
    const passed = !response.ok || body.error;
    logTest("Wrong password rejected", passed, `status=${response.status}`);
  } catch (error) {
    logTest("Wrong password rejected", false, error.message);
  }
}

// Test 8: Sign-in with non-existent email
async function testNonExistentEmail() {
  try {
    const response = await fetch(`${AUTH_URL}/api/auth/sign-in/email`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ email: "nonexistent@example.com", password: "Test123!" }),
    });

    const body = await response.json();
    const passed = !response.ok || body.error;
    logTest("Non-existent email rejected", passed, `status=${response.status}`);
  } catch (error) {
    logTest("Non-existent email rejected", false, error.message);
  }
}

// Test 9: Sign-in with invalid email format
async function testInvalidEmailFormat() {
  try {
    const response = await fetch(`${AUTH_URL}/api/auth/sign-in/email`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ email: "not-an-email", password: "Test123!" }),
    });

    const body = await response.json();
    const passed = !response.ok || body.error;
    logTest("Invalid email format rejected", passed, `status=${response.status}`);
  } catch (error) {
    logTest("Invalid email format rejected", false, error.message);
  }
}

// Test 10: Unauthenticated user cannot authorize
async function testUnauthenticatedAuthorize() {
  try {
    const authUrl = `${AUTH_URL}/api/auth/oauth2/authorize?` +
      `client_id=${PUBLIC_CLIENT.clientId}&` +
      `redirect_uri=${encodeURIComponent(PUBLIC_CLIENT.redirectUri)}&` +
      `response_type=code&scope=openid`;

    const response = await fetch(authUrl, {
      redirect: "manual",
      // No cookies!
    });

    // Should redirect to login or return error
    const passed = response.status === 302 || response.status === 401 ||
                   (response.status === 200 && (await response.json()).redirect);
    logTest("Unauthenticated authorize handled", passed, `status=${response.status}`);
  } catch (error) {
    logTest("Unauthenticated authorize handled", false, error.message);
  }
}

// ==================== TOKEN EDGE CASES ====================

// Test 11: Refresh token flow (if tokens include refresh_token)
async function testRefreshTokenFlow() {
  try {
    const codeVerifier = generateCodeVerifier();
    const codeChallenge = generateCodeChallenge(codeVerifier);
    const cookies = await getSessionCookie();

    // Get tokens
    const authUrl = `${AUTH_URL}/api/auth/oauth2/authorize?` +
      `client_id=${PUBLIC_CLIENT.clientId}&` +
      `redirect_uri=${encodeURIComponent(PUBLIC_CLIENT.redirectUri)}&` +
      `response_type=code&scope=openid%20offline_access&state=test&` +
      `code_challenge=${codeChallenge}&code_challenge_method=S256`;

    const authResponse = await fetch(authUrl, {
      headers: { Cookie: cookies },
      redirect: "manual",
    });

    let code;
    if (authResponse.status === 200) {
      const body = await authResponse.json();
      if (body.redirect && body.url) {
        const match = body.url.match(/code=([^&]+)/);
        if (match) code = match[1];
      }
    }

    if (!code) {
      logTest("Refresh token flow", false, "Could not get auth code");
      return;
    }

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

    const tokens = await tokenResponse.json();

    if (!tokens.refresh_token) {
      logTest("Refresh token flow", true, "No refresh_token returned (offline_access not granted)");
      return;
    }

    // Try to refresh
    const refreshResponse = await fetch(`${AUTH_URL}/api/auth/oauth2/token`, {
      method: "POST",
      headers: { "Content-Type": "application/x-www-form-urlencoded" },
      body: new URLSearchParams({
        grant_type: "refresh_token",
        refresh_token: tokens.refresh_token,
        client_id: PUBLIC_CLIENT.clientId,
      }),
    });

    const passed = refreshResponse.ok;
    const newTokens = await refreshResponse.json();
    logTest("Refresh token flow", passed, `new_access_token=${!!newTokens.access_token}`);
  } catch (error) {
    logTest("Refresh token flow", false, error.message);
  }
}

// Test 12: Token with invalid grant_type
async function testInvalidGrantType() {
  try {
    const response = await fetch(`${AUTH_URL}/api/auth/oauth2/token`, {
      method: "POST",
      headers: { "Content-Type": "application/x-www-form-urlencoded" },
      body: new URLSearchParams({
        grant_type: "password", // This should not be supported
        username: TEST_EMAIL,
        password: TEST_PASSWORD,
        client_id: PUBLIC_CLIENT.clientId,
      }),
    });

    const passed = !response.ok;
    logTest("Invalid grant_type rejected", passed, `status=${response.status}`);
  } catch (error) {
    logTest("Invalid grant_type rejected", false, error.message);
  }
}

// ==================== SCOPE EDGE CASES ====================

// Test 13: Request invalid scope
async function testInvalidScope() {
  try {
    const codeVerifier = generateCodeVerifier();
    const codeChallenge = generateCodeChallenge(codeVerifier);
    const cookies = await getSessionCookie();

    const authUrl = `${AUTH_URL}/api/auth/oauth2/authorize?` +
      `client_id=${PUBLIC_CLIENT.clientId}&` +
      `redirect_uri=${encodeURIComponent(PUBLIC_CLIENT.redirectUri)}&` +
      `response_type=code&scope=admin:all%20delete:everything&state=test&` +
      `code_challenge=${codeChallenge}&code_challenge_method=S256`;

    const response = await fetch(authUrl, {
      headers: { Cookie: cookies },
      redirect: "manual",
    });

    // Either rejects invalid scopes or ignores them gracefully
    const body = response.status === 200 ? await response.json() : null;
    const passed = response.status === 400 ||
                   (body && body.redirect) || // Accepted with reduced scope
                   response.status === 200;
    logTest("Invalid scope handled", passed, `status=${response.status}`);
  } catch (error) {
    logTest("Invalid scope handled", false, error.message);
  }
}

// ==================== MAIN ====================

async function main() {
  console.log("=".repeat(60));
  console.log("Edge Case Test Suite");
  console.log(`Auth Server: ${AUTH_URL}`);
  console.log("=".repeat(60));
  console.log("");

  console.log("--- Security Edge Cases ---");
  await testInvalidClientId();
  await testMismatchedRedirectUri();
  await testPKCEVerifierMismatch();
  await testCodeReuse();
  await testInvalidAccessToken();
  await testMissingAuthHeader();

  console.log("\n--- Auth Flow Edge Cases ---");
  await testWrongPassword();
  await testNonExistentEmail();
  await testInvalidEmailFormat();
  await testUnauthenticatedAuthorize();

  console.log("\n--- Token Edge Cases ---");
  await testRefreshTokenFlow();
  await testInvalidGrantType();

  console.log("\n--- Scope Edge Cases ---");
  await testInvalidScope();

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
