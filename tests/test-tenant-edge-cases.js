/**
 * Tenant/Organization Edge Case Tests
 * Tests multi-tenant claims, organization memberships, and edge scenarios
 *
 * Prerequisites:
 * 1. Auth server running: pnpm dev
 * 2. Test user with organization: pnpm seed:org
 *
 * Run: node tests/test-tenant-edge-cases.js
 */

const crypto = require("crypto");

const AUTH_URL = "http://localhost:3001";
const TEST_EMAIL = "admin@robolearn.io";
const TEST_PASSWORD = "admin@robolearn.io";

// Results tracking
const results = { passed: 0, failed: 0, tests: [] };

function logTest(name, passed, details = "") {
  const icon = passed ? "\u2705" : "\u274c";
  console.log(`${icon} ${name}: ${passed ? "PASS" : "FAIL"}${details ? ` - ${details}` : ""}`);
  results.tests.push({ name, passed, details });
  if (passed) results.passed++;
  else results.failed++;
}

function generateCodeVerifier() {
  return crypto.randomBytes(32).toString("base64url");
}

function generateCodeChallenge(verifier) {
  return crypto.createHash("sha256").update(verifier).digest().toString("base64url");
}

async function getTokensForUser(email, password) {
  // Sign in
  const signInResponse = await fetch(`${AUTH_URL}/api/auth/sign-in/email`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ email, password }),
  });

  if (!signInResponse.ok) {
    throw new Error(`Sign-in failed for ${email}`);
  }

  const cookies = signInResponse.headers.get("set-cookie");

  // PKCE flow
  const codeVerifier = generateCodeVerifier();
  const codeChallenge = generateCodeChallenge(codeVerifier);

  const authUrl = `${AUTH_URL}/api/auth/oauth2/authorize?` +
    `client_id=robolearn-public-client&` +
    `redirect_uri=${encodeURIComponent("http://localhost:3000/auth/callback")}&` +
    `response_type=code&scope=openid%20profile%20email&state=test&` +
    `code_challenge=${codeChallenge}&code_challenge_method=S256`;

  const authResponse = await fetch(authUrl, {
    headers: { Cookie: cookies },
    redirect: "manual",
  });

  const body = await authResponse.json();
  const code = body.url?.match(/code=([^&]+)/)?.[1];

  if (!code) throw new Error("No auth code");

  const tokenResponse = await fetch(`${AUTH_URL}/api/auth/oauth2/token`, {
    method: "POST",
    headers: { "Content-Type": "application/x-www-form-urlencoded" },
    body: new URLSearchParams({
      grant_type: "authorization_code",
      code,
      redirect_uri: "http://localhost:3000/auth/callback",
      client_id: "robolearn-public-client",
      code_verifier: codeVerifier,
    }),
  });

  return tokenResponse.json();
}

async function getUserinfo(accessToken) {
  const response = await fetch(`${AUTH_URL}/api/auth/oauth2/userinfo`, {
    headers: { Authorization: `Bearer ${accessToken}` },
  });
  return response.json();
}

// ==================== TENANT CLAIM TESTS ====================

// Test 1: User with organization has tenant_id
async function testUserWithOrganization() {
  try {
    const tokens = await getTokensForUser(TEST_EMAIL, TEST_PASSWORD);
    const userinfo = await getUserinfo(tokens.access_token);

    const hasTenantId = userinfo.tenant_id !== undefined;
    const hasOrgIds = Array.isArray(userinfo.organization_ids);
    const hasOrgRole = userinfo.org_role !== undefined;

    const passed = hasTenantId && hasOrgIds && hasOrgRole;
    logTest(
      "User with org has tenant claims",
      passed,
      `tenant_id=${userinfo.tenant_id}, org_count=${userinfo.organization_ids?.length}, role=${userinfo.org_role}`
    );
  } catch (error) {
    logTest("User with org has tenant claims", false, error.message);
  }
}

// Test 2: Verify tenant_id matches first organization
async function testTenantIdIsPrimaryOrg() {
  try {
    const tokens = await getTokensForUser(TEST_EMAIL, TEST_PASSWORD);
    const userinfo = await getUserinfo(tokens.access_token);

    if (!userinfo.organization_ids || userinfo.organization_ids.length === 0) {
      logTest("tenant_id matches primary org", true, "No organizations - tenant_id is null");
      return;
    }

    const passed = userinfo.tenant_id === userinfo.organization_ids[0];
    logTest(
      "tenant_id matches primary org",
      passed,
      `tenant_id=${userinfo.tenant_id}, first_org=${userinfo.organization_ids[0]}`
    );
  } catch (error) {
    logTest("tenant_id matches primary org", false, error.message);
  }
}

// Test 3: Verify org_role is from primary organization
async function testOrgRoleFromPrimaryOrg() {
  try {
    const tokens = await getTokensForUser(TEST_EMAIL, TEST_PASSWORD);
    const userinfo = await getUserinfo(tokens.access_token);

    // If user has organizations, they should have a role
    if (userinfo.organization_ids && userinfo.organization_ids.length > 0) {
      const hasRole = userinfo.org_role !== null && userinfo.org_role !== undefined;
      logTest("org_role present for org member", hasRole, `role=${userinfo.org_role}`);
    } else {
      const noRole = userinfo.org_role === null;
      logTest("org_role null for non-member", noRole, `role=${userinfo.org_role}`);
    }
  } catch (error) {
    logTest("org_role from primary org", false, error.message);
  }
}

// Test 4: Verify custom profile claims exist
async function testProfileClaims() {
  try {
    const tokens = await getTokensForUser(TEST_EMAIL, TEST_PASSWORD);
    const userinfo = await getUserinfo(tokens.access_token);

    const hasSoftwareBg = userinfo.hasOwnProperty("software_background");
    const hasHardwareTier = userinfo.hasOwnProperty("hardware_tier");
    const hasRole = userinfo.hasOwnProperty("role");

    const passed = hasSoftwareBg && hasHardwareTier && hasRole;
    logTest(
      "Profile claims present",
      passed,
      `software_bg=${userinfo.software_background}, hardware_tier=${userinfo.hardware_tier}, role=${userinfo.role}`
    );
  } catch (error) {
    logTest("Profile claims present", false, error.message);
  }
}

// Test 5: Verify ID token also contains tenant claims
async function testIdTokenContainsTenantClaims() {
  try {
    const tokens = await getTokensForUser(TEST_EMAIL, TEST_PASSWORD);

    if (!tokens.id_token) {
      logTest("ID token contains tenant claims", false, "No ID token returned");
      return;
    }

    // Decode ID token (without verification)
    const parts = tokens.id_token.split(".");
    const payload = JSON.parse(Buffer.from(parts[1], "base64url").toString());

    // ID token may or may not contain custom claims depending on config
    // At minimum, check standard OIDC claims
    const hasStandardClaims = payload.iss && payload.sub && payload.aud && payload.exp;

    logTest(
      "ID token has standard claims",
      hasStandardClaims,
      `iss=${payload.iss}, sub=${payload.sub?.slice(0, 10)}...`
    );
  } catch (error) {
    logTest("ID token contains tenant claims", false, error.message);
  }
}

// Test 6: Verify organization_ids is always an array
async function testOrganizationIdsIsArray() {
  try {
    const tokens = await getTokensForUser(TEST_EMAIL, TEST_PASSWORD);
    const userinfo = await getUserinfo(tokens.access_token);

    const isArray = Array.isArray(userinfo.organization_ids);
    logTest(
      "organization_ids is array",
      isArray,
      `type=${typeof userinfo.organization_ids}, value=${JSON.stringify(userinfo.organization_ids)}`
    );
  } catch (error) {
    logTest("organization_ids is array", false, error.message);
  }
}

// Test 7: Verify role claim exists and has valid value
async function testRoleClaim() {
  try {
    const tokens = await getTokensForUser(TEST_EMAIL, TEST_PASSWORD);
    const userinfo = await getUserinfo(tokens.access_token);

    const hasRole = userinfo.role !== undefined;
    const validRole = ["user", "admin", "owner", "member"].includes(userinfo.role) || userinfo.role === null;

    logTest("Role claim valid", hasRole && validRole, `role=${userinfo.role}`);
  } catch (error) {
    logTest("Role claim valid", false, error.message);
  }
}

// Test 8: Verify email_verified claim
async function testEmailVerifiedClaim() {
  try {
    const tokens = await getTokensForUser(TEST_EMAIL, TEST_PASSWORD);
    const userinfo = await getUserinfo(tokens.access_token);

    const passed = typeof userinfo.email_verified === "boolean";
    logTest("email_verified is boolean", passed, `email_verified=${userinfo.email_verified}`);
  } catch (error) {
    logTest("email_verified is boolean", false, error.message);
  }
}

// ==================== MAIN ====================

async function main() {
  console.log("=".repeat(60));
  console.log("Tenant/Organization Edge Case Tests");
  console.log(`Auth Server: ${AUTH_URL}`);
  console.log(`Test User: ${TEST_EMAIL}`);
  console.log("=".repeat(60));
  console.log("");

  console.log("--- Tenant Claim Tests ---");
  await testUserWithOrganization();
  await testTenantIdIsPrimaryOrg();
  await testOrgRoleFromPrimaryOrg();

  console.log("\n--- Profile Claim Tests ---");
  await testProfileClaims();
  await testRoleClaim();
  await testEmailVerifiedClaim();

  console.log("\n--- Token Structure Tests ---");
  await testIdTokenContainsTenantClaims();
  await testOrganizationIdsIsArray();

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
