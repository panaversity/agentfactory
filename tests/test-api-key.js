/**
 * API Key M2M Authentication Tests
 * Tests the Better Auth API Key plugin endpoints
 *
 * Prerequisites:
 * 1. Auth server running on port 3001: pnpm dev
 * 2. Test user exists and is admin: admin@robolearn.io / admin@robolearn.io
 *
 * Run: node tests/test-api-key.js
 */

// Configuration
const AUTH_URL = "http://localhost:3001";
const TEST_EMAIL = "admin@robolearn.io";
const TEST_PASSWORD = "admin@robolearn.io";
const ORIGIN_HEADER = AUTH_URL; // Required for CSRF protection

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

async function getSessionCookie() {
  const response = await fetch(`${AUTH_URL}/api/auth/sign-in/email`, {
    method: "POST",
    headers: { "Content-Type": "application/json", Origin: ORIGIN_HEADER },
    body: JSON.stringify({ email: TEST_EMAIL, password: TEST_PASSWORD }),
  });

  if (!response.ok) {
    throw new Error(`Sign-in failed: ${response.status} ${await response.text()}`);
  }

  return response.headers.get("set-cookie");
}

// Store created keys for cleanup
const createdKeyIds = [];

// === Test 1: Create API Key - Success ===
async function testCreateKeySuccess(cookie) {
  try {
    // Use admin-only endpoint (Better Auth's /api-key/create is disabled)
    const response = await fetch(`${AUTH_URL}/api/admin/api-keys/create`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        Cookie: cookie,
        Origin: ORIGIN_HEADER,
      },
      body: JSON.stringify({
        name: "test-key-" + Date.now(),
      }),
    });

    const data = await response.json();
    const success = response.ok && data.key && data.key.startsWith("pana_");

    if (success && data.id) {
      createdKeyIds.push(data.id);
    }

    logTest("Create API Key - Success", success, data.key ? `prefix=${data.key.substring(0, 8)}...` : "no key");
    return data;
  } catch (error) {
    logTest("Create API Key - Success", false, error.message);
    return null;
  }
}

// === Test 2: Create API Key - Fail Without Auth ===
async function testCreateKeyNoAuth() {
  try {
    // Use admin-only endpoint
    const response = await fetch(`${AUTH_URL}/api/admin/api-keys/create`, {
      method: "POST",
      headers: { "Content-Type": "application/json", Origin: ORIGIN_HEADER },
      body: JSON.stringify({ name: "unauthorized-key" }),
    });

    const isUnauthorized = response.status === 401;
    logTest("Create API Key - Fail Without Auth", isUnauthorized, `status=${response.status}`);
  } catch (error) {
    logTest("Create API Key - Fail Without Auth", false, error.message);
  }
}

// === Test 3: Create API Key - Fail With Empty Name ===
async function testCreateKeyEmptyName(cookie) {
  try {
    // Use admin-only endpoint
    const response = await fetch(`${AUTH_URL}/api/admin/api-keys/create`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        Cookie: cookie,
        Origin: ORIGIN_HEADER,
      },
      body: JSON.stringify({ name: "" }),
    });

    // Should fail because requireName: true in plugin config
    const failed = !response.ok || response.status >= 400;
    logTest("Create API Key - Fail With Empty Name", failed, `status=${response.status}`);
  } catch (error) {
    logTest("Create API Key - Fail With Empty Name", true, "threw error as expected");
  }
}

// === Test 4: API Key Created with Correct Format ===
// Better Auth verifies API keys via headers on custom protected endpoints
// The Better Auth admin endpoints (list, create, etc.) require session auth, not API key
async function testVerifyKeySuccess(apiKey) {
  if (!apiKey) {
    logTest("API Key Format Valid", false, "no key to test");
    return;
  }

  // Verify key has correct format: prefix + random string
  const hasCorrectPrefix = apiKey.startsWith("pana_");
  const hasCorrectLength = apiKey.length > 20; // Should be prefix + sufficient random chars
  const success = hasCorrectPrefix && hasCorrectLength;

  logTest("API Key Format Valid", success, `prefix=${hasCorrectPrefix}, length=${apiKey.length}`);
}

// === Test 4b: Verify API Key via Custom Endpoint (for M2M services) ===
async function testVerifyKeyEndpoint(apiKey) {
  if (!apiKey) {
    logTest("Verify Key Endpoint - Valid Key", false, "no key to test");
    return;
  }

  try {
    const response = await fetch(`${AUTH_URL}/api/api-key/verify`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ key: apiKey }),
    });

    const data = await response.json();
    const success = response.ok && data.valid === true && data.key?.id;
    logTest("Verify Key Endpoint - Valid Key", success, `valid=${data.valid}, userId=${data.key?.userId?.substring(0, 8)}...`);
  } catch (error) {
    logTest("Verify Key Endpoint - Valid Key", false, error.message);
  }
}

// === Test 4c: Verify Invalid Key via Custom Endpoint ===
async function testVerifyInvalidKeyEndpoint() {
  try {
    const response = await fetch(`${AUTH_URL}/api/api-key/verify`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ key: "pana_invalid_key_12345" }),
    });

    const data = await response.json();
    const success = response.status === 401 && data.valid === false && data.error;
    logTest("Verify Key Endpoint - Invalid Key", success, `valid=${data.valid}, error=${data.error?.code}`);
  } catch (error) {
    logTest("Verify Key Endpoint - Invalid Key", false, error.message);
  }
}

// === Test 4d: Verify Missing Key via Custom Endpoint ===
async function testVerifyMissingKeyEndpoint() {
  try {
    const response = await fetch(`${AUTH_URL}/api/api-key/verify`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({}),
    });

    const data = await response.json();
    const success = response.status === 400 && data.valid === false && data.error?.code === "MISSING_KEY";
    logTest("Verify Key Endpoint - Missing Key", success, `valid=${data.valid}, error=${data.error?.code}`);
  } catch (error) {
    logTest("Verify Key Endpoint - Missing Key", false, error.message);
  }
}

// === Test 5: API Key List Returns Created Keys ===
async function testVerifyKeyInvalid(cookie) {
  try {
    // This test just verifies we can list keys (should have at least 1 from previous test)
    // Use admin-only endpoint
    const response = await fetch(`${AUTH_URL}/api/admin/api-keys/list`, {
      method: "GET",
      headers: { Cookie: cookie, Origin: ORIGIN_HEADER },
    });

    const data = await response.json();
    const success = response.ok && Array.isArray(data) && data.length >= 1;
    logTest("API Key List Contains Keys", success, `count=${data?.length || 0}`);
  } catch (error) {
    logTest("API Key List Contains Keys", false, error.message);
  }
}

// === Test 6: List API Keys ===
async function testListKeys(cookie) {
  try {
    // Use admin-only endpoint
    const response = await fetch(`${AUTH_URL}/api/admin/api-keys/list`, {
      method: "GET",
      headers: { Cookie: cookie, Origin: ORIGIN_HEADER },
    });

    const data = await response.json();
    const success = response.ok && Array.isArray(data);
    logTest("List API Keys", success, `count=${data?.length || 0}`);
    return data;
  } catch (error) {
    logTest("List API Keys", false, error.message);
    return [];
  }
}

// === Test 7: Update API Key (Revoke) ===
async function testRevokeKey(cookie, keyId) {
  if (!keyId) {
    logTest("Revoke API Key", false, "no key to revoke");
    return;
  }

  try {
    // Use admin-only endpoint
    const response = await fetch(`${AUTH_URL}/api/admin/api-keys/update`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        Cookie: cookie,
        Origin: ORIGIN_HEADER,
      },
      body: JSON.stringify({
        keyId,
        enabled: false,
      }),
    });

    const success = response.ok;
    logTest("Revoke API Key", success, `status=${response.status}`);
  } catch (error) {
    logTest("Revoke API Key", false, error.message);
  }
}

// === Test 8: Verify Revoked Key is Disabled in List ===
async function testVerifyRevokedKey(cookie, keyId) {
  if (!keyId) {
    logTest("Revoked Key Shows Disabled", false, "no key to test");
    return;
  }

  try {
    // List keys and verify the revoked key shows as disabled
    // Use admin-only endpoint
    const response = await fetch(`${AUTH_URL}/api/admin/api-keys/list`, {
      method: "GET",
      headers: { Cookie: cookie, Origin: ORIGIN_HEADER },
    });

    const data = await response.json();
    const revokedKey = data.find((k) => k.id === keyId);
    const success = revokedKey && revokedKey.enabled === false;
    logTest("Revoked Key Shows Disabled", success, `enabled=${revokedKey?.enabled}`);
  } catch (error) {
    logTest("Revoked Key Shows Disabled", false, error.message);
  }
}

// === Test 9: Delete API Key ===
async function testDeleteKey(cookie, keyId) {
  if (!keyId) {
    logTest("Delete API Key", false, "no key to delete");
    return;
  }

  try {
    // Use admin-only endpoint
    const response = await fetch(`${AUTH_URL}/api/admin/api-keys/delete`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        Cookie: cookie,
        Origin: ORIGIN_HEADER,
      },
      body: JSON.stringify({ keyId }),
    });

    const success = response.ok;
    logTest("Delete API Key", success, `status=${response.status}`);
  } catch (error) {
    logTest("Delete API Key", false, error.message);
  }
}

// === Test 10: Create Key With Expiration ===
async function testCreateKeyWithExpiration(cookie) {
  try {
    // Use admin-only endpoint
    const response = await fetch(`${AUTH_URL}/api/admin/api-keys/create`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        Cookie: cookie,
        Origin: ORIGIN_HEADER,
      },
      body: JSON.stringify({
        name: "expiring-key-" + Date.now(),
        expiresIn: 30 * 24 * 60 * 60, // 30 days in seconds (2592000)
      }),
    });

    const data = await response.json();
    const success = response.ok && data.key && data.expiresAt;

    if (success && data.id) {
      createdKeyIds.push(data.id);
    }

    logTest("Create Key With Expiration", success, data.expiresAt ? `expires=${data.expiresAt}` : "no expiration");
    return data;
  } catch (error) {
    logTest("Create Key With Expiration", false, error.message);
    return null;
  }
}

// Cleanup function
async function cleanup(cookie) {
  console.log("\n--- Cleanup ---");
  for (const keyId of createdKeyIds) {
    try {
      // Use admin-only endpoint
      await fetch(`${AUTH_URL}/api/admin/api-keys/delete`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          Cookie: cookie,
          Origin: ORIGIN_HEADER,
        },
        body: JSON.stringify({ keyId }),
      });
      console.log(`Cleaned up key: ${keyId}`);
    } catch (e) {
      console.log(`Failed to cleanup key ${keyId}: ${e.message}`);
    }
  }
}

// Main test runner
async function runTests() {
  console.log("=== API Key M2M Authentication Tests ===\n");
  console.log(`Testing against: ${AUTH_URL}`);
  console.log(`Test user: ${TEST_EMAIL}\n`);

  try {
    // Get session cookie
    console.log("--- Setup: Getting session cookie ---");
    const cookie = await getSessionCookie();
    if (!cookie) {
      console.error("Failed to get session cookie. Ensure the server is running and test user exists.");
      process.exit(1);
    }
    console.log("Session obtained successfully\n");

    // Run tests
    console.log("--- Running API Key Tests ---\n");

    // Test 1-3: Create operations
    const createdKey = await testCreateKeySuccess(cookie);
    await testCreateKeyNoAuth();
    await testCreateKeyEmptyName(cookie);

    // Test 4: Verify operations (format check)
    await testVerifyKeySuccess(createdKey?.key);

    // Test 4b-4d: Custom verify endpoint for M2M services
    await testVerifyKeyEndpoint(createdKey?.key);
    await testVerifyInvalidKeyEndpoint();
    await testVerifyMissingKeyEndpoint();

    // Test 5: List keys
    await testVerifyKeyInvalid(cookie);

    // Test 6: List
    await testListKeys(cookie);

    // Test 7-8: Revoke and verify revoked
    const keyToRevoke = await testCreateKeySuccess(cookie);
    await testRevokeKey(cookie, keyToRevoke?.id);
    await testVerifyRevokedKey(cookie, keyToRevoke?.id);

    // Test 9: Delete
    const keyToDelete = await testCreateKeySuccess(cookie);
    await testDeleteKey(cookie, keyToDelete?.id);
    // Remove from cleanup list since already deleted
    const deleteIdx = createdKeyIds.indexOf(keyToDelete?.id);
    if (deleteIdx > -1) createdKeyIds.splice(deleteIdx, 1);

    // Test 10: Expiration
    await testCreateKeyWithExpiration(cookie);

    // Cleanup
    await cleanup(cookie);

    // Summary
    console.log("\n=== Test Summary ===");
    console.log(`Passed: ${results.passed}`);
    console.log(`Failed: ${results.failed}`);
    console.log(`Total: ${results.passed + results.failed}`);

    if (results.failed > 0) {
      console.log("\nFailed tests:");
      results.tests.filter((t) => !t.passed).forEach((t) => console.log(`  - ${t.name}: ${t.details}`));
      process.exit(1);
    } else {
      console.log("\nAll tests passed!");
      process.exit(0);
    }
  } catch (error) {
    console.error("\nTest runner failed:", error);
    process.exit(1);
  }
}

runTests();
