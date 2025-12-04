/**
 * OAuth Client Edit Functionality Tests
 *
 * Tests the PATCH endpoint for updating OAuth client redirect URLs
 * Run with: node tests/test-client-edit.js
 *
 * Prerequisites:
 * 1. Auth server running at http://localhost:3001
 * 2. Admin user logged in (get session cookie from browser)
 * 3. At least one non-trusted OAuth client in database
 */

const BASE_URL = process.env.BETTER_AUTH_URL || "http://localhost:3001";
const ADMIN_EMAIL = process.env.ADMIN_EMAIL || "admin@robolearn.io";
const ADMIN_PASSWORD = process.env.ADMIN_PASSWORD || "admin@robolearn.io";

// Admin session cookie (will be auto-populated via login)
let ADMIN_COOKIE = process.env.ADMIN_SESSION_COOKIE || "";

/**
 * Auto-login helper - Signs in as admin and gets session cookie
 */
async function autoLogin() {
  console.log("🔐 Auto-login: Signing in as admin...");

  try {
    const response = await fetch(`${BASE_URL}/api/auth/sign-in/email`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        email: ADMIN_EMAIL,
        password: ADMIN_PASSWORD
      }),
    });

    if (!response.ok) {
      const error = await response.text();
      throw new Error(`Sign-in failed (${response.status}): ${error}`);
    }

    const setCookie = response.headers.get("set-cookie");
    if (!setCookie) {
      throw new Error("No session cookie received from server");
    }

    // Extract session token from Set-Cookie header
    ADMIN_COOKIE = setCookie.split(";")[0]; // Get just the token part
    console.log("✅ Auto-login successful\n");
    return true;
  } catch (error) {
    console.error("❌ Auto-login failed:", error.message);
    console.error("\nFallback: Set ADMIN_SESSION_COOKIE manually:");
    console.error('   export ADMIN_SESSION_COOKIE="better-auth.session_token=xxx"\n');
    return false;
  }
}

console.log("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
console.log("  OAuth Client Edit Functionality Tests");
console.log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

let testsPassed = 0;
let testsFailed = 0;

// Test helper
async function test(name, fn) {
  try {
    await fn();
    console.log(`✅ ${name}`);
    testsPassed++;
  } catch (error) {
    console.error(`❌ ${name}`);
    console.error(`   ${error.message}`);
    if (error.details) {
      console.error(`   Details: ${JSON.stringify(error.details, null, 2)}`);
    }
    testsFailed++;
  }
}

// Assertion helper
function assert(condition, message) {
  if (!condition) {
    const error = new Error(message || "Assertion failed");
    error.details = { condition };
    throw error;
  }
}

// HTTP helper
async function apiRequest(path, options = {}) {
  const response = await fetch(`${BASE_URL}${path}`, {
    ...options,
    headers: {
      "Content-Type": "application/json",
      "Cookie": ADMIN_COOKIE,
      ...options.headers,
    },
  });

  const text = await response.text();
  let data;
  try {
    data = JSON.parse(text);
  } catch {
    data = { text };
  }

  return { response, data, status: response.status };
}

// Get first non-trusted client for testing
async function getTestClient() {
  const { data, status } = await apiRequest("/api/admin/clients");
  assert(status === 200, `Failed to fetch clients: ${status}`);

  const client = data.clients.find(c => !c.isTrusted);
  assert(client, "No non-trusted client found for testing. Create one first via Admin UI.");

  return client;
}

// Run tests
async function runTests() {
  console.log("📊 Setup: Fetching test client...\n");
  const testClient = await getTestClient();
  console.log(`   Using client: ${testClient.name} (${testClient.clientId})\n`);

  // Test 1: Update redirect URLs successfully
  await test("1. Update redirect URLs - Success", async () => {
    const newUrls = [
      "http://localhost:3000/auth/callback",
      "http://localhost:4000/test/callback",
      "https://production.example.com/auth/callback",
    ];

    const { status, data } = await apiRequest("/api/admin/clients", {
      method: "PATCH",
      body: JSON.stringify({
        clientId: testClient.clientId,
        redirectUrls: newUrls,
      }),
    });

    assert(status === 200, `Expected 200, got ${status}`);
    assert(data.success === true, "Expected success: true");

    // Verify the update
    const { data: clients } = await apiRequest("/api/admin/clients");
    const updated = clients.clients.find(c => c.clientId === testClient.clientId);
    assert(
      JSON.stringify(updated.redirectUrls.sort()) === JSON.stringify(newUrls.sort()),
      "Redirect URLs not updated correctly"
    );
  });

  // Test 2: Restore original URLs (cleanup)
  await test("2. Restore original URLs", async () => {
    const { status } = await apiRequest("/api/admin/clients", {
      method: "PATCH",
      body: JSON.stringify({
        clientId: testClient.clientId,
        redirectUrls: testClient.redirectUrls,
      }),
    });

    assert(status === 200, `Expected 200, got ${status}`);
  });

  // Test 3: Empty redirect URLs - Should fail
  await test("3. Empty redirect URLs - Should fail (400)", async () => {
    const { status, data } = await apiRequest("/api/admin/clients", {
      method: "PATCH",
      body: JSON.stringify({
        clientId: testClient.clientId,
        redirectUrls: [],
      }),
    });

    assert(status === 400, `Expected 400, got ${status}`);
    assert(
      data.error.includes("At least one redirect URL"),
      "Expected error about redirect URLs"
    );
  });

  // Test 4: Missing clientId - Should fail
  await test("4. Missing clientId - Should fail (400)", async () => {
    const { status, data } = await apiRequest("/api/admin/clients", {
      method: "PATCH",
      body: JSON.stringify({
        redirectUrls: ["http://localhost:3000/callback"],
      }),
    });

    assert(status === 400, `Expected 400, got ${status}`);
    assert(data.error.includes("Client ID required"), "Expected error about client ID");
  });

  // Test 5: Non-existent clientId - Should succeed (no rows updated)
  await test("5. Non-existent clientId - Should succeed (no error)", async () => {
    const { status, data } = await apiRequest("/api/admin/clients", {
      method: "PATCH",
      body: JSON.stringify({
        clientId: "non-existent-client-12345",
        redirectUrls: ["http://localhost:3000/callback"],
      }),
    });

    // Should succeed but nothing updated
    assert(status === 200, `Expected 200, got ${status}`);
  });

  // Test 6: Editing trusted client - Should fail
  await test("6. Editing trusted client - Should fail (400)", async () => {
    const { status, data } = await apiRequest("/api/admin/clients", {
      method: "PATCH",
      body: JSON.stringify({
        clientId: "robolearn-public-client",
        redirectUrls: ["http://localhost:3000/callback"],
      }),
    });

    assert(status === 400, `Expected 400, got ${status}`);
    assert(
      data.error.includes("Cannot edit pre-configured trusted client"),
      "Expected error about trusted client"
    );
  });

  // Test 7: Unauthorized request (no cookie) - Should fail
  await test("7. Unauthorized request - Should fail (401)", async () => {
    const response = await fetch(`${BASE_URL}/api/admin/clients`, {
      method: "PATCH",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        clientId: testClient.clientId,
        redirectUrls: ["http://localhost:3000/callback"],
      }),
    });

    assert(response.status === 401, `Expected 401, got ${response.status}`);
  });

  // Test 8: Invalid redirect URLs format (not array) - Should fail
  await test("8. Invalid redirect URLs format - Should fail (400)", async () => {
    const { status, data } = await apiRequest("/api/admin/clients", {
      method: "PATCH",
      body: JSON.stringify({
        clientId: testClient.clientId,
        redirectUrls: "not-an-array",
      }),
    });

    assert(status === 400, `Expected 400, got ${status}`);
  });

  // Test 9: Single redirect URL - Should succeed
  await test("9. Single redirect URL - Should succeed", async () => {
    const newUrls = ["https://single-url.example.com/callback"];

    const { status, data } = await apiRequest("/api/admin/clients", {
      method: "PATCH",
      body: JSON.stringify({
        clientId: testClient.clientId,
        redirectUrls: newUrls,
      }),
    });

    assert(status === 200, `Expected 200, got ${status}`);
    assert(data.success === true, "Expected success: true");

    // Restore original
    await apiRequest("/api/admin/clients", {
      method: "PATCH",
      body: JSON.stringify({
        clientId: testClient.clientId,
        redirectUrls: testClient.redirectUrls,
      }),
    });
  });

  // Test 10: Very long URL - Should succeed
  await test("10. Very long URL - Should succeed", async () => {
    const longUrl = `https://very-long-domain-name-for-testing-purposes.example.com/auth/callback${"?param=value".repeat(20)}`;
    const newUrls = [longUrl];

    const { status } = await apiRequest("/api/admin/clients", {
      method: "PATCH",
      body: JSON.stringify({
        clientId: testClient.clientId,
        redirectUrls: newUrls,
      }),
    });

    assert(status === 200, `Expected 200, got ${status}`);

    // Restore original
    await apiRequest("/api/admin/clients", {
      method: "PATCH",
      body: JSON.stringify({
        clientId: testClient.clientId,
        redirectUrls: testClient.redirectUrls,
      }),
    });
  });

  // Test 11: Multiple localhost URLs with different ports
  await test("11. Multiple localhost URLs - Should succeed", async () => {
    const newUrls = [
      "http://localhost:3000/callback",
      "http://localhost:4000/callback",
      "http://localhost:5000/callback",
      "http://localhost:8080/callback",
    ];

    const { status } = await apiRequest("/api/admin/clients", {
      method: "PATCH",
      body: JSON.stringify({
        clientId: testClient.clientId,
        redirectUrls: newUrls,
      }),
    });

    assert(status === 200, `Expected 200, got ${status}`);

    // Restore original
    await apiRequest("/api/admin/clients", {
      method: "PATCH",
      body: JSON.stringify({
        clientId: testClient.clientId,
        redirectUrls: testClient.redirectUrls,
      }),
    });
  });

  // Test 12: Idempotency - Same update twice
  await test("12. Idempotency - Same update twice", async () => {
    const newUrls = ["https://idempotent.example.com/callback"];

    // First update
    const { status: status1 } = await apiRequest("/api/admin/clients", {
      method: "PATCH",
      body: JSON.stringify({
        clientId: testClient.clientId,
        redirectUrls: newUrls,
      }),
    });

    assert(status1 === 200, `First update failed: ${status1}`);

    // Second update (same data)
    const { status: status2 } = await apiRequest("/api/admin/clients", {
      method: "PATCH",
      body: JSON.stringify({
        clientId: testClient.clientId,
        redirectUrls: newUrls,
      }),
    });

    assert(status2 === 200, `Second update failed: ${status2}`);

    // Restore original
    await apiRequest("/api/admin/clients", {
      method: "PATCH",
      body: JSON.stringify({
        clientId: testClient.clientId,
        redirectUrls: testClient.redirectUrls,
      }),
    });
  });

  // Results
  console.log("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  console.log("  Test Results");
  console.log("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
  console.log(`✅ Passed: ${testsPassed}`);
  console.log(`❌ Failed: ${testsFailed}`);
  console.log(`📊 Total:  ${testsPassed + testsFailed}\n`);

  if (testsFailed === 0) {
    console.log("🎉 All tests passed!\n");
    process.exit(0);
  } else {
    console.log("⚠️  Some tests failed. Please review the errors above.\n");
    process.exit(1);
  }
}

// Main execution with auto-login
(async () => {
  // Try auto-login if cookie not provided
  if (!ADMIN_COOKIE) {
    const loginSuccess = await autoLogin();
    if (!loginSuccess) {
      console.error("❌ Cannot proceed without admin session");
      process.exit(1);
    }
  } else {
    console.log("✅ Using provided ADMIN_SESSION_COOKIE\n");
  }

  // Run the tests
  await runTests();
})().catch((error) => {
  console.error("\n❌ Test suite failed:");
  console.error(error);
  process.exit(1);
});
