#!/usr/bin/env npx tsx
/**
 * OIDC Endpoint Tests
 *
 * Tests for:
 * - /.well-known/openid-configuration (OIDC Discovery proxy)
 * - /api/auth/oauth2/endsession (RP-Initiated Logout)
 *
 * Security coverage:
 * - Open redirect protection (CWE-601)
 * - Dynamic discovery (no hardcoded values)
 * - Cookie prefix consistency
 *
 * Run with: pnpm tsx tests/test-oidc-endpoints.ts
 */

const BASE_URL = process.env.BETTER_AUTH_URL || "http://localhost:3001";

interface TestResult {
  test: string;
  status: "PASS" | "FAIL" | "SKIP";
  notes?: string;
  error?: string;
}

const results: TestResult[] = [];

function logTest(
  test: string,
  status: "PASS" | "FAIL" | "SKIP",
  notes?: string,
  error?: string
) {
  results.push({ test, status, notes, error });
  const emoji = status === "PASS" ? "✅" : status === "FAIL" ? "❌" : "⏭️";
  console.log(`${emoji} ${test}`);
  if (notes) console.log(`   📝 ${notes}`);
  if (error) console.log(`   ⚠️  ${error}`);
}

// =============================================================================
// Test Suite 1: OIDC Discovery Endpoint
// =============================================================================

async function testDiscoveryEndpoint() {
  console.log("\n🔍 Test Suite 1: OIDC Discovery Endpoint\n");

  // Test 1.1: Standard path returns valid response
  try {
    const response = await fetch(`${BASE_URL}/.well-known/openid-configuration`);

    if (response.ok) {
      logTest("Discovery: Standard path responds", "PASS", `Status: ${response.status}`);
    } else {
      logTest(
        "Discovery: Standard path responds",
        "FAIL",
        undefined,
        `Status: ${response.status}`
      );
      return null;
    }

    const discovery = await response.json();

    // Test 1.2: Required fields present
    const requiredFields = [
      "issuer",
      "authorization_endpoint",
      "token_endpoint",
      "userinfo_endpoint",
      "jwks_uri",
    ];

    for (const field of requiredFields) {
      if (discovery[field]) {
        logTest(`Discovery: ${field} present`, "PASS", discovery[field]);
      } else {
        logTest(`Discovery: ${field} present`, "FAIL", undefined, "Missing field");
      }
    }

    // Test 1.3: Issuer matches BASE_URL (dynamic, not hardcoded)
    if (discovery.issuer === BASE_URL) {
      logTest("Discovery: Issuer matches base URL", "PASS", discovery.issuer);
    } else {
      logTest(
        "Discovery: Issuer matches base URL",
        "FAIL",
        undefined,
        `Expected ${BASE_URL}, got ${discovery.issuer}`
      );
    }

    // Test 1.4: end_session_endpoint present (required for global logout)
    if (discovery.end_session_endpoint) {
      logTest(
        "Discovery: end_session_endpoint present",
        "PASS",
        discovery.end_session_endpoint
      );
    } else {
      logTest(
        "Discovery: end_session_endpoint present",
        "SKIP",
        undefined,
        "Not exposed in discovery (custom endpoint at /api/auth/oauth2/endsession)"
      );
    }

    // Test 1.5: PKCE support
    if (discovery.code_challenge_methods_supported?.includes("S256")) {
      logTest("Discovery: PKCE S256 supported", "PASS");
    } else {
      logTest("Discovery: PKCE S256 supported", "FAIL");
    }

    // Test 1.6: Cache-Control header present
    const cacheControl = response.headers.get("cache-control");
    if (cacheControl?.includes("max-age")) {
      logTest("Discovery: Cache-Control header present", "PASS", cacheControl);
    } else {
      logTest(
        "Discovery: Cache-Control header present",
        "SKIP",
        undefined,
        "No caching header"
      );
    }

    return discovery;
  } catch (error) {
    logTest("Discovery: Fetch endpoint", "FAIL", undefined, String(error));
    return null;
  }
}

// =============================================================================
// Test Suite 2: End Session Endpoint
// =============================================================================

async function testEndSessionEndpoint() {
  console.log("\n🔐 Test Suite 2: End Session Endpoint\n");

  const endpoint = `${BASE_URL}/api/auth/oauth2/endsession`;

  // Test 2.1: GET request without parameters returns success
  try {
    const response = await fetch(endpoint);
    const data = await response.json();

    if (response.ok && data.success === true) {
      logTest(
        "EndSession: GET without params succeeds",
        "PASS",
        `Message: ${data.message}`
      );
    } else if (response.ok) {
      logTest(
        "EndSession: GET without params succeeds",
        "PASS",
        `Status: ${response.status}`
      );
    } else {
      logTest(
        "EndSession: GET without params succeeds",
        "FAIL",
        undefined,
        `Status: ${response.status}`
      );
    }
  } catch (error) {
    logTest("EndSession: GET without params", "FAIL", undefined, String(error));
  }

  // Test 2.2: POST request works
  try {
    const response = await fetch(endpoint, { method: "POST" });
    const data = await response.json();

    if (response.ok && data.success === true) {
      logTest("EndSession: POST method works", "PASS");
    } else if (response.ok) {
      logTest("EndSession: POST method works", "PASS", `Status: ${response.status}`);
    } else {
      logTest(
        "EndSession: POST method works",
        "FAIL",
        undefined,
        `Status: ${response.status}`
      );
    }
  } catch (error) {
    logTest("EndSession: POST method works", "FAIL", undefined, String(error));
  }

  // Test 2.3: Valid post_logout_redirect_uri redirects
  // Using localhost:3000 which should be in trusted clients
  try {
    const validRedirectUri = "http://localhost:3000/logged-out";
    const response = await fetch(
      `${endpoint}?post_logout_redirect_uri=${encodeURIComponent(validRedirectUri)}`,
      { redirect: "manual" }
    );

    if (response.status === 302) {
      const location = response.headers.get("location");
      if (location?.startsWith(validRedirectUri)) {
        logTest(
          "EndSession: Valid redirect URI redirects",
          "PASS",
          `Redirects to: ${location}`
        );
      } else {
        logTest(
          "EndSession: Valid redirect URI redirects",
          "FAIL",
          undefined,
          `Unexpected location: ${location}`
        );
      }
    } else {
      logTest(
        "EndSession: Valid redirect URI redirects",
        "FAIL",
        undefined,
        `Expected 302, got ${response.status}`
      );
    }
  } catch (error) {
    logTest("EndSession: Valid redirect URI", "FAIL", undefined, String(error));
  }

  // Test 2.4: State parameter is preserved in redirect
  try {
    const validRedirectUri = "http://localhost:3000/logged-out";
    const state = "test-state-12345";
    const response = await fetch(
      `${endpoint}?post_logout_redirect_uri=${encodeURIComponent(validRedirectUri)}&state=${state}`,
      { redirect: "manual" }
    );

    if (response.status === 302) {
      const location = response.headers.get("location");
      if (location?.includes(`state=${state}`)) {
        logTest("EndSession: State parameter preserved", "PASS", `Location includes state`);
      } else {
        logTest(
          "EndSession: State parameter preserved",
          "FAIL",
          undefined,
          `State not in location: ${location}`
        );
      }
    } else {
      logTest(
        "EndSession: State parameter preserved",
        "FAIL",
        undefined,
        `Expected 302, got ${response.status}`
      );
    }
  } catch (error) {
    logTest("EndSession: State parameter", "FAIL", undefined, String(error));
  }

  // Test 2.5: SECURITY - Invalid redirect URI is rejected (CWE-601)
  try {
    const maliciousUri = "https://evil.com/steal-tokens";
    const response = await fetch(
      `${endpoint}?post_logout_redirect_uri=${encodeURIComponent(maliciousUri)}`
    );

    if (response.status === 400) {
      const data = await response.json();
      if (data.error === "invalid_request") {
        logTest(
          "EndSession: SECURITY - Rejects malicious URI (CWE-601)",
          "PASS",
          "Returns 400 with invalid_request error"
        );
      } else {
        logTest(
          "EndSession: SECURITY - Rejects malicious URI (CWE-601)",
          "FAIL",
          undefined,
          `Wrong error: ${JSON.stringify(data)}`
        );
      }
    } else if (response.status === 302) {
      const location = response.headers.get("location");
      if (location?.includes("evil.com")) {
        logTest(
          "EndSession: SECURITY - Rejects malicious URI (CWE-601)",
          "FAIL",
          undefined,
          "CRITICAL: Open redirect vulnerability! Redirected to malicious site"
        );
      } else {
        logTest(
          "EndSession: SECURITY - Rejects malicious URI (CWE-601)",
          "PASS",
          `Redirected safely to: ${location}`
        );
      }
    } else {
      logTest(
        "EndSession: SECURITY - Rejects malicious URI (CWE-601)",
        "FAIL",
        undefined,
        `Unexpected status: ${response.status}`
      );
    }
  } catch (error) {
    logTest("EndSession: SECURITY - Malicious URI", "FAIL", undefined, String(error));
  }

  // Test 2.6: SECURITY - Reject javascript: URI
  try {
    const jsUri = "javascript:alert(1)";
    const response = await fetch(
      `${endpoint}?post_logout_redirect_uri=${encodeURIComponent(jsUri)}`
    );

    if (response.status === 400) {
      logTest(
        "EndSession: SECURITY - Rejects javascript: URI",
        "PASS",
        "Returns 400"
      );
    } else {
      logTest(
        "EndSession: SECURITY - Rejects javascript: URI",
        "FAIL",
        undefined,
        `Expected 400, got ${response.status}`
      );
    }
  } catch (error) {
    logTest("EndSession: SECURITY - javascript: URI", "FAIL", undefined, String(error));
  }

  // Test 2.7: SECURITY - Reject data: URI
  try {
    const dataUri = "data:text/html,<script>alert(1)</script>";
    const response = await fetch(
      `${endpoint}?post_logout_redirect_uri=${encodeURIComponent(dataUri)}`
    );

    if (response.status === 400) {
      logTest("EndSession: SECURITY - Rejects data: URI", "PASS", "Returns 400");
    } else {
      logTest(
        "EndSession: SECURITY - Rejects data: URI",
        "FAIL",
        undefined,
        `Expected 400, got ${response.status}`
      );
    }
  } catch (error) {
    logTest("EndSession: SECURITY - data: URI", "FAIL", undefined, String(error));
  }
}

// =============================================================================
// Test Suite 3: Discovery and EndSession Integration
// =============================================================================

async function testIntegration(discovery: Record<string, unknown> | null) {
  console.log("\n🔗 Test Suite 3: Integration Tests\n");

  // Test 3.1: Discovery proxies correctly from Better Auth
  try {
    const internalResponse = await fetch(
      `${BASE_URL}/api/auth/.well-known/openid-configuration`
    );
    const proxyResponse = await fetch(`${BASE_URL}/.well-known/openid-configuration`);

    if (internalResponse.ok && proxyResponse.ok) {
      const internal = await internalResponse.json();
      const proxy = await proxyResponse.json();

      // Compare key fields
      if (internal.issuer === proxy.issuer && internal.jwks_uri === proxy.jwks_uri) {
        logTest(
          "Integration: Discovery proxy matches internal",
          "PASS",
          "Issuer and JWKS URI match"
        );
      } else {
        logTest(
          "Integration: Discovery proxy matches internal",
          "FAIL",
          undefined,
          "Mismatch in discovery documents"
        );
      }
    } else {
      logTest(
        "Integration: Discovery proxy matches internal",
        "SKIP",
        undefined,
        "Could not compare endpoints"
      );
    }
  } catch (error) {
    logTest("Integration: Discovery proxy", "FAIL", undefined, String(error));
  }

  // Test 3.2: All endpoints use consistent base URL (no hardcoding)
  if (discovery) {
    const endpoints = [
      "authorization_endpoint",
      "token_endpoint",
      "userinfo_endpoint",
      "jwks_uri",
    ];

    let allMatch = true;
    for (const ep of endpoints) {
      const value = discovery[ep] as string;
      if (value && !value.startsWith(BASE_URL)) {
        allMatch = false;
        break;
      }
    }

    if (allMatch) {
      logTest(
        "Integration: All endpoints use consistent base URL",
        "PASS",
        "No hardcoded URLs detected"
      );
    } else {
      logTest(
        "Integration: All endpoints use consistent base URL",
        "FAIL",
        undefined,
        "Some endpoints don't match BASE_URL"
      );
    }
  }
}

// =============================================================================
// Summary and Main
// =============================================================================

function printSummary() {
  console.log("\n" + "=".repeat(60));
  console.log("📊 OIDC ENDPOINTS TEST SUMMARY");
  console.log("=".repeat(60) + "\n");

  const passed = results.filter((r) => r.status === "PASS").length;
  const failed = results.filter((r) => r.status === "FAIL").length;
  const skipped = results.filter((r) => r.status === "SKIP").length;

  console.log(`✅ PASS: ${passed}`);
  console.log(`❌ FAIL: ${failed}`);
  console.log(`⏭️  SKIP: ${skipped}`);
  console.log(`📈 Total: ${results.length}\n`);

  if (failed > 0) {
    console.log("❌ FAILED TESTS:\n");
    results
      .filter((r) => r.status === "FAIL")
      .forEach((r) => {
        console.log(`  • ${r.test}`);
        if (r.error) console.log(`    ${r.error}`);
      });
    console.log("");
  }

  return { passed, failed, skipped, total: results.length };
}

async function main() {
  console.log("🚀 OIDC Endpoints Test Suite");
  console.log(`🌐 Testing against: ${BASE_URL}\n`);

  // Run all test suites
  const discovery = await testDiscoveryEndpoint();
  await testEndSessionEndpoint();
  await testIntegration(discovery);

  // Print summary
  const summary = printSummary();

  // Exit with error code if any tests failed
  if (summary.failed > 0) {
    console.log("❌ Some tests failed. Please review and fix issues.\n");
    process.exit(1);
  } else {
    console.log("✅ All tests passed!\n");
    process.exit(0);
  }
}

main().catch((error) => {
  console.error("❌ Test suite crashed:", error);
  process.exit(1);
});
