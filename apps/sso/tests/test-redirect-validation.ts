/**
 * Test Suite for Redirect URL Validation
 * 
 * Verifies that the redirect validation logic correctly allows trusted origins
 * and blocks untrusted ones to prevent open redirect vulnerabilities.
 */

import { getTrustedOrigins, isValidRedirectUrl } from "../src/lib/redirect-utils";

console.log("============================================================");
console.log("Redirect URL Validation Test Suite");
console.log("============================================================\n");

// Mock window.location for testing
(global as any).window = {
  location: {
    origin: "http://localhost:3001"
  }
};

let passedTests = 0;
let totalTests = 0;

function test(name: string, fn: () => boolean) {
  totalTests++;
  try {
    const result = fn();
    if (result) {
      console.log(`✅ ${name}: PASS`);
      passedTests++;
    } else {
      console.log(`❌ ${name}: FAIL - Test returned false`);
    }
  } catch (error) {
    console.log(`❌ ${name}: FAIL - ${error instanceof Error ? error.message : String(error)}`);
  }
}

// Test 1: Get trusted origins returns expected origins
test("Get Trusted Origins - Contains localhost:3001", () => {
  const origins = getTrustedOrigins();
  return origins.includes("http://localhost:3001");
});

// Test 2: Get trusted origins includes RoboLearn
test("Get Trusted Origins - Contains RoboLearn", () => {
  const origins = getTrustedOrigins();
  return origins.includes("https://mjunaidca.github.io");
});

// Test 3: Get trusted origins includes AI Native (Legacy)
test("Get Trusted Origins - Contains AI Native (Legacy)", () => {
  const origins = getTrustedOrigins();
  return origins.includes("https://ai-native.panaversity.org");
});

// Test 3b: Get trusted origins includes Agent Factory
test("Get Trusted Origins - Contains Agent Factory", () => {
  const origins = getTrustedOrigins();
  return origins.includes("https://agentfactory.panaversity.org");
});

// Test 4: Relative URLs are valid
test("Valid Redirect - Relative URL", () => {
  return isValidRedirectUrl("/account/profile");
});

// Test 5: Same origin URLs are valid
test("Valid Redirect - Same Origin", () => {
  return isValidRedirectUrl("http://localhost:3001/auth/sign-in");
});

// Test 6: RoboLearn origin is valid
test("Valid Redirect - RoboLearn Origin", () => {
  return isValidRedirectUrl("https://mjunaidca.github.io/robolearn");
});

// Test 7: RoboLearn with path is valid
test("Valid Redirect - RoboLearn with Path", () => {
  return isValidRedirectUrl("https://mjunaidca.github.io/robolearn/chapter/1");
});

// Test 8: AI Native origin is valid
test("Valid Redirect - AI Native Origin", () => {
  return isValidRedirectUrl("https://ai-native.panaversity.org");
});

// Test 9: AI Native with path is valid
test("Valid Redirect - AI Native with Path", () => {
  return isValidRedirectUrl("https://ai-native.panaversity.org/dashboard");
});

// Test 9b: Agent Factory origin is valid
test("Valid Redirect - Agent Factory Origin", () => {
  return isValidRedirectUrl("https://agentfactory.panaversity.org");
});

// Test 9c: Agent Factory with path is valid
test("Valid Redirect - Agent Factory with Path", () => {
  return isValidRedirectUrl("https://agentfactory.panaversity.org/dashboard");
});

// Test 10: Untrusted origin is invalid
test("Invalid Redirect - Untrusted Origin", () => {
  return !isValidRedirectUrl("https://evil.example.com/steal-tokens");
});

// Test 11: Malicious URL with @ is invalid
test("Invalid Redirect - Malicious URL with @", () => {
  return !isValidRedirectUrl("https://mjunaidca.github.io@evil.com");
});

// Test 12: Empty string is invalid
test("Invalid Redirect - Empty String", () => {
  return !isValidRedirectUrl("");
});

// Test 13: Invalid URL format
test("Invalid Redirect - Malformed URL", () => {
  return !isValidRedirectUrl("not-a-valid-url");
});

// Test 14: localhost:3000 (client app) is valid
test("Valid Redirect - Client App (localhost:3000)", () => {
  return isValidRedirectUrl("http://localhost:3000");
});

// Test 15: Panaversity.org is valid
test("Valid Redirect - Panaversity.org", () => {
  const origins = getTrustedOrigins();
  // Check if panaversity.org is in trusted origins
  const hasPanaversity = origins.some(origin => origin.includes("panaversity.org"));
  return hasPanaversity && isValidRedirectUrl("https://panaversity.org/home");
});

// =============================================================================
// OAuth Callback URL Blocking Tests (Prevents PKCE errors after profile updates)
// =============================================================================
console.log("\n--- OAuth Callback URL Blocking Tests ---\n");

// Test 16: Block RoboLearn OAuth callback URL
test("Invalid Redirect - RoboLearn OAuth Callback", () => {
  return !isValidRedirectUrl("https://mjunaidca.github.io/robolearn/auth/callback");
});

// Test 17: Block AI Native OAuth callback URL
test("Invalid Redirect - AI Native OAuth Callback", () => {
  return !isValidRedirectUrl("https://ai-native.panaversity.org/auth/callback");
});

// Test 17b: Block Agent Factory OAuth callback URL
test("Invalid Redirect - Agent Factory OAuth Callback", () => {
  return !isValidRedirectUrl("https://agentfactory.panaversity.org/auth/callback");
});

// Test 18: Block localhost OAuth callback URL
test("Invalid Redirect - Localhost OAuth Callback", () => {
  return !isValidRedirectUrl("http://localhost:3000/auth/callback");
});

// Test 19: Block relative OAuth callback URL
test("Invalid Redirect - Relative OAuth Callback", () => {
  return !isValidRedirectUrl("/auth/callback");
});

// Test 20: Block OAuth callback with query params
test("Invalid Redirect - OAuth Callback with Query Params", () => {
  return !isValidRedirectUrl("https://mjunaidca.github.io/robolearn/auth/callback?code=abc123");
});

// Test 21: Block /oauth/callback pattern
test("Invalid Redirect - OAuth Pattern /oauth/callback", () => {
  return !isValidRedirectUrl("http://localhost:3000/oauth/callback");
});

// Test 22: Block /callback pattern
test("Invalid Redirect - Bare /callback Pattern", () => {
  return !isValidRedirectUrl("http://localhost:3000/callback");
});

// Test 23: Allow non-callback paths on same origin
test("Valid Redirect - Non-callback Path on Trusted Origin", () => {
  return isValidRedirectUrl("https://mjunaidca.github.io/robolearn/dashboard");
});

// Test 24: Allow deep paths that don't match callback
test("Valid Redirect - Deep Path Without Callback", () => {
  return isValidRedirectUrl("https://ai-native.panaversity.org/projects/123/settings");
});

// Test 25: Block same-origin callback
test("Invalid Redirect - Same Origin OAuth Callback", () => {
  return !isValidRedirectUrl("http://localhost:3001/auth/callback");
});

console.log("\n============================================================");
console.log(`Results: ${passedTests}/${totalTests} tests passed`);
if (passedTests !== totalTests) {
  console.log("\nFailed tests:");
  process.exit(1);
} else {
  console.log("\n✅ All tests passed!");
}
console.log("============================================================");
