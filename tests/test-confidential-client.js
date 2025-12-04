/**
 * Confidential Client Test
 * Tests server-to-server OAuth flow with client secret
 *
 * Prerequisites:
 * 1. Auth server running: pnpm dev
 * 2. Confidential client seeded with secret
 *
 * Run: node tests/test-confidential-client.js
 */

const CLIENT_ID = "robolearn-confidential-client";
const CLIENT_SECRET = "robolearn-confidential-secret-for-testing-only";
const AUTH_SERVER = "http://localhost:3001";
const REDIRECT_URI = "http://localhost:8000/auth/callback"; // Must match trusted-clients.ts

// Test user credentials
const TEST_USER = {
  email: "admin@robolearn.io",
  password: "admin@robolearn.io"
};

async function testConfidentialClient() {
  console.log("============================================================");
  console.log("Confidential Client OAuth Flow Test");
  console.log("============================================================\n");

  try {
    // Step 1: Get authorization code (simulated - in real flow, user would authorize in browser)
    console.log("Step 1: Authorization Code Flow");
    console.log("Note: In production, user authorizes in browser, then backend receives code\n");

    // For testing, we'll use the authorization code flow
    const authUrl = new URL(`${AUTH_SERVER}/api/auth/oauth2/authorize`);
    authUrl.searchParams.set("client_id", CLIENT_ID);
    authUrl.searchParams.set("redirect_uri", REDIRECT_URI);
    authUrl.searchParams.set("response_type", "code");
    authUrl.searchParams.set("scope", "openid profile email offline_access"); // offline_access for refresh token

    console.log("Authorization URL:", authUrl.toString());
    console.log("(User would visit this URL, sign in, and approve)\n");

    // Step 2: Simulate getting authorization code
    // In real flow: User signs in → Redirected to redirect_uri?code=XXX
    // For testing: We'll manually get a code by signing in

    console.log("Step 2: Sign in and get session cookie...");
    const signInResponse = await fetch(`${AUTH_SERVER}/api/auth/sign-in/email`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(TEST_USER)
    });

    if (!signInResponse.ok) {
      throw new Error(`Sign in failed: ${signInResponse.status}`);
    }

    const cookies = signInResponse.headers.get("set-cookie");
    console.log("✓ Signed in successfully\n");

    // Step 3: Get authorization code
    console.log("Step 3: Request authorization code...");
    const authResponse = await fetch(authUrl.toString(), {
      headers: { Cookie: cookies },
      redirect: "manual"
    });

    // Better Auth returns JSON with redirect URL (not a 302 redirect)
    let code;
    if (authResponse.status === 200) {
      const body = await authResponse.json();
      if (body.redirect && body.url) {
        const match = body.url.match(/code=([^&]+)/);
        if (match) code = match[1];
      }
    }

    if (!code) {
      throw new Error("No authorization code in redirect");
    }

    console.log("✓ Authorization code received:", code.substring(0, 20) + "...\n");

    // Step 4: Exchange code for tokens using client secret
    console.log("Step 4: Exchange code for tokens (with client secret)...");

    const tokenResponse = await fetch(`${AUTH_SERVER}/api/auth/oauth2/token`, {
      method: "POST",
      headers: {
        "Content-Type": "application/x-www-form-urlencoded",
        // Confidential client uses Basic Auth with client_id:client_secret
        "Authorization": `Basic ${Buffer.from(`${CLIENT_ID}:${CLIENT_SECRET}`).toString("base64")}`
      },
      body: new URLSearchParams({
        grant_type: "authorization_code",
        code: code,
        redirect_uri: REDIRECT_URI
      })
    });

    if (!tokenResponse.ok) {
      const error = await tokenResponse.text();
      throw new Error(`Token exchange failed: ${tokenResponse.status} - ${error}`);
    }

    const tokens = await tokenResponse.json();
    console.log("✓ Tokens received:");
    console.log("  - access_token:", tokens.access_token?.substring(0, 30) + "...");
    console.log("  - refresh_token:", tokens.refresh_token?.substring(0, 30) + "...");
    console.log("  - expires_in:", tokens.expires_in, "seconds");
    console.log("  - token_type:", tokens.token_type);
    console.log();

    // Step 5: Get user info
    console.log("Step 5: Fetch user info with access token...");
    const userinfoResponse = await fetch(`${AUTH_SERVER}/api/auth/oauth2/userinfo`, {
      headers: {
        "Authorization": `Bearer ${tokens.access_token}`
      }
    });

    if (!userinfoResponse.ok) {
      throw new Error(`Userinfo failed: ${userinfoResponse.status}`);
    }

    const userinfo = await userinfoResponse.json();
    console.log("✓ User info received:");
    console.log(JSON.stringify(userinfo, null, 2));
    console.log();

    // Step 6: Verify tenant claims
    console.log("Step 6: Verify tenant/organization claims...");
    const requiredClaims = ["sub", "email", "tenant_id", "organization_ids", "org_role"];
    const missingClaims = requiredClaims.filter(claim => !(claim in userinfo));

    if (missingClaims.length > 0) {
      throw new Error(`Missing claims: ${missingClaims.join(", ")}`);
    }

    console.log("✓ All required claims present");
    console.log("  - tenant_id:", userinfo.tenant_id);
    console.log("  - organization_ids:", userinfo.organization_ids);
    console.log("  - org_role:", userinfo.org_role);
    console.log();

    // Step 7: Test refresh token
    console.log("Step 7: Test refresh token...");
    const refreshResponse = await fetch(`${AUTH_SERVER}/api/auth/oauth2/token`, {
      method: "POST",
      headers: {
        "Content-Type": "application/x-www-form-urlencoded",
        "Authorization": `Basic ${Buffer.from(`${CLIENT_ID}:${CLIENT_SECRET}`).toString("base64")}`
      },
      body: new URLSearchParams({
        grant_type: "refresh_token",
        refresh_token: tokens.refresh_token
      })
    });

    if (!refreshResponse.ok) {
      throw new Error(`Refresh token failed: ${refreshResponse.status}`);
    }

    const newTokens = await refreshResponse.json();
    console.log("✓ New tokens received:");
    console.log("  - new access_token:", newTokens.access_token?.substring(0, 30) + "...");
    console.log("  - expires_in:", newTokens.expires_in, "seconds");
    console.log();

    console.log("============================================================");
    console.log("✅ ALL TESTS PASSED");
    console.log("============================================================");
    console.log("\nConfidential client OAuth flow working correctly!");
    console.log("- Authorization code exchange: ✓");
    console.log("- Client secret authentication: ✓");
    console.log("- Access token issued: ✓");
    console.log("- User info retrieval: ✓");
    console.log("- Tenant claims present: ✓");
    console.log("- Refresh token working: ✓");

  } catch (error) {
    console.error("\n❌ TEST FAILED:", error.message);
    process.exit(1);
  }
}

// Run the test
testConfidentialClient();
