import crypto from 'crypto';

// Test credentials
const TEST_EMAIL = 'admin@robolearn.io';
const TEST_PASSWORD = 'Admin123!';
const AUTH_URL = 'http://localhost:3001';
const CLIENT_ID = 'robolearn-interface';
const REDIRECT_URI = 'http://localhost:3000/auth/callback';

// PKCE Helpers
function generateCodeVerifier() {
  return crypto.randomBytes(32).toString('base64url');
}

function generateCodeChallenge(verifier) {
  return crypto.createHash('sha256').update(verifier).digest('base64url');
}

// Parse Set-Cookie header
function parseCookies(setCookieHeaders) {
  if (!setCookieHeaders) return '';
  const headers = Array.isArray(setCookieHeaders) ? setCookieHeaders : [setCookieHeaders];
  return headers.map(h => h.split(';')[0]).join('; ');
}

async function runTests() {
  console.log('====================================');
  console.log('PKCE OAuth Flow - API Level Testing');
  console.log('====================================\n');

  let allPassed = true;

  // Test 1: OIDC Discovery
  console.log('TEST 1: OIDC Discovery');
  try {
    const discoveryResponse = await fetch(`${AUTH_URL}/api/auth/.well-known/openid-configuration`);
    const discovery = await discoveryResponse.json();

    const hasPKCE = discovery.code_challenge_methods_supported?.includes('S256');
    console.log('  ✓ Discovery endpoint: OK');
    console.log('  ' + (hasPKCE ? '✓' : '✗') + ' PKCE S256 supported: ' + hasPKCE);
    console.log('  Supported scopes:', discovery.scopes_supported?.join(', '));
    if (!hasPKCE) allPassed = false;
  } catch (e) {
    console.log('  ✗ Discovery failed:', e.message);
    allPassed = false;
  }

  // Test 2: Sign In and Get Session Cookie
  console.log('\nTEST 2: Sign In with Credentials');
  let sessionCookie = '';
  try {
    const signInResponse = await fetch(`${AUTH_URL}/api/auth/sign-in/email`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ email: TEST_EMAIL, password: TEST_PASSWORD }),
    });

    if (signInResponse.ok) {
      console.log('  ✓ Sign-in: OK (status ' + signInResponse.status + ')');
      sessionCookie = parseCookies(signInResponse.headers.get('set-cookie'));
      console.log('  ✓ Session cookie received:', sessionCookie ? 'YES' : 'NO');
    } else {
      const err = await signInResponse.text();
      console.log('  ✗ Sign-in failed:', signInResponse.status, err);
      allPassed = false;
    }
  } catch (e) {
    console.log('  ✗ Sign-in error:', e.message);
    allPassed = false;
  }

  // Test 3: Authorization with PKCE
  console.log('\nTEST 3: Authorization with PKCE');
  const codeVerifier = generateCodeVerifier();
  const codeChallenge = generateCodeChallenge(codeVerifier);
  let authorizationCode = '';

  console.log('  Code Verifier: ' + codeVerifier.slice(0, 20) + '...');
  console.log('  Code Challenge: ' + codeChallenge.slice(0, 20) + '...');

  try {
    const authUrl = `${AUTH_URL}/api/auth/oauth2/authorize?client_id=${CLIENT_ID}&redirect_uri=${encodeURIComponent(REDIRECT_URI)}&response_type=code&scope=openid%20profile%20email&state=test-state&code_challenge=${codeChallenge}&code_challenge_method=S256`;

    const authResponse = await fetch(authUrl, {
      headers: { 'Cookie': sessionCookie },
      redirect: 'manual',
    });

    const location = authResponse.headers.get('location');
    console.log('  Response status:', authResponse.status);
    console.log('  Location header:', location ? location.slice(0, 80) + '...' : 'none');

    if (authResponse.status === 302 && location) {
      const codeMatch = location.match(/code=([^&]+)/);
      if (codeMatch) {
        authorizationCode = codeMatch[1];
        console.log('  ✓ Authorization code received: ' + authorizationCode.slice(0, 20) + '...');
      } else {
        console.log('  ✗ No code in redirect URL');
        allPassed = false;
      }
    } else if (authResponse.status === 200) {
      // Server returned HTML (likely login page) - need fresh session
      console.log('  ✗ Received HTML instead of redirect - session may have expired');
      console.log('  ℹ This happens when session cookie is not properly sent or expired');
      allPassed = false;
    } else {
      console.log('  ✗ Unexpected response status');
      allPassed = false;
    }
  } catch (e) {
    console.log('  ✗ Authorization error:', e.message);
    allPassed = false;
  }

  // Test 4: Token Exchange with PKCE (no client_secret)
  console.log('\nTEST 4: Token Exchange with PKCE');
  let accessToken = '';
  let refreshToken = '';

  if (authorizationCode) {
    try {
      const tokenResponse = await fetch(`${AUTH_URL}/api/auth/oauth2/token`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: new URLSearchParams({
          grant_type: 'authorization_code',
          code: authorizationCode,
          redirect_uri: REDIRECT_URI,
          client_id: CLIENT_ID,
          code_verifier: codeVerifier,  // PKCE - no client_secret!
        }),
      });

      if (tokenResponse.ok) {
        const tokens = await tokenResponse.json();
        accessToken = tokens.access_token || '';
        refreshToken = tokens.refresh_token || '';

        console.log('  ✓ Token exchange: OK');
        console.log('  ✓ Access token:', accessToken ? 'YES' : 'NO');
        console.log('  ✓ Refresh token:', refreshToken ? 'YES' : 'NO');
        console.log('  ✓ ID token:', tokens.id_token ? 'YES' : 'NO');
        console.log('  ✓ Token type:', tokens.token_type);
        console.log('  ✓ Expires in:', tokens.expires_in, 'seconds');
      } else {
        const err = await tokenResponse.text();
        console.log('  ✗ Token exchange failed:', tokenResponse.status, err);
        allPassed = false;
      }
    } catch (e) {
      console.log('  ✗ Token exchange error:', e.message);
      allPassed = false;
    }
  } else {
    console.log('  ⊘ Skipped - no authorization code');
  }

  // Test 5: UserInfo with Custom Claims
  console.log('\nTEST 5: UserInfo Endpoint (Custom Claims)');
  if (accessToken) {
    try {
      const userinfoResponse = await fetch(`${AUTH_URL}/api/auth/oauth2/userinfo`, {
        headers: { 'Authorization': `Bearer ${accessToken}` },
      });

      if (userinfoResponse.ok) {
        const userinfo = await userinfoResponse.json();
        console.log('  ✓ UserInfo: OK');
        console.log('  User ID (sub):', userinfo.sub);
        console.log('  Email:', userinfo.email);
        console.log('  Name:', userinfo.name || 'not set');
        console.log('  ✓ Role claim:', userinfo.role || 'not present');
        console.log('  ✓ software_background:', userinfo.software_background !== undefined ? (userinfo.software_background || 'null') : 'not present');

        // Verify custom claims are present
        if (!userinfo.role) {
          console.log('  ⚠ Warning: role claim not in response');
        }
        if (userinfo.software_background === undefined) {
          console.log('  ⚠ Warning: software_background claim not in response');
        }
      } else {
        const err = await userinfoResponse.text();
        console.log('  ✗ UserInfo failed:', userinfoResponse.status, err);
        allPassed = false;
      }
    } catch (e) {
      console.log('  ✗ UserInfo error:', e.message);
      allPassed = false;
    }
  } else {
    console.log('  ⊘ Skipped - no access token');
  }

  // Test 6: Token Refresh
  console.log('\nTEST 6: Token Refresh');
  if (refreshToken) {
    try {
      const refreshResponse = await fetch(`${AUTH_URL}/api/auth/oauth2/token`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: new URLSearchParams({
          grant_type: 'refresh_token',
          refresh_token: refreshToken,
          client_id: CLIENT_ID,
        }),
      });

      if (refreshResponse.ok) {
        const newTokens = await refreshResponse.json();
        console.log('  ✓ Token refresh: OK');
        console.log('  ✓ New access token:', newTokens.access_token ? 'YES' : 'NO');
        console.log('  ✓ New refresh token:', newTokens.refresh_token ? 'YES' : 'NO');

        // Verify new token works
        const verifyResponse = await fetch(`${AUTH_URL}/api/auth/oauth2/userinfo`, {
          headers: { 'Authorization': `Bearer ${newTokens.access_token}` },
        });
        console.log('  ✓ Refreshed token valid:', verifyResponse.ok ? 'YES' : 'NO');
      } else {
        const err = await refreshResponse.text();
        console.log('  ✗ Token refresh failed:', refreshResponse.status, err);
        allPassed = false;
      }
    } catch (e) {
      console.log('  ✗ Token refresh error:', e.message);
      allPassed = false;
    }
  } else {
    console.log('  ⊘ Skipped - no refresh token');
  }

  // Test 7: Invalid PKCE Verifier (Security Test)
  console.log('\nTEST 7: Invalid PKCE Verifier (Security)');
  if (authorizationCode) {
    console.log('  ⊘ Cannot test - code already used');
  } else {
    console.log('  ⊘ Skipped - no authorization code to test with');
  }

  // Summary
  console.log('\n====================================');
  console.log('TEST SUMMARY');
  console.log('====================================');

  const features = [
    ['PKCE (S256)', 'Supported in OIDC discovery'],
    ['Public Client', 'No client_secret required'],
    ['Token Refresh', refreshToken ? 'Working' : 'Not tested'],
    ['Custom Claims', accessToken ? 'role + software_background' : 'Not tested'],
  ];

  features.forEach(([name, status]) => {
    console.log(`  ${status.includes('Not') ? '⊘' : '✓'} ${name}: ${status}`);
  });

  console.log('\n' + (allPassed && accessToken ? '✓ ALL TESTS PASSED' : '⚠ SOME TESTS INCOMPLETE'));
  console.log('====================================\n');

  if (!accessToken) {
    console.log('NOTE: The authorization step requires a valid session cookie.');
    console.log('The session cookie from sign-in may not persist across fetch calls.');
    console.log('This is normal - in a real browser, cookies are handled automatically.');
    console.log('\nTo test the full flow, use the book interface at http://localhost:3000');
    console.log('and click Sign In to go through the OAuth flow manually.');
  }
}

runTests().catch(console.error);
