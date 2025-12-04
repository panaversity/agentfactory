import { chromium } from 'playwright';

const TEST_EMAIL = 'admin@robolearn.io';
const TEST_PASSWORD = 'admin@robolearn.io';

async function testPKCEOAuthFlow() {
  console.log('=== PKCE OAuth Flow Test with Playwright ===\n');

  const browser = await chromium.launch({ headless: true });
  const context = await browser.newContext();
  const page = await context.newPage();

  try {
    // Step 1: Go to book interface homepage
    console.log('1. Loading book interface...');
    await page.goto('http://localhost:3000');
    await page.waitForLoadState('networkidle');
    console.log('   Homepage loaded: ' + page.url());

    // Step 2: Click Sign In button
    console.log('\n2. Looking for Sign In button...');
    const signInButton = await page.getByRole('button', { name: /sign in/i }).first();
    if (!signInButton) {
      // Try looking for a link instead
      const signInLink = await page.getByRole('link', { name: /sign in/i }).first();
      if (signInLink) {
        await signInLink.click();
      } else {
        console.log('   Could not find Sign In button or link');
        // Check what's on the page
        const pageContent = await page.content();
        console.log('   Page has navbar:', pageContent.includes('navbar'));

        // Try direct OAuth URL with PKCE
        console.log('   Trying direct OAuth flow...');
        // Generate PKCE values
        const codeVerifier = generateCodeVerifier();
        const codeChallenge = await generateCodeChallenge(codeVerifier);

        // Store verifier in sessionStorage
        await page.evaluate((verifier) => {
          sessionStorage.setItem('pkce_code_verifier', verifier);
        }, codeVerifier);

        // Navigate to authorize URL with PKCE
        const authUrl = `http://localhost:3001/api/auth/oauth2/authorize?client_id=robolearn-interface&redirect_uri=${encodeURIComponent('http://localhost:3000/auth/callback')}&response_type=code&scope=openid%20profile%20email&state=test&code_challenge=${codeChallenge}&code_challenge_method=S256`;
        await page.goto(authUrl);
        await page.waitForLoadState('networkidle');
        console.log('   Redirected to: ' + page.url());
      }
    } else {
      await signInButton.click();
      await page.waitForLoadState('networkidle');
      console.log('   Redirected to: ' + page.url());
    }

    // Step 3: Should be on auth server sign-in page
    console.log('\n3. On sign-in page...');
    console.log('   Current URL: ' + page.url());

    if (page.url().includes('localhost:3001/auth/sign-in')) {
      // Fill in credentials
      console.log('   Filling in credentials...');
      await page.fill('input[type="email"], input[name="email"]', TEST_EMAIL);
      await page.fill('input[type="password"], input[name="password"]', TEST_PASSWORD);

      // Click submit
      const submitButton = await page.getByRole('button', { name: /sign in|submit|login/i });
      await submitButton.click();

      // Wait for redirect
      await page.waitForLoadState('networkidle');
      console.log('   After login, URL: ' + page.url());
    }

    // Step 4: Check if we're back at callback or homepage with tokens
    console.log('\n4. Checking OAuth callback...');
    console.log('   Current URL: ' + page.url());

    // Wait a moment for token exchange to complete
    await page.waitForTimeout(2000);

    // Check localStorage for tokens
    const tokens = await page.evaluate(() => {
      return {
        accessToken: localStorage.getItem('robolearn_access_token'),
        refreshToken: localStorage.getItem('robolearn_refresh_token'),
        idToken: localStorage.getItem('robolearn_id_token'),
      };
    });

    console.log('   Access Token: ' + (tokens.accessToken ? 'YES (' + tokens.accessToken.slice(0, 20) + '...)' : 'NO'));
    console.log('   Refresh Token: ' + (tokens.refreshToken ? 'YES' : 'NO'));
    console.log('   ID Token: ' + (tokens.idToken ? 'YES' : 'NO'));

    // Step 5: Verify userinfo endpoint with custom claims
    if (tokens.accessToken) {
      console.log('\n5. Testing userinfo endpoint with custom claims...');
      const userinfo = await page.evaluate(async (accessToken) => {
        const response = await fetch('http://localhost:3001/api/auth/oauth2/userinfo', {
          headers: { 'Authorization': 'Bearer ' + accessToken },
        });
        return response.ok ? await response.json() : null;
      }, tokens.accessToken);

      if (userinfo) {
        console.log('   Userinfo:', JSON.stringify(userinfo, null, 2));
        console.log('   Has role claim: ' + (userinfo.role ? 'YES (' + userinfo.role + ')' : 'NO'));
        console.log('   Has software_background: ' + (userinfo.software_background !== undefined ? 'YES' : 'NO'));
      } else {
        console.log('   Failed to get userinfo');
      }

      // Step 6: Test token refresh
      console.log('\n6. Testing token refresh...');
      if (tokens.refreshToken) {
        const refreshResult = await page.evaluate(async (refreshToken) => {
          const response = await fetch('http://localhost:3001/api/auth/oauth2/token', {
            method: 'POST',
            headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
            body: new URLSearchParams({
              grant_type: 'refresh_token',
              refresh_token: refreshToken,
              client_id: 'robolearn-interface',
            }),
          });
          return { ok: response.ok, data: response.ok ? await response.json() : await response.text() };
        }, tokens.refreshToken);

        console.log('   Refresh status: ' + (refreshResult.ok ? 'SUCCESS' : 'FAILED'));
        if (refreshResult.ok) {
          console.log('   New access token: YES');
        } else {
          console.log('   Error:', refreshResult.data);
        }
      } else {
        console.log('   No refresh token to test');
      }

      console.log('\n=== PKCE OAuth Flow: SUCCESS ===');
    } else {
      console.log('\n=== PKCE OAuth Flow: INCOMPLETE (no tokens) ===');
      console.log('Final URL: ' + page.url());

      // Take a screenshot for debugging
      await page.screenshot({ path: '/tmp/oauth-debug.png' });
      console.log('Screenshot saved to /tmp/oauth-debug.png');
    }

  } catch (error) {
    console.error('Test failed:', error.message);
    await page.screenshot({ path: '/tmp/oauth-error.png' });
    console.log('Error screenshot saved to /tmp/oauth-error.png');
  } finally {
    await browser.close();
  }
}

// PKCE helpers
function generateCodeVerifier() {
  const array = new Uint8Array(32);
  crypto.getRandomValues(array);
  return base64UrlEncode(array);
}

async function generateCodeChallenge(verifier) {
  const encoder = new TextEncoder();
  const data = encoder.encode(verifier);
  const hash = await crypto.subtle.digest('SHA-256', data);
  return base64UrlEncode(new Uint8Array(hash));
}

function base64UrlEncode(buffer) {
  let binary = '';
  for (let i = 0; i < buffer.length; i++) {
    binary += String.fromCharCode(buffer[i]);
  }
  return btoa(binary).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, '');
}

testPKCEOAuthFlow().catch(console.error);
