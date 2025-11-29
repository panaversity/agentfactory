const { chromium } = require('playwright');

async function testCompleteSSOFlow() {
  console.log('\n=== COMPLETE SSO FLOW TEST ===\n');

  const browser = await chromium.launch({ headless: false, slowMo: 300 });
  const context = await browser.newContext();
  const page = await context.newPage();

  const testEmail = `sso-${Date.now()}@robolearn.io`;
  const testPassword = 'TestPassword123';

  try {
    // ============================================
    // TEST 1: Start OAuth flow from RoboLearn Interface
    // ============================================
    console.log('=== TEST 1: OAuth Flow from Book Interface ===');

    console.log('\n1.1 Opening RoboLearn Interface...');
    await page.goto('http://localhost:3000');
    await page.waitForLoadState('networkidle');
    await page.screenshot({ path: '/tmp/sso-01-interface-home.png' });
    console.log('    Screenshot: /tmp/sso-01-interface-home.png');

    // Look for Sign In or Get Started button
    console.log('\n1.2 Clicking Get Started/Sign In...');
    const getStartedBtn = page.locator('button:has-text("Get Started"), a:has-text("Get Started")').first();
    const signInBtn = page.locator('button:has-text("Sign In"), a:has-text("Sign In")').first();

    if (await getStartedBtn.isVisible()) {
      await getStartedBtn.click();
    } else if (await signInBtn.isVisible()) {
      await signInBtn.click();
    } else {
      console.log('    No auth buttons found, navigating directly to sign-up flow...');
      // Build OAuth URL manually like the interface does
      const oauthUrl = 'http://localhost:3001/auth/sign-up?redirect=' +
        encodeURIComponent('http://localhost:3001/api/auth/oauth2/authorize?' +
          new URLSearchParams({
            client_id: 'robolearn-interface',
            redirect_uri: 'http://localhost:3000/auth/callback',
            response_type: 'code',
            scope: 'openid profile email',
            state: 'test-signup-flow'
          }).toString());
      await page.goto(oauthUrl);
    }

    await page.waitForTimeout(2000);
    await page.screenshot({ path: '/tmp/sso-02-after-click.png' });
    console.log('    Current URL:', page.url());
    console.log('    Screenshot: /tmp/sso-02-after-click.png');

    // ============================================
    // TEST 2: Sign Up Flow
    // ============================================
    console.log('\n=== TEST 2: Sign Up on Auth Server ===');

    // Make sure we're on sign-up page
    if (!page.url().includes('/auth/sign-up')) {
      console.log('\n2.0 Navigating to sign-up page...');
      await page.goto('http://localhost:3001/auth/sign-up');
      await page.waitForLoadState('networkidle');
    }

    console.log('\n2.1 Filling sign-up form...');
    await page.fill('input#name', 'SSO Test User');
    await page.fill('input#email', testEmail);
    await page.fill('input#password', testPassword);
    await page.fill('input#confirmPassword', testPassword);

    // Select intermediate background
    const intermediateRadio = page.locator('input[value="intermediate"]');
    if (await intermediateRadio.isVisible()) {
      await intermediateRadio.click();
    }

    await page.screenshot({ path: '/tmp/sso-03-signup-filled.png' });
    console.log('    Email:', testEmail);
    console.log('    Screenshot: /tmp/sso-03-signup-filled.png');

    console.log('\n2.2 Submitting sign-up...');
    await page.click('button[type="submit"]');

    // Wait for navigation or error
    await page.waitForTimeout(5000);
    await page.screenshot({ path: '/tmp/sso-04-after-signup.png' });
    console.log('    Current URL:', page.url());
    console.log('    Screenshot: /tmp/sso-04-after-signup.png');

    // Check if we got an error
    const errorElement = page.locator('.bg-red-50, .text-red-600');
    if (await errorElement.isVisible()) {
      const errorText = await errorElement.textContent();
      console.log('    Error:', errorText);
    }

    // ============================================
    // TEST 3: Direct OAuth Authorization (as logged-in user)
    // ============================================
    console.log('\n=== TEST 3: OAuth Authorization Flow ===');

    // First check if we have a session
    console.log('\n3.1 Checking session...');
    await page.goto('http://localhost:3001/api/auth/get-session');
    await page.waitForTimeout(1000);
    const sessionText = await page.textContent('body');
    console.log('    Session:', sessionText.substring(0, 200));
    await page.screenshot({ path: '/tmp/sso-05-session.png' });

    // Now try OAuth authorize
    console.log('\n3.2 Testing OAuth authorization...');
    const authUrl = 'http://localhost:3001/api/auth/oauth2/authorize?' + new URLSearchParams({
      client_id: 'robolearn-interface',
      redirect_uri: 'http://localhost:3000/auth/callback',
      response_type: 'code',
      scope: 'openid profile email',
      state: 'test-state-123'
    }).toString();

    await page.goto(authUrl);
    await page.waitForTimeout(3000);
    await page.screenshot({ path: '/tmp/sso-06-oauth-result.png' });
    console.log('    Current URL:', page.url());
    console.log('    Screenshot: /tmp/sso-06-oauth-result.png');

    // Check if we got redirected with a code
    const currentUrl = page.url();
    if (currentUrl.includes('code=')) {
      console.log('    ✅ SUCCESS: Got authorization code!');
      const urlObj = new URL(currentUrl);
      const code = urlObj.searchParams.get('code');
      console.log('    Code:', code ? code.substring(0, 20) + '...' : 'N/A');
    } else if (currentUrl.includes('/auth/sign-in')) {
      console.log('    ⚠️  Redirected to sign-in (session may not be active)');
    } else if (currentUrl.includes('/auth/consent')) {
      console.log('    📋 Consent page shown (expected for non-trusted clients)');
    }

    // ============================================
    // TEST 4: Sign In and Complete OAuth
    // ============================================
    if (currentUrl.includes('/auth/sign-in')) {
      console.log('\n=== TEST 4: Sign In and Complete OAuth ===');

      console.log('\n4.1 Signing in with new account...');
      await page.fill('input#email', testEmail);
      await page.fill('input#password', testPassword);
      await page.screenshot({ path: '/tmp/sso-07-signin-filled.png' });

      await page.click('button[type="submit"]');
      await page.waitForTimeout(3000);
      await page.screenshot({ path: '/tmp/sso-08-after-signin.png' });
      console.log('    Current URL:', page.url());

      // Check for code again
      const newUrl = page.url();
      if (newUrl.includes('code=')) {
        console.log('    ✅ SUCCESS: Got authorization code after sign-in!');
        const urlObj = new URL(newUrl);
        const code = urlObj.searchParams.get('code');
        console.log('    Code:', code ? code.substring(0, 20) + '...' : 'N/A');
      }
    }

    // ============================================
    // TEST 5: Admin Dashboard
    // ============================================
    console.log('\n=== TEST 5: Admin Dashboard ===');

    console.log('\n5.1 Opening Admin Dashboard...');
    await page.goto('http://localhost:3001/admin');
    await page.waitForTimeout(2000);
    await page.screenshot({ path: '/tmp/sso-09-admin.png' });
    console.log('    Current URL:', page.url());
    console.log('    Screenshot: /tmp/sso-09-admin.png');

    // ============================================
    // TEST 6: OIDC Discovery
    // ============================================
    console.log('\n=== TEST 6: OIDC Endpoints ===');

    console.log('\n6.1 OIDC Discovery...');
    await page.goto('http://localhost:3001/api/auth/.well-known/openid-configuration');
    await page.waitForTimeout(1000);
    const oidcConfig = await page.textContent('body');
    console.log('    Config preview:', oidcConfig.substring(0, 150) + '...');
    await page.screenshot({ path: '/tmp/sso-10-oidc-discovery.png' });

    console.log('\n=== ALL TESTS COMPLETE ===');
    console.log('\nScreenshots saved to /tmp/sso-*.png');
    console.log('View them with: open /tmp/sso-01-interface-home.png');

  } catch (error) {
    console.error('\n❌ Test error:', error.message);
    await page.screenshot({ path: '/tmp/sso-error.png' });
    console.log('Error screenshot: /tmp/sso-error.png');
  }

  // Keep browser open briefly for inspection
  console.log('\nBrowser will close in 5 seconds...');
  await page.waitForTimeout(5000);
  await browser.close();
}

testCompleteSSOFlow().catch(console.error);
