const { chromium } = require('playwright');

async function testOAuthFlow() {
  console.log('\n=== COMPLETE SSO/OAUTH FLOW TEST ===\n');

  const browser = await chromium.launch({ headless: false, slowMo: 500 });
  const context = await browser.newContext();
  const page = await context.newPage();

  const testEmail = `oauth-test-${Date.now()}@robolearn.io`;
  const testPassword = 'TestPassword123';

  try {
    // Step 1: Go to auth server home
    console.log('1. Opening Auth Server...');
    await page.goto('http://localhost:3001');
    await page.waitForLoadState('networkidle');
    await page.screenshot({ path: '/tmp/oauth-01-auth-home.png' });
    console.log('   Screenshot: /tmp/oauth-01-auth-home.png');

    // Step 2: Go to sign-up
    console.log('\n2. Opening Sign-Up page...');
    await page.goto('http://localhost:3001/auth/sign-up');
    await page.waitForLoadState('networkidle');
    await page.screenshot({ path: '/tmp/oauth-02-signup.png' });
    console.log('   Screenshot: /tmp/oauth-02-signup.png');

    // Step 3: Fill sign-up form
    console.log('\n3. Filling sign-up form...');
    await page.fill('input#name', 'OAuth Test User');
    await page.fill('input#email', testEmail);
    await page.fill('input#password', testPassword);
    await page.fill('input#confirmPassword', testPassword);
    await page.click('input[value="intermediate"]');
    await page.screenshot({ path: '/tmp/oauth-03-signup-filled.png' });
    console.log('   Email:', testEmail);

    // Step 4: Submit sign-up
    console.log('\n4. Submitting sign-up...');
    await page.click('button[type="submit"]');
    await page.waitForTimeout(3000);
    await page.screenshot({ path: '/tmp/oauth-04-after-signup.png' });
    console.log('   Current URL:', page.url());

    // Step 5: Go to Admin dashboard (as admin user)
    console.log('\n5. Testing Admin Dashboard...');
    // First sign in as admin
    await page.goto('http://localhost:3001/auth/sign-in');
    await page.waitForLoadState('networkidle');
    await page.fill('input#email', 'mjs@gmail.com');
    await page.fill('input#password', 'TestPassword123'); // You'll need to use the actual password
    await page.screenshot({ path: '/tmp/oauth-05-admin-signin.png' });
    console.log('   Screenshot: /tmp/oauth-05-admin-signin.png');
    console.log('   Note: Admin login requires correct password for mjs@gmail.com');

    // Step 6: Test OIDC endpoints
    console.log('\n6. Testing OIDC Discovery endpoint...');
    await page.goto('http://localhost:3001/api/auth/.well-known/openid-configuration');
    await page.waitForTimeout(1000);
    await page.screenshot({ path: '/tmp/oauth-06-oidc-discovery.png' });
    console.log('   Screenshot: /tmp/oauth-06-oidc-discovery.png');

    // Step 7: Test OAuth authorize endpoint
    console.log('\n7. Testing OAuth Authorization endpoint...');
    const authUrl = 'http://localhost:3001/api/auth/oauth2/authorize?' + new URLSearchParams({
      client_id: 'robolearn-interface',
      redirect_uri: 'http://localhost:3000/auth/callback',
      response_type: 'code',
      scope: 'openid profile email',
      state: 'test-state-123'
    }).toString();

    await page.goto(authUrl);
    await page.waitForTimeout(2000);
    await page.screenshot({ path: '/tmp/oauth-07-oauth-authorize.png' });
    console.log('   Current URL:', page.url());
    console.log('   Screenshot: /tmp/oauth-07-oauth-authorize.png');

    console.log('\n=== TEST COMPLETE ===');
    console.log('\nScreenshots saved to /tmp/oauth-*.png');
    console.log('View them with: open /tmp/oauth-01-auth-home.png');

  } catch (error) {
    console.error('Test error:', error.message);
    await page.screenshot({ path: '/tmp/oauth-error.png' });
  }

  // Keep browser open for manual inspection
  console.log('\nBrowser will close in 10 seconds...');
  await page.waitForTimeout(10000);
  await browser.close();
}

testOAuthFlow().catch(console.error);
