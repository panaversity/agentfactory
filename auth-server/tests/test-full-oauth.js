const { chromium } = require('playwright');

async function testFullOAuthFlow() {
  console.log('\n=== FULL OAUTH/SSO FLOW TEST ===\n');

  const browser = await chromium.launch({ headless: false, slowMo: 200 });
  const context = await browser.newContext();
  const page = await context.newPage();

  const testEmail = `oauth-${Date.now()}@robolearn.io`;
  const testPassword = 'TestPassword123!';

  try {
    // ============================================
    // Step 1: Start from RoboLearn Interface Homepage
    // ============================================
    console.log('STEP 1: Starting from RoboLearn Interface');
    await page.goto('http://localhost:3000');
    await page.waitForLoadState('networkidle');
    await page.screenshot({ path: '/tmp/full-01-interface-home.png' });
    console.log('  ✓ Loaded interface homepage');

    // Click "Get Started" button which should redirect to OAuth sign-up flow
    console.log('\nSTEP 2: Clicking Get Started (initiates OAuth flow)');
    const getStartedBtn = page.locator('button:has-text("Get Started")').first();
    if (await getStartedBtn.isVisible()) {
      await getStartedBtn.click();
      // Wait for navigation to auth server
      await page.waitForURL(/localhost:3001/, { timeout: 10000 });
    }
    await page.screenshot({ path: '/tmp/full-02-redirected-to-auth.png' });
    console.log('  ✓ Redirected to auth server:', page.url().substring(0, 80));

    // ============================================
    // Step 2: Complete Sign Up on Auth Server
    // ============================================
    console.log('\nSTEP 3: Completing sign-up form');
    await page.waitForSelector('input#email', { timeout: 5000 });

    await page.fill('input#name', 'OAuth Test User');
    await page.fill('input#email', testEmail);
    await page.fill('input#password', testPassword);
    await page.fill('input#confirmPassword', testPassword);

    // Click intermediate background if visible
    const intermediateBtn = page.locator('input[value="intermediate"]');
    if (await intermediateBtn.isVisible()) {
      await intermediateBtn.click();
    }

    await page.screenshot({ path: '/tmp/full-03-signup-filled.png' });
    console.log('  ✓ Form filled with email:', testEmail);

    // Submit and wait for navigation
    console.log('\nSTEP 4: Submitting sign-up form');
    await page.click('button[type="submit"]');

    // Wait for either:
    // 1. Redirect to OAuth authorize (successful signup continues OAuth flow)
    // 2. Error message appears
    // 3. Redirect to interface (if OAuth flow completes)
    try {
      await Promise.race([
        page.waitForURL(/oauth2\/authorize|localhost:3000|code=/, { timeout: 15000 }),
        page.waitForSelector('.bg-red-50', { timeout: 15000 }),
      ]);
    } catch (e) {
      console.log('  ⏳ Waiting for page state to settle...');
      await page.waitForTimeout(3000);
    }

    await page.screenshot({ path: '/tmp/full-04-after-signup.png' });
    console.log('  Current URL:', page.url().substring(0, 100));

    // Check for errors
    const errorEl = page.locator('.bg-red-50');
    if (await errorEl.isVisible()) {
      const errorText = await errorEl.textContent();
      console.log('  ⚠️ Error:', errorText);
    }

    // ============================================
    // Step 3: If still on auth server, check session and OAuth
    // ============================================
    const currentUrl = page.url();

    if (currentUrl.includes('localhost:3001') && !currentUrl.includes('code=')) {
      console.log('\nSTEP 5: Checking session on auth server');

      // Check if user is logged in
      await page.goto('http://localhost:3001/api/auth/get-session');
      await page.waitForTimeout(1000);
      const sessionBody = await page.textContent('body');
      console.log('  Session response:', sessionBody.substring(0, 100));
      await page.screenshot({ path: '/tmp/full-05-session-check.png' });

      if (sessionBody.includes('"user"')) {
        console.log('  ✓ User is logged in!');

        // Now test OAuth authorization directly
        console.log('\nSTEP 6: Testing OAuth authorization');
        const oauthUrl = 'http://localhost:3001/api/auth/oauth2/authorize?' + new URLSearchParams({
          client_id: 'robolearn-interface',
          redirect_uri: 'http://localhost:3000/auth/callback',
          response_type: 'code',
          scope: 'openid profile email',
          state: 'test-state-' + Date.now()
        }).toString();

        await page.goto(oauthUrl);
        await page.waitForTimeout(2000);
        await page.screenshot({ path: '/tmp/full-06-oauth-authorize.png' });
        console.log('  URL after OAuth:', page.url().substring(0, 100));

        const afterOAuthUrl = page.url();
        if (afterOAuthUrl.includes('code=')) {
          console.log('  ✓ SUCCESS: Got authorization code!');
          const urlObj = new URL(afterOAuthUrl);
          const code = urlObj.searchParams.get('code');
          console.log('  Code:', code?.substring(0, 30) + '...');

          // The callback page should exchange this for tokens
          await page.waitForTimeout(3000);
          await page.screenshot({ path: '/tmp/full-07-callback-result.png' });
          console.log('  Final URL:', page.url());
        } else if (afterOAuthUrl.includes('/auth/consent')) {
          console.log('  📋 Consent page shown');
          // Click approve if consent page
          const approveBtn = page.locator('button:has-text("Allow"), button:has-text("Approve")');
          if (await approveBtn.isVisible()) {
            await approveBtn.click();
            await page.waitForTimeout(2000);
          }
        } else if (afterOAuthUrl.includes('/auth/sign-in')) {
          console.log('  ⚠️ Redirected to sign-in (session issue)');
        }
      } else {
        console.log('  ⚠️ No active session');
      }
    } else if (currentUrl.includes('code=')) {
      console.log('\n✅ OAuth flow completed! Got authorization code');
      const urlObj = new URL(currentUrl);
      console.log('  Code:', urlObj.searchParams.get('code')?.substring(0, 30) + '...');
    }

    // ============================================
    // Final: Test Sign-In OAuth Flow
    // ============================================
    console.log('\n=== TESTING SIGN-IN OAUTH FLOW ===');

    // Clear cookies to start fresh
    await context.clearCookies();

    console.log('\nSTEP A: Going to interface and clicking Sign In');
    await page.goto('http://localhost:3000');
    await page.waitForLoadState('networkidle');

    const signInBtn = page.locator('button:has-text("Sign In")').first();
    if (await signInBtn.isVisible()) {
      await signInBtn.click();
      await page.waitForURL(/localhost:3001/, { timeout: 10000 });
    }
    await page.screenshot({ path: '/tmp/full-08-signin-page.png' });
    console.log('  ✓ On sign-in page:', page.url().substring(0, 80));

    console.log('\nSTEP B: Signing in with created account');
    await page.waitForSelector('input#email', { timeout: 5000 });
    await page.fill('input#email', testEmail);
    await page.fill('input#password', testPassword);
    await page.screenshot({ path: '/tmp/full-09-signin-filled.png' });

    await page.click('button[type="submit"]');

    // Wait for OAuth redirect with code
    try {
      await page.waitForURL(/code=|localhost:3000/, { timeout: 15000 });
    } catch (e) {
      await page.waitForTimeout(3000);
    }

    await page.screenshot({ path: '/tmp/full-10-after-signin.png' });
    console.log('  Final URL:', page.url().substring(0, 100));

    const finalUrl = page.url();
    if (finalUrl.includes('code=')) {
      console.log('\n✅ SIGN-IN OAUTH FLOW SUCCESS!');
      const urlObj = new URL(finalUrl);
      console.log('  Code:', urlObj.searchParams.get('code')?.substring(0, 30) + '...');
    } else if (finalUrl.includes('localhost:3000') && !finalUrl.includes('/auth/')) {
      console.log('\n✅ Redirected to interface (might have tokens in localStorage)');
    }

    console.log('\n=== ALL TESTS COMPLETE ===');
    console.log('\nScreenshots saved to /tmp/full-*.png');

  } catch (error) {
    console.error('\n❌ Test error:', error.message);
    await page.screenshot({ path: '/tmp/full-error.png' });
  }

  console.log('\nBrowser closing in 5 seconds...');
  await page.waitForTimeout(5000);
  await browser.close();
}

testFullOAuthFlow().catch(console.error);
