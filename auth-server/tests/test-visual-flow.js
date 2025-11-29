const { chromium } = require('playwright');

async function testAuthFlow() {
  const browser = await chromium.launch({ headless: true });
  const context = await browser.newContext();
  const page = await context.newPage();

  console.log('\n=== VISUAL AUTH FLOW TEST ===\n');

  // 1. Homepage - Check "Get Started" button
  console.log('1. Testing Homepage (http://localhost:3000)...');
  await page.goto('http://localhost:3000');
  await page.waitForLoadState('networkidle');
  await page.screenshot({ path: '/tmp/01-homepage.png', fullPage: false });
  console.log('   Screenshot: /tmp/01-homepage.png');

  // Check if Get Started button exists
  const getStartedBtn = await page.$('a[href*="auth/sign-up"]');
  console.log('   "Get Started" button found:', !!getStartedBtn);

  // 2. Click Get Started - Should go to auth server
  console.log('\n2. Clicking "Get Started" -> Sign Up page...');
  await page.click('a[href*="auth/sign-up"]');
  await page.waitForLoadState('networkidle');
  await page.screenshot({ path: '/tmp/02-signup-page.png', fullPage: false });
  console.log('   Current URL:', page.url());
  console.log('   Screenshot: /tmp/02-signup-page.png');

  // 3. Fill sign-up form
  console.log('\n3. Filling sign-up form...');
  const testEmail = `visual-test-${Date.now()}@robolearn.io`;
  await page.fill('input[type="email"]', testEmail);
  await page.fill('input[id="password"]', 'TestPassword123');
  await page.fill('input[id="confirmPassword"]', 'TestPassword123');
  await page.fill('input[id="name"]', 'Visual Test User');

  // Select intermediate background
  await page.click('input[value="intermediate"]');
  await page.screenshot({ path: '/tmp/03-signup-filled.png', fullPage: false });
  console.log('   Form filled with:', testEmail);
  console.log('   Screenshot: /tmp/03-signup-filled.png');

  // 4. Submit sign-up
  console.log('\n4. Submitting sign-up...');
  await page.click('button[type="submit"]');
  await page.waitForTimeout(3000); // Wait for redirect
  await page.screenshot({ path: '/tmp/04-after-signup.png', fullPage: false });
  console.log('   Current URL after signup:', page.url());
  console.log('   Screenshot: /tmp/04-after-signup.png');

  // 5. Go to sign-in page
  console.log('\n5. Testing Sign-in page...');
  await page.goto('http://localhost:3001/auth/sign-in');
  await page.waitForLoadState('networkidle');
  await page.screenshot({ path: '/tmp/05-signin-page.png', fullPage: false });
  console.log('   Screenshot: /tmp/05-signin-page.png');

  // 6. Fill and submit sign-in
  console.log('\n6. Filling sign-in form...');
  await page.fill('input[type="email"]', testEmail);
  await page.fill('input[type="password"]', 'TestPassword123');
  await page.screenshot({ path: '/tmp/06-signin-filled.png', fullPage: false });
  console.log('   Screenshot: /tmp/06-signin-filled.png');

  console.log('\n7. Submitting sign-in...');
  await page.click('button[type="submit"]');
  await page.waitForTimeout(3000);
  await page.screenshot({ path: '/tmp/07-after-signin.png', fullPage: false });
  console.log('   Current URL after signin:', page.url());
  console.log('   Screenshot: /tmp/07-after-signin.png');

  await browser.close();

  console.log('\n=== TEST COMPLETE ===');
  console.log('\nScreenshots saved to /tmp/');
  console.log('View them with: open /tmp/01-homepage.png');
}

testAuthFlow().catch(console.error);
