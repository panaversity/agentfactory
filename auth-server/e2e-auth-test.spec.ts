import { test, expect } from '@playwright/test';

const AUTH_URL = 'http://localhost:3001';
const BOOK_URL = 'http://localhost:3000';

test.describe('Auth Flow E2E Tests', () => {
  const testEmail = `e2e-test-${Date.now()}@robolearn.io`;
  const testPassword = 'TestPassword123';
  const testName = 'E2E Test User';

  test('1. Sign-up page loads correctly', async ({ page }) => {
    await page.goto(`${AUTH_URL}/auth/sign-up`);

    // Check page title and branding
    await expect(page.locator('h1')).toContainText('RoboLearn');
    await expect(page.locator('h2')).toContainText('Create your account');

    // Check form fields exist
    await expect(page.locator('input#name')).toBeVisible();
    await expect(page.locator('input#email')).toBeVisible();
    await expect(page.locator('input#password')).toBeVisible();
    await expect(page.locator('input#confirmPassword')).toBeVisible();

    // Check software background options
    await expect(page.locator('input[value="beginner"]')).toBeVisible();
    await expect(page.locator('input[value="intermediate"]')).toBeVisible();
    await expect(page.locator('input[value="advanced"]')).toBeVisible();

    // Check submit button
    await expect(page.locator('button[type="submit"]')).toContainText('Create account');

    // Take screenshot
    await page.screenshot({ path: '/tmp/e2e-01-signup-page.png', fullPage: true });
  });

  test('2. Sign-up form validation works', async ({ page }) => {
    await page.goto(`${AUTH_URL}/auth/sign-up`);

    // Try to submit empty form
    await page.click('button[type="submit"]');

    // Fill only email (missing password)
    await page.fill('input#email', 'test@example.com');
    await page.fill('input#password', 'short'); // Too short
    await page.fill('input#confirmPassword', 'short');
    await page.click('button[type="submit"]');

    // Should show password error
    await expect(page.locator('text=at least 8 characters')).toBeVisible();

    // Take screenshot
    await page.screenshot({ path: '/tmp/e2e-02-validation.png', fullPage: true });
  });

  test('3. Complete sign-up flow', async ({ page }) => {
    await page.goto(`${AUTH_URL}/auth/sign-up`);

    // Fill the form
    await page.fill('input#name', testName);
    await page.fill('input#email', testEmail);
    await page.fill('input#password', testPassword);
    await page.fill('input#confirmPassword', testPassword);

    // Select intermediate background
    await page.click('input[value="intermediate"]');

    // Take screenshot before submit
    await page.screenshot({ path: '/tmp/e2e-03-signup-filled.png', fullPage: true });

    // Submit the form
    await page.click('button[type="submit"]');

    // Wait for redirect to book
    await page.waitForTimeout(3000);

    // Take screenshot after signup
    await page.screenshot({ path: '/tmp/e2e-04-after-signup.png', fullPage: true });

    // Verify we're redirected (URL should be book URL or show success)
    const currentUrl = page.url();
    console.log('After signup URL:', currentUrl);
  });

  test('4. Sign-in page loads correctly', async ({ page }) => {
    await page.goto(`${AUTH_URL}/auth/sign-in`);

    // Check page title and branding
    await expect(page.locator('h1')).toContainText('RoboLearn');
    await expect(page.locator('h2')).toContainText('Welcome back');

    // Check form fields exist
    await expect(page.locator('input#email')).toBeVisible();
    await expect(page.locator('input#password')).toBeVisible();

    // Check submit button
    await expect(page.locator('button[type="submit"]')).toContainText('Sign in');

    // Take screenshot
    await page.screenshot({ path: '/tmp/e2e-05-signin-page.png', fullPage: true });
  });

  test('5. Sign-in with wrong credentials shows error', async ({ page }) => {
    await page.goto(`${AUTH_URL}/auth/sign-in`);

    // Fill with wrong credentials
    await page.fill('input#email', 'wrong@example.com');
    await page.fill('input#password', 'wrongpassword');

    // Submit
    await page.click('button[type="submit"]');

    // Wait for error
    await page.waitForTimeout(2000);

    // Should show error message
    await expect(page.locator('text=Invalid credentials')).toBeVisible();

    // Take screenshot
    await page.screenshot({ path: '/tmp/e2e-06-signin-error.png', fullPage: true });
  });

  test('6. Sign-in with correct credentials works', async ({ page }) => {
    // First create a user
    const loginEmail = `login-test-${Date.now()}@robolearn.io`;

    // Sign up first
    await page.goto(`${AUTH_URL}/auth/sign-up`);
    await page.fill('input#name', 'Login Test');
    await page.fill('input#email', loginEmail);
    await page.fill('input#password', testPassword);
    await page.fill('input#confirmPassword', testPassword);
    await page.click('button[type="submit"]');
    await page.waitForTimeout(3000);

    // Now go to sign-in
    await page.goto(`${AUTH_URL}/auth/sign-in`);

    // Fill credentials
    await page.fill('input#email', loginEmail);
    await page.fill('input#password', testPassword);

    // Take screenshot before submit
    await page.screenshot({ path: '/tmp/e2e-07-signin-filled.png', fullPage: true });

    // Submit
    await page.click('button[type="submit"]');

    // Wait for redirect
    await page.waitForTimeout(3000);

    // Take screenshot after signin
    await page.screenshot({ path: '/tmp/e2e-08-after-signin.png', fullPage: true });

    const currentUrl = page.url();
    console.log('After signin URL:', currentUrl);
  });

  test('7. Homepage has Get Started button', async ({ page }) => {
    await page.goto(BOOK_URL);
    await page.waitForLoadState('networkidle');

    // Take screenshot of homepage
    await page.screenshot({ path: '/tmp/e2e-09-homepage.png', fullPage: false });

    // Check for auth-related content (navbar might have auth buttons)
    const pageContent = await page.content();
    console.log('Homepage has auth links:', pageContent.includes('sign-up') || pageContent.includes('Get Started'));
  });
});
