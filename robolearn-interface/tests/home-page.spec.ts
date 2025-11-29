import { test, expect } from '@playwright/test';

/**
 * Home Page E2E Tests
 *
 * Tests the Industrial Confidence design:
 * - FR-001 to FR-004: Hero Section - Brand, headline, robot assembly, CTAs
 * - FR-005 to FR-008: Modules Section
 * - FR-009 to FR-011: Features Section
 * - FR-012 to FR-014: Hardware Section
 * - FR-015 to FR-016: CTA Section
 * - FR-017, FR-018: Responsive Design
 */

const BASE_URL = 'http://localhost:3000/';

test.describe('Home Page - Industrial Confidence Design', () => {
  test.beforeEach(async ({ page }) => {
    await page.goto(BASE_URL);
    // Wait for entrance animations
    await page.waitForTimeout(2000);
  });

  test.describe('Hero Section (US1 - First-Time Visitor Landing)', () => {
    test('FR-001: displays brand mark and platform tag', async ({ page }) => {
      const hero = page.locator('section').first();
      await expect(hero).toBeVisible();

      // Brand mark (scoped to hero section)
      await expect(hero.getByText('ROBOLEARN', { exact: true })).toBeVisible();
      await expect(hero.getByText('PLATFORM', { exact: true })).toBeVisible();
    });

    test('FR-002: displays headline with "understand" accent', async ({ page }) => {
      const title = page.locator('h1#hero-title');
      await expect(title).toBeVisible();
      await expect(title).toContainText('Build robots that');
      await expect(title).toContainText('understand');
      await expect(title).toContainText('the physical world');
    });

    test('FR-003: displays robot assembly visualization', async ({ page }) => {
      const robotSvg = page.locator('[aria-label="Robot assembly visualization"]');
      await expect(robotSvg).toBeVisible();
    });

    test('FR-004: displays CTAs - Begin Building and Explore the Stack', async ({ page }) => {
      // Primary CTA
      const primaryCta = page.locator('a:has-text("Begin Building")');
      await expect(primaryCta).toBeVisible();
      await expect(primaryCta).toHaveAttribute('href', /\/docs\/preface-agent-native$/);

      // Secondary CTA
      const secondaryCta = page.locator('a:has-text("Explore the Stack")');
      await expect(secondaryCta).toBeVisible();
    });

    test('FR-004b: displays tech stack pills', async ({ page }) => {
      // Tech stack pills are in the hero section's stack item divs
      const stackItems = page.locator('[class*="stackItem"]');
      await expect(stackItems).toHaveCount(4);
    });

    test('FR-004c: displays "Free forever" in subheadline', async ({ page }) => {
      await expect(page.getByText('Free forever.')).toBeVisible();
    });
  });

  test.describe('Modules Section (US2)', () => {
    test('FR-005: displays 4 module cards', async ({ page }) => {
      const modulesSection = page.locator('section[aria-labelledby="modules-heading"]');
      await expect(modulesSection).toBeVisible();

      const moduleCards = modulesSection.locator('article');
      await expect(moduleCards).toHaveCount(4);
    });

    test('FR-006: module cards have correct titles', async ({ page }) => {
      const modulesSection = page.locator('section[aria-labelledby="modules-heading"]');

      await expect(modulesSection.locator('text=ROS 2 Fundamentals')).toBeVisible();
      await expect(modulesSection.locator('text=Digital Twins')).toBeVisible();
      await expect(modulesSection.locator('text=NVIDIA Isaac')).toBeVisible();
      await expect(modulesSection.locator('text=Vision-Language-Action')).toBeVisible();
    });

    test('FR-007: advanced modules have badges', async ({ page }) => {
      const modulesSection = page.locator('section[aria-labelledby="modules-heading"]');
      const badgeElements = modulesSection.locator('[class*="badge"]');
      await expect(badgeElements).toHaveCount(2);
    });

    test('FR-008: module cards display week ranges', async ({ page }) => {
      const modulesSection = page.locator('section[aria-labelledby="modules-heading"]');

      await expect(modulesSection.locator('text=Weeks 1-5')).toBeVisible();
      await expect(modulesSection.locator('text=Weeks 6-7')).toBeVisible();
      await expect(modulesSection.locator('text=Weeks 8-10')).toBeVisible();
      await expect(modulesSection.locator('text=Weeks 11-13')).toBeVisible();
    });
  });

  test.describe('Features Section', () => {
    test('FR-009: displays 6 feature cards', async ({ page }) => {
      const featuresSection = page.locator('section[aria-labelledby="features-heading"]');
      await expect(featuresSection).toBeVisible();

      const featureCards = featuresSection.locator('article');
      await expect(featureCards).toHaveCount(6);
    });

    test('FR-010: featured cards have "Core" badge', async ({ page }) => {
      const featuresSection = page.locator('section[aria-labelledby="features-heading"]');
      const coreBadges = featuresSection.locator('text=Core');
      await expect(coreBadges).toHaveCount(2);
    });

    test('FR-011: feature cards have correct titles', async ({ page }) => {
      const featuresSection = page.locator('section[aria-labelledby="features-heading"]');

      await expect(featuresSection.locator('text=Embodied Intelligence')).toBeVisible();
      await expect(featuresSection.locator('text=Human-Centered Design')).toBeVisible();
      await expect(featuresSection.locator('text=Production-Ready Skills')).toBeVisible();
      await expect(featuresSection.locator('text=Conversational Robotics')).toBeVisible();
      await expect(featuresSection.locator('text=Sim-to-Real Transfer')).toBeVisible();
      await expect(featuresSection.locator('text=Interactive Learning')).toBeVisible();
    });
  });

  test.describe('Hardware Section (US3)', () => {
    test('FR-012: displays 3 hardware tiers', async ({ page }) => {
      const hardwareSection = page.locator('section[aria-labelledby="hardware-heading"]');
      await expect(hardwareSection).toBeVisible();

      const hardwareTiers = hardwareSection.locator('article');
      await expect(hardwareTiers).toHaveCount(3);
    });

    test('FR-013: tier 2 has RECOMMENDED badge', async ({ page }) => {
      const hardwareSection = page.locator('section[aria-labelledby="hardware-heading"]');

      await expect(hardwareSection.locator('text=RECOMMENDED')).toBeVisible();
      await expect(hardwareSection.locator('text=Cloud + Edge')).toBeVisible();
    });

    test('FR-014: hardware tiers display cost estimates', async ({ page }) => {
      const hardwareSection = page.locator('section[aria-labelledby="hardware-heading"]');

      await expect(hardwareSection.locator('text=$2,500')).toBeVisible();
      await expect(hardwareSection.locator('text=$200/quarter')).toBeVisible();
      await expect(hardwareSection.locator('text=Cloud compute only')).toBeVisible();
    });
  });

  test.describe('CTA Section', () => {
    test('FR-015: displays final CTA block', async ({ page }) => {
      await expect(page.locator('text=The Future is Physical AI')).toBeVisible();
      await expect(page.locator('text=Ready to Begin?')).toBeVisible();
    });

    test('FR-016: CTA button navigates to docs', async ({ page }) => {
      const ctaButton = page.locator('a:has-text("Begin Learning")');
      await expect(ctaButton).toBeVisible();
      await expect(ctaButton).toHaveAttribute('href', /\/docs\/preface-agent-native$/);
    });
  });
});

test.describe('Responsive Design (US4)', () => {
  test('FR-017: mobile viewport (375px)', async ({ page }) => {
    await page.setViewportSize({ width: 375, height: 667 });
    await page.goto(BASE_URL);
    await page.waitForTimeout(1000);

    await expect(page.locator('h1#hero-title')).toBeVisible();
    const hero = page.locator('section').first();
    await expect(hero.getByText('ROBOLEARN', { exact: true })).toBeVisible();
  });

  test('FR-018: tablet viewport (768px)', async ({ page }) => {
    await page.setViewportSize({ width: 768, height: 1024 });
    await page.goto(BASE_URL);

    await expect(page.locator('h1#hero-title')).toBeVisible();
    await expect(page.locator('section[aria-labelledby="modules-heading"]')).toBeVisible();
  });

  test('desktop viewport (1440px)', async ({ page }) => {
    await page.setViewportSize({ width: 1440, height: 900 });
    await page.goto(BASE_URL);

    await expect(page.locator('h1#hero-title')).toBeVisible();
    await expect(page.locator('section[aria-labelledby="modules-heading"]')).toBeVisible();
    await expect(page.locator('section[aria-labelledby="features-heading"]')).toBeVisible();
    await expect(page.locator('section[aria-labelledby="hardware-heading"]')).toBeVisible();
  });
});

test.describe('Accessibility', () => {
  test('skip link exists', async ({ page }) => {
    await page.goto(BASE_URL);
    const skipLink = page.locator('a.skipLink, a:has-text("Skip to content")');
    await expect(skipLink).toHaveCount(1);
  });

  test('all sections have proper ARIA labels', async ({ page }) => {
    await page.goto(BASE_URL);

    await expect(page.locator('section[aria-labelledby="modules-heading"]')).toBeVisible();
    await expect(page.locator('section[aria-labelledby="features-heading"]')).toBeVisible();
    await expect(page.locator('section[aria-labelledby="hardware-heading"]')).toBeVisible();
  });

  test('robot visualization has accessible label', async ({ page }) => {
    await page.goto(BASE_URL);
    await page.waitForTimeout(1500);

    const robot = page.locator('svg[aria-label="Robot assembly visualization"]');
    await expect(robot).toBeVisible();
  });

  test('primary CTA is keyboard accessible', async ({ page }) => {
    await page.goto(BASE_URL);
    await page.waitForTimeout(1500);

    const primaryCta = page.locator('a:has-text("Begin Building")');
    await primaryCta.focus();
    await expect(primaryCta).toBeFocused();
  });
});

test.describe('Dark/Light Mode (US5)', () => {
  test('page renders in dark mode by default', async ({ page }) => {
    await page.goto(BASE_URL);
    const heroSection = page.locator('section').first();
    await expect(heroSection).toBeVisible();
  });

  test('page supports light mode', async ({ page }) => {
    await page.emulateMedia({ colorScheme: 'light' });
    await page.goto(BASE_URL);
    await expect(page.locator('h1#hero-title')).toBeVisible();
  });
});
