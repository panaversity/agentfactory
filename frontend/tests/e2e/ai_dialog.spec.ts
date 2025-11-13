/**
 * E2E tests for the AI Dialog functionality
 * These tests simulate real user interactions with the AI Dialog
 */

import { test, expect, Page } from '@playwright/test';

// Mock API endpoints for testing
test.beforeEach(async ({ page }) => {
  // Mock the config status API to return configured and valid
  await page.route('**/api/config/status', async (route) => {
    await route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({
        is_configured: true,
        is_valid: true
      })
    });
  });

  // Mock the AI query API
  await page.route('**/api/ai/query', async (route) => {
    await route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({
        response: 'This is a test response from the AI service.'
      })
    });
  });
});

test.describe('AI Dialog E2E Tests', () => {
  test('should open AI dialog when text is highlighted and ask AI button is available', async ({ page }) => {
    // Navigate to a test page
    await page.goto('http://localhost:3000/test-page'); // Adjust URL as needed
    
    // Create some content to highlight
    await page.setContent(`
      <div id="content">
        This is some sample text that can be highlighted for testing purposes.
        It contains multiple words that a user might want to get AI insights about.
      </div>
    `);
    
    // Select a portion of text
    await page.locator('#content').click();
    await page.keyboard.press('ArrowRight');
    await page.keyboard.press('ArrowRight');
    await page.keyboard.press('Shift+ArrowRight');
    await page.keyboard.press('Shift+ArrowRight');
    await page.keyboard.press('Shift+ArrowRight');
    
    // Wait for AI dialog to appear
    await expect(page.locator('.ai-dialog')).toBeVisible();
    
    // Verify the highlighted text appears in the dialog
    const dialog = page.locator('.ai-dialog');
    await expect(dialog).toContainText('sample text');
  });

  test('should submit query and display AI response', async ({ page }) => {
    // Since we can't easily simulate text selection in a test,
    // we'll navigate directly to a page where the AI dialog is already open
    await page.goto('http://localhost:3000/test-ai-dialog');
    
    // Fill in a query
    await page.locator('.query-input').fill('Explain this concept');
    
    // Submit the query
    await page.locator('.submit-button').click();
    
    // Wait for response
    await expect(page.locator('.response-content')).toContainText('This is a test response from the AI service.');
  });

  test('should handle error when AI service fails', async ({ page }) => {
    // Mock a failure response from the AI service
    await page.route('**/api/ai/query', async (route) => {
      await route.fulfill({
        status: 500,
        contentType: 'application/json',
        body: JSON.stringify({
          detail: 'AI service temporarily unavailable'
        })
      });
    });
    
    await page.goto('http://localhost:3000/test-ai-dialog');
    
    // Fill in a query
    await page.locator('.query-input').fill('Test query');
    
    // Submit the query
    await page.locator('.submit-button').click();
    
    // Wait for error message
    await expect(page.locator('.error-message')).toContainText('Error getting AI response');
  });

  test('should disable submit button when AI is not configured', async ({ page }) => {
    // Mock unconfigured AI service
    await page.route('**/api/config/status', async (route) => {
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({
          is_configured: false,
          is_valid: null
        })
      });
    });
    
    await page.goto('http://localhost:3000/test-ai-dialog');
    
    // Verify submit button is disabled
    await expect(page.locator('.submit-button')).toBeDisabled();
    
    // Verify warning message is shown
    await expect(page.locator('.config-warning')).toContainText('AI is not configured');
  });

  test('should close AI dialog when close button is clicked', async ({ page }) => {
    await page.goto('http://localhost:3000/test-ai-dialog');
    
    // Verify dialog is visible
    await expect(page.locator('.ai-dialog')).toBeVisible();
    
    // Click close button
    await page.locator('.close-button').click();
    
    // Verify dialog is no longer visible
    await expect(page.locator('.ai-dialog')).not.toBeVisible();
  });
});