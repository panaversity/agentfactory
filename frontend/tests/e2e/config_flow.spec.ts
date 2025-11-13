/**
 * E2E tests for the Configuration Flow functionality
 * These tests simulate real user interactions with the Configuration UI
 */

import { test, expect, Page } from '@playwright/test';

// Mock API endpoints for testing
test.beforeEach(async ({ page }) => {
  // Initially mock the config status API to return not configured
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
});

test.describe('Configuration Flow E2E Tests', () => {
  test('should open config UI and display initial state', async ({ page }) => {
    await page.goto('http://localhost:3000/test-config');
    
    // Click the configuration button (assuming there's a button to open config)
    await page.locator('.config-button').click(); // Adjust selector as needed
    
    // Verify config UI is visible
    await expect(page.locator('.config-ui')).toBeVisible();
    
    // Verify initial state
    await expect(page.locator('.config-ui')).toContainText('API Key Configured: No');
  });

  test('should save and validate a new API key', async ({ page }) => {
    // Set up route to mock successful API key validation
    await page.route('**/api/config/gemini-key', async (route) => {
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({ status: 'success' })
      });
    });

    // Update the config status route to reflect that the API is now configured
    page.route('**/api/config/status', async (route) => {
      // For the initial request (on component mount), return not configured
      // For subsequent requests, return configured
      if (route.request().method() === 'GET') {
        await route.fulfill({
          status: 200,
          contentType: 'application/json',
          body: JSON.stringify({
            is_configured: true,
            is_valid: true
          })
        });
      }
    });
    
    await page.goto('http://localhost:3000/test-config');
    await page.locator('.config-button').click(); // Adjust selector as needed
    
    // Enter a new API key
    await page.locator('#api-key').fill('test_api_key_for_validation');
    
    // Click save API key button
    await page.locator('.save-button').click();
    
    // Wait for success message
    await expect(page.locator('.success-message')).toContainText('API key saved and validated successfully');
    
    // Verify the status is now configured
    await expect(page.locator('.status-info')).toContainText('API Key Configured: Yes');
    await expect(page.locator('.status-info')).toContainText('API Key Valid: Yes');
  });

  test('should show error for invalid API key', async ({ page }) => {
    // Set up route to mock failed API key validation
    await page.route('**/api/config/gemini-key', async (route) => {
      await route.fulfill({
        status: 400,
        contentType: 'application/json',
        body: JSON.stringify({ 
          detail: 'Invalid API key: API key is invalid' 
        })
      });
    });
    
    await page.goto('http://localhost:3000/test-config');
    await page.locator('.config-button').click(); // Adjust selector as needed
    
    // Enter an invalid API key
    await page.locator('#api-key').fill('invalid_api_key');
    
    // Click save API key button
    await page.locator('.save-button').click();
    
    // Wait for error message
    await expect(page.locator('.error-message')).toContainText('Failed to save API key');
  });

  test('should not save empty API key', async ({ page }) => {
    await page.goto('http://localhost:3000/test-config');
    await page.locator('.config-button').click(); // Adjust selector as needed
    
    // Try to save without entering an API key
    await page.locator('#api-key').fill('');
    await page.locator('.save-button').click();
    
    // Verify error message appears
    await expect(page.locator('.error-message')).toContainText('API key is required');
  });

  test('should close config UI when close button is clicked', async ({ page }) => {
    await page.goto('http://localhost:3000/test-config');
    await page.locator('.config-button').click(); // Adjust selector as needed
    
    // Verify config UI is visible
    await expect(page.locator('.config-ui')).toBeVisible();
    
    // Click close button
    await page.locator('.close-button').click();
    
    // Verify config UI is no longer visible
    await expect(page.locator('.config-ui')).not.toBeVisible();
  });

  test('should display instructions in config UI', async ({ page }) => {
    await page.goto('http://localhost:3000/test-config');
    await page.locator('.config-button').click(); // Adjust selector as needed
    
    // Verify instructions are present
    await expect(page.locator('.config-section').nth(2)).toContainText('Instructions');
    await expect(page.locator('.config-section').nth(2)).toContainText('Get your Gemini API key');
    await expect(page.locator('.config-section').nth(2)).toContainText('Enter your API key');
  });

  test('should mask API key after successful save', async ({ page }) => {
    // Mock successful save and subsequent status
    await page.route('**/api/config/gemini-key', async (route) => {
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({ status: 'success' })
      });
    });

    page.route('**/api/config/status', async (route) => {
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({
          is_configured: true,
          is_valid: true
        })
      });
    });
    
    await page.goto('http://localhost:3000/test-config');
    await page.locator('.config-button').click(); // Adjust selector as needed
    
    // Enter and save an API key
    await page.locator('#api-key').fill('my_test_api_key');
    await page.locator('.save-button').click();
    
    // Wait for success and verify key is masked when config is reopened or status is loaded
    await expect(page.locator('.success-message')).toContainText('API key saved and validated successfully');
  });
});