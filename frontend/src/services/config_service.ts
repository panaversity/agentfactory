/**
 * Service for handling configuration-related frontend operations.
 */

import AIService, { ConfigRequest, ConfigStatusResponse } from './ai_service';

class ConfigService {
  /**
   * Save the Gemini API key to the backend.
   * @param request The configuration request with API key
   * @returns Promise resolving to success status
   */
  async saveAPIKey(request: ConfigRequest): Promise<{ status: string }> {
    return AIService.configureAPIKey(request);
  }

  /**
   * Get the current configuration status from the backend.
   * @returns Promise resolving to configuration status
   */
  async getConfigStatus(): Promise<ConfigStatusResponse> {
    return AIService.getConfigStatus();
  }

  /**
   * Validate the format of an API key (basic validation).
   * @param apiKey The API key to validate
   * @returns True if the key format is valid, false otherwise
   */
  validateAPIKeyFormat(apiKey: string): boolean {
    // Basic format validation: Gemini API keys are typically long strings
    // starting with "AI" followed by letters and numbers
    if (!apiKey || typeof apiKey !== 'string') {
      return false;
    }

    // Check length (Gemini keys are typically 39+ characters)
    if (apiKey.length < 30) {
      return false;
    }

    // For basic validation, we just check if it's reasonably long
    // The real validation happens on the backend with an actual API call
    return true;
  }
}

export default new ConfigService();