/**
 * Service for handling AI-related API calls.
 */

export interface AIQueryRequest {
  highlighted_text?: string;
  query: string;
}

export interface AIQueryResponse {
  response: string;
}

export interface ConfigRequest {
  api_key: string;
}

export interface ConfigStatusResponse {
  is_configured: boolean;
  is_valid?: boolean;
  message?: string;
}

class AIService {
  private baseUrl: string;

  constructor(baseUrl: string = 'http://localhost:8000') {
    this.baseUrl = baseUrl;
  }

  /**
   * Send a query to the AI backend service.
   * @param request The query request with optional highlighted text
   * @returns Promise resolving to the AI response
   */
  async queryAI(request: AIQueryRequest): Promise<AIQueryResponse> {
    try {
      const response = await fetch(`${this.baseUrl}/api/ai/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          highlighted_text: request.highlighted_text,
          query: request.query
        })
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data as AIQueryResponse;
    } catch (error) {
      console.error('Error querying AI:', error);
      throw error;
    }
  }

  /**
   * Configure the Gemini API key on the backend.
   * @param request The configuration request with API key
   * @returns Promise resolving to success status
   */
  async configureAPIKey(request: ConfigRequest): Promise<{ status: string }> {
    try {
      const response = await fetch(`${this.baseUrl}/api/config/gemini-key`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          api_key: request.api_key
        })
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error configuring API key:', error);
      throw error;
    }
  }

  /**
   * Get the current configuration status.
   * @returns Promise resolving to configuration status
   */
  async getConfigStatus(): Promise<ConfigStatusResponse> {
    try {
      const response = await fetch(`${this.baseUrl}/api/config/status`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        }
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data as ConfigStatusResponse;
    } catch (error) {
      console.error('Error getting config status:', error);
      throw error;
    }
  }

  /**
   * Test the AI service by checking if the backend is reachable.
   * @returns Promise resolving to true if service is reachable
   */
  async testConnection(): Promise<boolean> {
    try {
      const response = await fetch(`${this.baseUrl}/health`);
      return response.ok;
    } catch (error) {
      console.error('Error testing connection:', error);
      return false;
    }
  }
}

export default new AIService();