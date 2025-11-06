/**
 * Gemini Service - Base Client
 * Google Gemini 2.0 Flash API integration with error handling and rate limiting
 */

import { GoogleGenerativeAI, type GenerativeModel } from '@google/generative-ai';
import type { GeminiError } from '../types';
import { globalRateLimiter } from './rateLimiter';
import { errorLogger } from './errorLogger';

export class GeminiService {
  private genAI: GoogleGenerativeAI | null = null;
  private model: GenerativeModel | null = null;
  private readonly modelName: string = 'gemini-2.0-flash-exp';

  constructor() {
    this.initialize();
  }

  /**
   * Initialize Gemini API client
   */
  private initialize(): void {
    try {
      // Get API key from window object or use hardcoded development key
      // @ts-ignore - Custom window property
      const apiKey = (typeof window !== 'undefined' && window.GEMINI_API_KEY) || 
                     'AIzaSyCZPNyTTr_p2HIi4LhEZoNJf1K9ExSHF-Y'; // Development key
      
      if (!apiKey) {
        console.warn('[GeminiService] GEMINI_API_KEY not found');
        return;
      }

      this.genAI = new GoogleGenerativeAI(apiKey);
      this.model = this.genAI.getGenerativeModel({ model: this.modelName });
      
      console.log('[GeminiService] Initialized successfully');
    } catch (error) {
      errorLogger.logError(error as Error, { context: 'GeminiService initialization' });
      throw error;
    }
  }

  /**
   * Check if service is ready
   */
  public isReady(): boolean {
    return this.model !== null;
  }

  /**
   * Get the GenerativeModel instance
   * @throws Error if not initialized
   */
  protected getModel(): GenerativeModel {
    if (!this.model) {
      throw this.createError(
        'SERVICE_NOT_INITIALIZED',
        'Gemini API not initialized. Check GEMINI_API_KEY.',
        false
      );
    }
    return this.model;
  }

  /**
   * Execute request with rate limiting
   * @param fn Function to execute
   * @returns Result of function
   */
  protected async withRateLimit<T>(fn: () => Promise<T>): Promise<T> {
    // Check rate limit
    const allowed = await globalRateLimiter.tryRequest();
    
    if (!allowed) {
      const status = globalRateLimiter.getStatus();
      const waitMs = status.resetAt - Date.now();
      
      throw this.createError(
        'RATE_LIMIT_EXCEEDED',
        `Rate limit exceeded. Try again in ${Math.ceil(waitMs / 1000)} seconds.`,
        true
      );
    }

    try {
      return await fn();
    } catch (error) {
      throw this.handleApiError(error);
    }
  }

  /**
   * Handle API errors and convert to GeminiError
   */
  private handleApiError(error: any): GeminiError {
    errorLogger.logError(error, { context: 'Gemini API request' });

    // Check for specific error types
    if (error.message?.includes('quota') || error.message?.includes('rate limit')) {
      return this.createError('RATE_LIMIT_EXCEEDED', error.message, true);
    }

    if (error.message?.includes('API key')) {
      return this.createError('INVALID_API_KEY', 'Invalid API key', false);
    }

    if (error.message?.includes('network') || error.message?.includes('timeout')) {
      return this.createError('NETWORK_ERROR', 'Network error occurred', true);
    }

    // Generic error
    return this.createError('API_ERROR', error.message || 'Unknown error', false);
  }

  /**
   * Create structured error object
   */
  protected createError(code: string, message: string, retryable: boolean): GeminiError {
    return {
      code,
      message,
      retryable,
    };
  }

  /**
   * Generate prompt for specific operation
   * Subclasses can override for specific use cases
   */
  protected generateSystemPrompt(operation: string): string {
    const basePrompt = `You are an AI learning assistant integrated into a technical book platform.
Your role is to help readers understand complex concepts, answer questions, and enhance their learning experience.

Context: The user is reading educational content about AI-native software development.
Current operation: ${operation}

Guidelines:
- Provide clear, concise, and accurate information
- Reference the page content when relevant
- Use simple language for explanations
- Be encouraging and supportive
- If unsure, acknowledge limitations
- Keep responses focused and on-topic`;

    return basePrompt;
  }

  /**
   * Format content for API request
   * Truncates if exceeds reasonable limit
   */
  protected formatContent(content: string, maxLength: number = 8000): string {
    if (content.length <= maxLength) {
      return content;
    }

    // Truncate with ellipsis
    const truncated = content.substring(0, maxLength - 100);
    return `${truncated}\n\n[... content truncated for length ...]`;
  }

  /**
   * Extract text from API response
   */
  protected extractText(response: any): string {
    try {
      return response.response.text();
    } catch (error) {
      errorLogger.logError(error as Error, { context: 'Extract response text' });
      throw this.createError('RESPONSE_PARSE_ERROR', 'Failed to parse API response', false);
    }
  }

  /**
   * Explain highlighted text with caching (30-day TTL)
   * Per gemini-explain.contract.md
   */
  public async explainText(
    selectedText: string,
    surroundingContext: string,
    pageTitle: string
  ): Promise<string> {
    try {
      console.log('[GeminiService] explainText called');
      
      return await this.withRateLimit(async () => {
        console.log('[GeminiService] Inside rate limiter');
        
        const model = this.getModel();
        console.log('[GeminiService] Model retrieved:', !!model);

        // Check cache first
        const { cacheService } = await import('./cacheService');
        const { computeHash } = await import('../utils/hash');
        const textHash = await computeHash(selectedText);
        const cacheKey = `explain_${textHash}`;
        const cached = cacheService.get(cacheKey);

        if (cached) {
          console.log('[GeminiService] Explanation cache hit');
          return cached;
        }

        console.log('[GeminiService] No cache, generating new explanation');

        // Build prompt
        const prompt = `You are explaining a highlighted passage from a technical book to help a reader understand it better.

**Page Context**: ${pageTitle}

**Highlighted Text**:
"${selectedText}"

**Surrounding Context** (for reference):
${surroundingContext}

**Instructions**:
1. Provide a clear, simple explanation of the highlighted text
2. Break down any technical terms or complex concepts
3. Use analogies or examples if helpful
4. Keep the explanation concise (2-3 paragraphs maximum)
5. Reference the surrounding context if it helps clarify meaning

Provide a helpful explanation that enhances the reader's understanding:`;

        console.log('[GeminiService] Calling model.generateContent...');

        const result = await model.generateContent(prompt);
        console.log('[GeminiService] generateContent completed');

        const explanation = this.extractText(result);
        console.log('[GeminiService] Extracted text, length:', explanation.length);

        // Cache for 30 days
        cacheService.set(cacheKey, explanation, 30 * 24 * 60 * 60 * 1000);
        console.log('[GeminiService] Cached explanation');

        return explanation;
      });
    } catch (error) {
      console.error('[GeminiService] explainText error:', error);
      console.error('[GeminiService] Error details:', {
        message: error instanceof Error ? error.message : 'Unknown error',
        stack: error instanceof Error ? error.stack : undefined,
        type: typeof error,
        error: error
      });
      throw error;
    }
  }

  /**
   * Reset rate limiter (for testing)
   */
  public resetRateLimiter(): void {
    globalRateLimiter.reset();
  }
}

// Singleton instance
export const geminiService = new GeminiService();
