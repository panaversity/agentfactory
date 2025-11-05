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
      const apiKey = process.env.GEMINI_API_KEY || '';
      
      if (!apiKey) {
        console.warn('[GeminiService] GEMINI_API_KEY not found in environment');
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
   * Reset rate limiter (for testing)
   */
  public resetRateLimiter(): void {
    globalRateLimiter.reset();
  }
}

// Singleton instance
export const geminiService = new GeminiService();
