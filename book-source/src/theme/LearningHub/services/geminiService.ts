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
        const cached = cacheService.get<string>(cacheKey);

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
        cacheService.set(cacheKey, explanation, { ttl: 30 * 24 * 60 * 60 * 1000 });
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
   * Generate quiz questions from page content
   * Per gemini-quiz.contract.md
   */
  public async generateQuiz(
    pageContext: {
      url: string;
      title: string;
      content: string;
    },
    questionCount: number = 4,
    difficulty: 'easy' | 'medium' | 'hard' = 'medium'
  ): Promise<Array<{
    question: string;
    choices: string[];
    correctAnswer: number;
    explanation: string;
    difficulty: string;
  }>> {
    try {
      console.log('[GeminiService] generateQuiz called', { questionCount, difficulty });
      
      return await this.withRateLimit(async () => {
        const model = this.getModel();

        // Truncate content to 20000 chars max
        const truncatedContent = this.formatContent(pageContext.content, 20000);

        // Build prompt
        const prompt = `You are an educational assessment expert creating quiz questions.

## Task
Generate ${questionCount} multiple-choice quiz questions based on the page content below.

## Requirements
1. Each question must test understanding of key concepts from the page
2. Provide exactly 4 answer choices per question
3. Clearly mark the correct answer (use index 0-3)
4. Include a brief explanation for why the answer is correct
5. Difficulty level: ${difficulty}
6. Questions should be varied (cover different topics from the page)

## Output Format (JSON)
Return ONLY valid JSON with this structure (no markdown, no code blocks):
{
  "questions": [
    {
      "question": "What is...?",
      "choices": ["Option A", "Option B", "Option C", "Option D"],
      "correctAnswer": 0,
      "explanation": "This is correct because...",
      "difficulty": "${difficulty}"
    }
  ]
}

## Page Content
Title: ${pageContext.title}
Content: ${truncatedContent}

Generate the quiz now as valid JSON:`;

        console.log('[GeminiService] Calling model.generateContent for quiz...');

        const result = await model.generateContent({
          contents: [{ role: 'user', parts: [{ text: prompt }] }],
          generationConfig: {
            temperature: 0.8,  // Higher for question variety
            topK: 40,
            topP: 0.95,
            maxOutputTokens: 2048,
          }
        });

        console.log('[GeminiService] Quiz generateContent completed');

        const responseText = this.extractText(result);
        console.log('[GeminiService] Response text:', responseText.substring(0, 200));

        // Extract JSON from response (handle markdown code blocks if present)
        let jsonText = responseText.trim();
        
        // Remove markdown code blocks if present
        if (jsonText.startsWith('```')) {
          jsonText = jsonText.replace(/```json\n?/g, '').replace(/```\n?/g, '').trim();
        }

        // Parse JSON
        let parsed: any;
        try {
          parsed = JSON.parse(jsonText);
        } catch (parseError) {
          console.error('[GeminiService] JSON parse error:', parseError);
          console.error('[GeminiService] Raw response:', responseText);
          throw this.createError(
            'RESPONSE_PARSE_ERROR',
            'Failed to parse quiz JSON response',
            false
          );
        }

        // Validate response structure
        if (!parsed.questions || !Array.isArray(parsed.questions)) {
          throw this.createError(
            'RESPONSE_VALIDATION_ERROR',
            'Invalid quiz response structure: missing questions array',
            false
          );
        }

        // Validate each question
        const questions = parsed.questions;
        if (questions.length < 3 || questions.length > 5) {
          console.warn('[GeminiService] Quiz has unexpected question count:', questions.length);
        }

        for (let i = 0; i < questions.length; i++) {
          const q = questions[i];
          
          if (!q.question || typeof q.question !== 'string') {
            throw this.createError(
              'RESPONSE_VALIDATION_ERROR',
              `Question ${i + 1} missing or invalid question text`,
              false
            );
          }

          if (!Array.isArray(q.choices) || q.choices.length !== 4) {
            throw this.createError(
              'RESPONSE_VALIDATION_ERROR',
              `Question ${i + 1} must have exactly 4 choices`,
              false
            );
          }

          if (typeof q.correctAnswer !== 'number' || q.correctAnswer < 0 || q.correctAnswer > 3) {
            throw this.createError(
              'RESPONSE_VALIDATION_ERROR',
              `Question ${i + 1} has invalid correctAnswer (must be 0-3)`,
              false
            );
          }

          if (!q.explanation || typeof q.explanation !== 'string') {
            throw this.createError(
              'RESPONSE_VALIDATION_ERROR',
              `Question ${i + 1} missing explanation`,
              false
            );
          }
        }

        console.log('[GeminiService] Quiz validation passed:', questions.length, 'questions');

        return questions;
      });
    } catch (error) {
      console.error('[GeminiService] generateQuiz error:', error);
      throw error;
    }
  }

  /**
   * Extract key concepts from page content
   * Per gemini-concepts.contract.md with 7-day cache and MD5 content hash
   */
  public async extractConcepts(
    pageContext: {
      url: string;
      title: string;
      content: string;
    },
    targetCount: number = 6
  ): Promise<Array<{
    title: string;
    description: string;
    sectionId?: string;
    importance: number;
  }>> {
    try {
      console.log('[GeminiService] extractConcepts called', { targetCount });
      
      return await this.withRateLimit(async () => {
        const model = this.getModel();

        // Truncate content to 20000 chars max
        const truncatedContent = this.formatContent(pageContext.content, 20000);

        // Build prompt
        const prompt = `You are an expert educational content analyst extracting key concepts.

## Task
Identify and extract ${targetCount} key concepts from the page content below. These concepts should be the most important ideas students need to understand.

## Requirements
1. Each concept must have:
   - A clear, concise title (5-100 chars)
   - A brief description explaining the concept (10-500 chars)
   - An importance score (1-10, where 10 is most critical)
   - Optional: sectionId (HTML ID to link to specific section if identifiable)
2. Concepts should be ordered by importance (most important first)
3. Concepts should be distinct (no duplicates or overlaps)
4. Descriptions should be student-friendly and actionable

## Output Format (JSON)
Return ONLY valid JSON with this structure (no markdown, no code blocks):
{
  "concepts": [
    {
      "title": "Concept Name",
      "description": "Brief explanation of the concept...",
      "importance": 9,
      "sectionId": "optional-section-id"
    }
  ]
}

## Page Content
Title: ${pageContext.title}
Content: ${truncatedContent}

Extract the key concepts now as valid JSON:`;

        console.log('[GeminiService] Calling model.generateContent for concepts...');

        const result = await model.generateContent({
          contents: [{ role: 'user', parts: [{ text: prompt }] }],
          generationConfig: {
            temperature: 0.5,  // Lower for consistent extraction
            topK: 40,
            topP: 0.9,
            maxOutputTokens: 1500,
          }
        });

        console.log('[GeminiService] Concepts generateContent completed');

        const responseText = this.extractText(result);
        console.log('[GeminiService] Response text:', responseText.substring(0, 200));

        // Extract JSON from response (handle markdown code blocks if present)
        let jsonText = responseText.trim();
        
        // Remove markdown code blocks if present
        if (jsonText.startsWith('```')) {
          jsonText = jsonText.replace(/```json\n?/g, '').replace(/```\n?/g, '').trim();
        }

        // Parse JSON
        let parsed: any;
        try {
          parsed = JSON.parse(jsonText);
        } catch (parseError) {
          console.error('[GeminiService] JSON parse error:', parseError);
          console.error('[GeminiService] Raw response:', responseText);
          throw this.createError(
            'RESPONSE_PARSE_ERROR',
            'Failed to parse concepts JSON response',
            false
          );
        }

        // Validate response structure
        if (!parsed.concepts || !Array.isArray(parsed.concepts)) {
          throw this.createError(
            'RESPONSE_VALIDATION_ERROR',
            'Invalid concepts response structure: missing concepts array',
            false
          );
        }

        const concepts = parsed.concepts;
        
        if (concepts.length < 2) {
          console.warn('[GeminiService] Too few concepts extracted:', concepts.length);
        }

        // Validate each concept
        for (let i = 0; i < concepts.length; i++) {
          const c = concepts[i];
          
          if (!c.title || typeof c.title !== 'string') {
            throw this.createError(
              'RESPONSE_VALIDATION_ERROR',
              `Concept ${i + 1} missing or invalid title`,
              false
            );
          }

          if (!c.description || typeof c.description !== 'string') {
            throw this.createError(
              'RESPONSE_VALIDATION_ERROR',
              `Concept ${i + 1} missing or invalid description`,
              false
            );
          }

          if (typeof c.importance !== 'number' || c.importance < 1 || c.importance > 10) {
            // Set default importance if invalid
            c.importance = 10 - i;
          }
        }

        console.log('[GeminiService] Concepts validation passed:', concepts.length, 'concepts');

        return concepts;
      });
    } catch (error) {
      console.error('[GeminiService] extractConcepts error:', error);
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
