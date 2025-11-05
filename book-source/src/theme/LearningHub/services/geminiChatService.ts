/**
 * Gemini Chat Service
 * Streaming chat with system prompt template and conversation history
 * Based on gemini-chat.contract.md
 */

import { GoogleGenerativeAI } from '@google/generative-ai';
import { GeminiService } from './geminiService';
import type { ChatRequest, ChatStreamChunk } from '../types';

const CHAT_SYSTEM_PROMPT = `You are an expert AI tutor helping students learn from a technical book about AI-native software development.
Your role is to provide clear, accurate, and educational responses based on the page content.

## Current Context
- Page: {pageTitle}
- URL: {pageUrl}

## Guidelines
1. Base answers on the provided page content FIRST
2. If the question requires knowledge from other chapters, suggest related pages
3. Use simple language appropriate for the student's level
4. Include examples when helpful
5. Keep responses under 500 words
6. If unsure, acknowledge limitations rather than guessing
7. Be encouraging and supportive

## Available Content
{pageContent}

Now answer the student's question thoughtfully.`;

class GeminiChatService extends GeminiService {
  /**
   * Send chat message with streaming response
   */
  async *sendChatMessage(request: ChatRequest): AsyncGenerator<string> {
    yield* this.withRateLimit(async function* (this: GeminiChatService) {
      const model = this.getModel();

      // Build system context
      const systemContext = CHAT_SYSTEM_PROMPT
        .replace('{pageTitle}', request.pageTitle)
        .replace('{pageUrl}', request.pageUrl)
        .replace('{pageContent}', this.formatContent(request.pageContent, 8000));

      // Construct conversation history
      const history = [
        {
          role: 'user' as const,
          parts: [{ text: systemContext }],
        },
        {
          role: 'model' as const,
          parts: [{ text: "I understand. I'm ready to help students with this content." }],
        },
        ...request.conversationHistory.slice(-10).map(msg => ({
          role: msg.role === 'user' ? ('user' as const) : ('model' as const),
          parts: [{ text: msg.content }],
        })),
      ];

      // Start chat with history
      const chat = model.startChat({
        history,
        generationConfig: {
          temperature: 0.7,
          topK: 40,
          topP: 0.95,
          maxOutputTokens: 2048,
        },
      });

      // Send user message with streaming
      const result = await chat.sendMessageStream(request.userMessage);

      // Yield chunks as they arrive
      for await (const chunk of result.stream) {
        const text = chunk.text();
        if (text) {
          yield text;
        }
      }
    }.bind(this));
  }
}

// Create singleton instance
const geminiChatService = new GeminiChatService();

/**
 * Send chat message (exported function)
 */
export async function* sendChatMessage(request: ChatRequest): AsyncGenerator<string> {
  yield* geminiChatService.sendChatMessage(request);
}
