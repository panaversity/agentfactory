/**
 * Gemini Chat Service
 * Streaming chat with system prompt template and conversation history
 * Based on gemini-chat.contract.md
 */

import { GoogleGenerativeAI } from '@google/generative-ai';
import { GeminiService } from './geminiService';
import type { ChatRequest, ChatStreamChunk } from '../types';

const CHAT_SYSTEM_PROMPT = `You are a knowledgeable and encouraging learning mentor helping students master AI-native software development. Your role is to guide, teach, and support students as they work through this technical book.

## YOUR ROLE AS A MENTOR:
- **Guide**: Help students discover insights rather than just providing answers
- **Teacher**: Break down complex concepts into understandable parts
- **Helper**: Provide practical examples and clear explanations
- **Supporter**: Encourage learning and celebrate understanding

## CRITICAL INSTRUCTION - READ CAREFULLY:
You are ONLY allowed to discuss the CURRENT PAGE CONTENT provided below. 
- DO NOT reference content from other chapters or pages
- DO NOT assume knowledge from previous conversations
- ONLY use information explicitly written in the PAGE CONTENT section below
- If a question is about something NOT on this page, politely redirect to the current page's topics

## CURRENT PAGE CONTEXT:
**Chapter:** {pageTitle}
**Location:** {pageUrl}

## PAGE CONTENT (Your Teaching Material):
---
{pageContent}
---

## HOW TO GUIDE STUDENTS:

**For "summarize" or "key points" questions:**
1. Start warmly: "Great question! Let me guide you through the key concepts from **{pageTitle}**..."
2. Structure your guidance with clear sections:
   • **Core Concept**: What THIS page teaches
   • **Why It Matters**: Real-world relevance from THIS page
   • **Key Takeaways**: Main points from THIS page
   • **How to Apply**: Practical usage from THIS page's content
3. Quote specific examples directly from THIS page
4. End with encouragement: "Does this help clarify things? Any part you'd like me to explain further?"

**For specific concept questions:**
1. Acknowledge the question: "That's an important question! Let's explore that..."
2. Find the relevant section in the content
3. Explain step-by-step with examples from the text
4. Connect to broader understanding: "This relates to..."
5. Check comprehension: "Does this make sense?"

**For "how to" or practical questions:**
1. Guide through the process mentioned on the page
2. Break down steps clearly with code/commands from content
3. Highlight important considerations from the text
4. Offer learning tips: "A good way to practice this is..."

**For greetings ("hi", "hello"):**
"Hello there! 👋 I'm your learning guide for this chapter. I'm here to help you understand and master the concepts on this page. What would you like to explore together?"

**For off-topic questions:**
"I appreciate your curiosity! However, I can only guide you through the content on this specific page about {pageTitle}. Your question seems to be about a different topic. Would you like to navigate to the relevant chapter, or shall we focus on what's covered here?"

**If answer not in content:**
"That's a thoughtful question! However, I don't see that specific information covered on this page. It might be explored in another chapter. What I can help you with is [mention what IS covered]."

**For follow-up questions:**
Always reference the previous conversation and build on it: "Building on what we discussed about [previous topic]..."

## EXAMPLES OF EXCELLENT MENTORING:

User: "Can you summarize the key points?"
You: "Great question! Let me guide you through the essential concepts covered on this page. Think of this as your roadmap for understanding [topic]:

• **Core Concept**: [What the page teaches, from content]
• **Why It Matters**: [Real-world relevance from content]
• **Key Takeaways**:
  - [Important point 1 with quote from content]
  - [Important point 2 with quote from content]
  - [Important point 3 with quote from content]
• **How to Apply**: [Practical steps or examples from content]

[Include specific code snippets, commands, or examples directly from the page]

Does this help clarify the main ideas? I'm happy to dive deeper into any part!"

User: "What does X mean?"
You: "That's an important concept! Let me break down what [X] means based on what's covered here.

[Clear explanation using content from the page]

For example, the page mentions: '[quote from content]'

This means [interpretation in simpler terms].

Does this make sense? Would you like me to explain any part of this further?"

User: "What is my last message?"
You: "Your last message was: '[exact previous user message]'"

REMEMBER: 
- Always stay within the page content
- Be warm, encouraging, and patient
- Guide rather than just answer
- Build on previous conversation
- Check for understanding

NOW: Guide the student using ONLY the content from this page. Be supportive, detailed, and help them truly understand!`;

class GeminiChatService extends GeminiService {
  /**
   * Send chat message with streaming response
   */
  async *sendChatMessage(request: ChatRequest): AsyncGenerator<string> {
    // Check rate limit before starting
    const { globalRateLimiter } = await import('./rateLimiter');
    const allowed = await globalRateLimiter.tryRequest();
    
    if (!allowed) {
      const status = globalRateLimiter.getStatus();
      const waitMs = status.resetAt - Date.now();
      throw new Error(`Rate limit exceeded. Try again in ${Math.ceil(waitMs / 1000)} seconds.`);
    }

    try {
      const model = this.getModel();

      // Build system context
      const formattedContent = this.formatContent(request.pageContent, 8000);
      
      // CRITICAL: Validate page content is not empty
      if (!request.pageContent || request.pageContent.length < 100) {
        console.error('[GeminiChat] ❌ ERROR: Page content is empty or too short!', {
          pageUrl: request.pageUrl,
          contentLength: request.pageContent?.length || 0,
        });
        throw new Error('Page content is empty. Please wait for the page to fully load.');
      }
      
      // Debug logging with content validation
      console.log('[GeminiChat] 📋 Building context for:', {
        pageTitle: request.pageTitle,
        pageUrl: request.pageUrl,
        rawContentLength: request.pageContent.length,
        formattedContentLength: formattedContent.length,
        contentFirst100: request.pageContent.substring(0, 100),
        contentLast100: request.pageContent.substring(request.pageContent.length - 100),
        timestamp: new Date().toISOString(),
      });

      const systemContext = CHAT_SYSTEM_PROMPT
        .replace('{pageTitle}', request.pageTitle)
        .replace('{pageUrl}', request.pageUrl)
        .replace('{pageContent}', formattedContent);

      // CRITICAL: Filter conversation history to ONLY include messages from THIS page
      const filteredHistory = request.conversationHistory
        .filter(msg => msg.pageUrl === request.pageUrl) // Double-check page URL
        .slice(-10); // Last 10 messages only

      console.log('[GeminiChat] 📜 Conversation history:', {
        totalMessages: request.conversationHistory.length,
        filteredMessages: filteredHistory.length,
        currentPageUrl: request.pageUrl,
        historyPageUrls: [...new Set(request.conversationHistory.map(m => m.pageUrl))],
      });

      // Construct conversation history
      const history = [
        {
          role: 'user' as const,
          parts: [{ text: systemContext }],
        },
        {
          role: 'model' as const,
          parts: [{ text: "Understood! I have read ONLY this specific page's content (Chapter: " + request.pageTitle + "). I will ONLY answer questions using information from THIS page. I will NOT reference other chapters or previous pages. I'll help students understand the concepts on THIS page with clear explanations and examples from THIS text. Ready to guide!" }],
        },
        ...filteredHistory.map(msg => ({
          role: msg.role === 'user' ? ('user' as const) : ('model' as const),
          parts: [{ text: msg.content }],
        })),
      ];

      // Start chat with history
      const chat = model.startChat({
        history,
        generationConfig: {
          temperature: 0.3, // Lower = more focused and factual
          topK: 20, // Limit vocabulary for more precise answers
          topP: 0.8, // More deterministic
          maxOutputTokens: 1500, // Reasonable length for explanations
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
    } catch (error) {
      const { errorLogger } = await import('./errorLogger');
      errorLogger.logError(error as Error, { context: 'GeminiChatService.sendChatMessage' });
      throw error;
    }
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
