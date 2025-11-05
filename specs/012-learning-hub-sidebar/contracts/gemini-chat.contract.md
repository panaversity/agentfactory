# API Contract: Gemini Chat

**Feature**: 012-learning-hub-sidebar  
**Service**: Google Gemini 2.0 Flash  
**Purpose**: Interactive Q&A chat with streaming responses

---

## Endpoint Information

**Provider**: Google AI (Generative Language API)  
**Model**: `gemini-2.0-flash-exp`  
**Method**: `generateContentStream`  
**Rate Limits**: 15 RPM, 1M TPM, 1500 RPD

---

## Request Specification

### Input Parameters

```typescript
interface ChatRequest {
  userMessage: string;           // User's question (1-10000 chars)
  pageContext: {
    url: string;                 // Current page URL
    title: string;               // Page title
    content: string;             // MDX content (max 20000 chars)
    sectionId?: string;          // Current section if applicable
  };
  conversationHistory: Array<{  // Last 10 messages for context
    role: 'user' | 'model';
    parts: string;
  }>;
}
```

### System Prompt Template

```typescript
const CHAT_SYSTEM_PROMPT = `
You are an expert AI tutor helping students learn from the book "${bookTitle}". 
Your role is to provide clear, accurate, and educational responses.

## Current Context
- Page: {pageTitle}
- Section: {sectionId || 'Main content'}

## Guidelines
1. Base answers on the provided page content FIRST
2. If question requires knowledge from other chapters, suggest related pages
3. Use simple language appropriate for the student's level
4. Include examples when helpful
5. Keep responses under 500 words
6. If unsure, acknowledge limitations rather than guessing

## Available Content
{pageContent}

Now answer the student's question thoughtfully.
`;
```

### API Call Structure

```typescript
import { GoogleGenerativeAI } from '@google/generative-ai';

async function* sendChatMessage(request: ChatRequest): AsyncGenerator<string> {
  const genAI = new GoogleGenerativeAI(process.env.GEMINI_API_KEY!);
  const model = genAI.getGenerativeModel({ model: 'gemini-2.0-flash-exp' });

  // Build prompt with context
  const systemContext = CHAT_SYSTEM_PROMPT
    .replace('{pageTitle}', request.pageContext.title)
    .replace('{sectionId}', request.pageContext.sectionId || 'Main content')
    .replace('{pageContent}', request.pageContext.content);

  // Construct conversation history
  const chat = model.startChat({
    history: [
      { role: 'user', parts: [{ text: systemContext }] },
      { role: 'model', parts: [{ text: 'I understand. I\'m ready to help students with this content.' }] },
      ...request.conversationHistory.map(msg => ({
        role: msg.role,
        parts: [{ text: msg.parts }]
      }))
    ],
    generationConfig: {
      temperature: 0.7,
      topK: 40,
      topP: 0.95,
      maxOutputTokens: 2048,
    }
  });

  // Send user message with streaming
  const result = await chat.sendMessageStream(request.userMessage);

  for await (const chunk of result.stream) {
    yield chunk.text();
  }
}
```

---

## Response Specification

### Success Response (Streaming)

**Type**: AsyncIterator<string>  
**Behavior**: Yields text chunks as they arrive  
**Final State**: Complete message in accumulated chunks

```typescript
interface StreamChunk {
  text: string;              // Partial response text
  isComplete: boolean;       // True on final chunk
}

// Usage example
const chunks: string[] = [];
for await (const text of sendChatMessage(request)) {
  chunks.push(text);
  // Update UI with partial response
  updateChatUI(chunks.join(''));
}

// Final assembled message
const fullResponse = chunks.join('');
```

### Error Response

```typescript
interface GeminiError {
  code: number;
  message: string;
  status: string;
}

// Error handling
try {
  for await (const chunk of sendChatMessage(request)) {
    // Process chunk
  }
} catch (error) {
  if (error.status === 429) {
    // Rate limit exceeded
    throw new ChatError('TOO_MANY_REQUESTS', 'Please wait a moment before sending another message');
  } else if (error.status === 400) {
    // Invalid request
    throw new ChatError('INVALID_REQUEST', 'Your message could not be processed');
  } else if (error.status === 503) {
    // Service unavailable
    throw new ChatError('SERVICE_UNAVAILABLE', 'AI service temporarily unavailable. Please try again.');
  } else {
    // Unknown error
    throw new ChatError('UNKNOWN_ERROR', 'An unexpected error occurred');
  }
}
```

---

## Rate Limiting Strategy

### Implementation

```typescript
class RateLimiter {
  private requests: number[] = [];  // Timestamps of recent requests
  private readonly RPM_LIMIT = 15;
  private readonly WINDOW_MS = 60000;

  async checkLimit(): Promise<void> {
    const now = Date.now();
    
    // Remove requests outside the window
    this.requests = this.requests.filter(t => now - t < this.WINDOW_MS);

    if (this.requests.length >= this.RPM_LIMIT) {
      const oldestRequest = this.requests[0];
      const waitTime = this.WINDOW_MS - (now - oldestRequest);
      
      throw new RateLimitError(
        `Rate limit reached. Please wait ${Math.ceil(waitTime / 1000)} seconds.`,
        waitTime
      );
    }

    this.requests.push(now);
  }
}

const rateLimiter = new RateLimiter();

// Use before each API call
await rateLimiter.checkLimit();
const response = await sendChatMessage(request);
```

### User-Facing Feedback

- Show remaining requests in UI: `"15 questions remaining this minute"`
- Disable input when limit reached
- Display countdown timer: `"Available again in 45 seconds"`

---

## Caching Strategy

**Not Applicable**: Chat responses are contextual and unique per conversation. No caching.

---

## Error Handling

| Error Code | Status | User Message | Recovery Action |
|------------|--------|--------------|-----------------|
| 429 | Too Many Requests | "You've reached the question limit. Please wait {X} seconds." | Show countdown, disable input |
| 400 | Bad Request | "Your message couldn't be processed. Please try rephrasing." | Allow retry with edited message |
| 401 | Unauthorized | "AI service authentication failed. Please refresh the page." | Reload page |
| 403 | Forbidden | "Access to AI service denied. Check your configuration." | Contact admin |
| 503 | Service Unavailable | "AI service is temporarily unavailable. Try again shortly." | Retry button with 30s delay |
| Network | Network Error | "Connection lost. Check your internet connection." | Retry button |
| Timeout | Request Timeout | "Request timed out. Please try again." | Retry button |

---

## Testing Strategy

### Unit Tests

```typescript
describe('Gemini Chat API', () => {
  it('should stream response chunks', async () => {
    const mockStream = ['Hello', ' world', '!'];
    // Mock API response
    const chunks: string[] = [];
    for await (const chunk of sendChatMessage(mockRequest)) {
      chunks.push(chunk);
    }
    expect(chunks).toEqual(mockStream);
  });

  it('should include conversation history', async () => {
    const request: ChatRequest = {
      userMessage: 'What did we discuss?',
      pageContext: { /* ... */ },
      conversationHistory: [
        { role: 'user', parts: 'Tell me about AI' },
        { role: 'model', parts: 'AI stands for...' }
      ]
    };
    // Verify history is passed to API
  });

  it('should handle rate limit errors', async () => {
    // Simulate 15+ requests in 1 minute
    await expect(rateLimiter.checkLimit()).rejects.toThrow(RateLimitError);
  });
});
```

### Integration Tests

```typescript
describe('Gemini Chat Integration', () => {
  it('should get real response from API', async () => {
    const response = await sendChatMessage({
      userMessage: 'What is this chapter about?',
      pageContext: {
        url: '/test-page',
        title: 'Test Chapter',
        content: 'This chapter covers testing...'
      },
      conversationHistory: []
    });
    
    const fullText = [];
    for await (const chunk of response) {
      fullText.push(chunk);
    }
    
    expect(fullText.join('')).toContain('testing');
  }, 30000);  // 30s timeout for API call
});
```

### Mock Service Worker (MSW) Setup

```typescript
import { http, HttpResponse } from 'msw';

export const geminiHandlers = [
  http.post('https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash-exp:streamGenerateContent', () => {
    const encoder = new TextEncoder();
    const stream = new ReadableStream({
      start(controller) {
        controller.enqueue(encoder.encode('data: {"candidates":[{"content":{"parts":[{"text":"Hello"}]}}]}\n\n'));
        controller.enqueue(encoder.encode('data: {"candidates":[{"content":{"parts":[{"text":" world"}]}}]}\n\n'));
        controller.enqueue(encoder.encode('data: [DONE]\n\n'));
        controller.close();
      }
    });
    
    return new HttpResponse(stream, {
      headers: { 'Content-Type': 'text/event-stream' }
    });
  })
];
```

---

## Performance Targets

- **First Token Latency**: < 1 second
- **Streaming Speed**: 20-50 tokens/second
- **Total Response Time**: < 3 seconds for typical 100-200 word answer
- **Success Rate**: > 99%

---

## Security Considerations

1. **API Key Protection**: Store in environment variable, never expose client-side
2. **Input Sanitization**: Validate userMessage length (1-10000 chars), strip HTML
3. **Content Filtering**: Gemini has built-in safety filters (harassment, hate speech, etc.)
4. **Rate Limiting**: Enforce 15 RPM limit to prevent abuse
5. **CORS**: Ensure API calls originate from allowed domains only

---

Contract complete. Ready for implementation.
