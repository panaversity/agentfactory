# API Contract: Gemini Highlight Explanation

**Feature**: 012-learning-hub-sidebar  
**Service**: Google Gemini 2.0 Flash  
**Purpose**: Generate AI explanations for user-selected text

---

## Endpoint Information

**Provider**: Google AI (Generative Language API)  
**Model**: `gemini-2.0-flash-exp`  
**Method**: `generateContent` (non-streaming for faster response)  
**Rate Limits**: 15 RPM, 1M TPM, 1500 RPD

---

## Request Specification

### Input Parameters

```typescript
interface ExplainTextRequest {
  selectedText: string;          // Text to explain (10-1000 chars)
  pageContext: {
    url: string;                 // Current page URL
    title: string;               // Page title
    surroundingContext: string;  // 500 chars before + after selection
  };
}
```

### System Prompt Template

```typescript
const EXPLAIN_SYSTEM_PROMPT = `
You are an expert educational assistant helping students understand concepts from "${bookTitle}".

## Task
Provide a clear, concise explanation of the highlighted text below. Your explanation should:
1. Define key terms in simple language
2. Explain why this concept is important
3. Provide a brief example or analogy if helpful
4. Keep the explanation under 300 words

## Context
Page: {pageTitle}
Surrounding Content: {surroundingContext}

## Highlighted Text
"{selectedText}"

Provide your explanation now:
`;
```

### API Call Structure

```typescript
import { GoogleGenerativeAI } from '@google/generative-ai';

async function explainText(request: ExplainTextRequest): Promise<string> {
  const genAI = new GoogleGenerativeAI(process.env.GEMINI_API_KEY!);
  const model = genAI.getGenerativeModel({ model: 'gemini-2.0-flash-exp' });

  // Build prompt
  const prompt = EXPLAIN_SYSTEM_PROMPT
    .replace('{pageTitle}', request.pageContext.title)
    .replace('{surroundingContext}', request.pageContext.surroundingContext)
    .replace('{selectedText}', request.selectedText);

  // Generate response
  const result = await model.generateContent({
    contents: [{ role: 'user', parts: [{ text: prompt }] }],
    generationConfig: {
      temperature: 0.5,        // Lower for more consistent explanations
      topK: 40,
      topP: 0.9,
      maxOutputTokens: 500,    // ~300 words
    }
  });

  const response = await result.response;
  return response.text();
}
```

---

## Response Specification

### Success Response

```typescript
interface ExplainTextResponse {
  explanation: string;         // Generated explanation (50-2000 chars)
  tokensUsed: number;         // For usage tracking
}

// Example response
{
  explanation: "A lambda function is an anonymous function in Python...",
  tokensUsed: 187
}
```

### Response Validation

```typescript
function validateExplanation(text: string): boolean {
  return (
    text.length >= 50 &&
    text.length <= 2000 &&
    !text.includes('[ERROR]') &&
    !text.toLowerCase().includes('i cannot') &&
    !text.toLowerCase().includes('i don\'t have')
  );
}

// Handle invalid responses
const explanation = await explainText(request);
if (!validateExplanation(explanation)) {
  throw new ExplanationError(
    'INVALID_RESPONSE',
    'Could not generate a valid explanation. Please try again.'
  );
}
```

---

## Error Handling

| Error Code | Status | User Message | Recovery Action |
|------------|--------|--------------|-----------------|
| 429 | Too Many Requests | "Explanation limit reached. Please wait before highlighting more text." | Disable highlight button temporarily |
| 400 | Bad Request | "Selected text could not be explained. Try selecting a different passage." | Allow retry with new selection |
| 401 | Unauthorized | "AI service authentication failed. Please refresh the page." | Reload page |
| 503 | Service Unavailable | "Explanation service temporarily unavailable." | Show cached explanation if available |
| Network | Network Error | "Connection lost. Your highlight was saved without an explanation." | Save highlight, retry explanation later |
| Timeout | Request Timeout | "Explanation request timed out. Try again." | Retry button |

```typescript
class ExplanationError extends Error {
  constructor(
    public code: string,
    message: string,
    public userMessage?: string
  ) {
    super(message);
    this.name = 'ExplanationError';
  }
}

async function explainTextWithErrorHandling(
  request: ExplainTextRequest
): Promise<string> {
  try {
    await rateLimiter.checkLimit();
    const explanation = await explainText(request);
    
    if (!validateExplanation(explanation)) {
      throw new ExplanationError(
        'INVALID_RESPONSE',
        'Invalid explanation format',
        'Could not generate explanation. Please try again.'
      );
    }
    
    return explanation;
  } catch (error) {
    if (error.status === 429) {
      throw new ExplanationError(
        'RATE_LIMIT',
        'Rate limit exceeded',
        'Too many explanation requests. Please wait a moment.'
      );
    } else if (error.status === 503) {
      throw new ExplanationError(
        'SERVICE_UNAVAILABLE',
        'Service unavailable',
        'Explanation service is temporarily down.'
      );
    } else {
      throw new ExplanationError(
        'UNKNOWN_ERROR',
        error.message,
        'An unexpected error occurred. Please try again.'
      );
    }
  }
}
```

---

## Caching Strategy

**Cache Key**: MD5 hash of `selectedText + pageUrl`  
**Cache Duration**: 30 days  
**Storage**: localStorage under `learningHub_explanations_v1`

### Cache Implementation

```typescript
interface CachedExplanation {
  selectedText: string;
  explanation: string;
  pageUrl: string;
  cachedAt: number;
  expiresAt: number;
}

type ExplanationCache = {
  [cacheKey: string]: CachedExplanation;
};

function getCacheKey(text: string, pageUrl: string): string {
  const content = `${text}|${pageUrl}`;
  return CryptoJS.MD5(content).toString();
}

async function explainTextWithCache(
  request: ExplainTextRequest
): Promise<string> {
  const cacheKey = getCacheKey(
    request.selectedText,
    request.pageContext.url
  );
  
  // Check cache
  const cache: ExplanationCache = JSON.parse(
    localStorage.getItem('learningHub_explanations_v1') || '{}'
  );
  
  const cached = cache[cacheKey];
  if (cached && Date.now() < cached.expiresAt) {
    console.log('[ExplanationCache] Cache hit:', cacheKey);
    return cached.explanation;
  }
  
  // Generate new explanation
  const explanation = await explainTextWithErrorHandling(request);
  
  // Store in cache
  const TTL_MS = 30 * 24 * 60 * 60 * 1000;  // 30 days
  cache[cacheKey] = {
    selectedText: request.selectedText,
    explanation,
    pageUrl: request.pageContext.url,
    cachedAt: Date.now(),
    expiresAt: Date.now() + TTL_MS
  };
  
  localStorage.setItem('learningHub_explanations_v1', JSON.stringify(cache));
  
  return explanation;
}
```

### Cache Cleanup

```typescript
function cleanupExplanationCache(): void {
  const cache: ExplanationCache = JSON.parse(
    localStorage.getItem('learningHub_explanations_v1') || '{}'
  );
  
  const now = Date.now();
  let cleaned = 0;
  
  for (const [key, value] of Object.entries(cache)) {
    if (now > value.expiresAt) {
      delete cache[key];
      cleaned++;
    }
  }
  
  if (cleaned > 0) {
    localStorage.setItem('learningHub_explanations_v1', JSON.stringify(cache));
    console.log(`[ExplanationCache] Cleaned ${cleaned} expired entries`);
  }
}

// Run cleanup on app initialization and periodically
cleanupExplanationCache();
```

---

## Rate Limiting Strategy

**Shared Limiter**: Uses same RateLimiter as chat API (combined 15 RPM budget)

### Implementation

```typescript
class SharedRateLimiter {
  private requests: Map<string, number[]> = new Map();
  private readonly RPM_LIMIT = 15;
  private readonly WINDOW_MS = 60000;

  async checkLimit(apiType: 'chat' | 'explain' | 'quiz' | 'concepts' | 'topics'): Promise<void> {
    // Combine all API types into single limit
    const allRequests = Array.from(this.requests.values()).flat();
    const now = Date.now();
    
    // Filter to current window
    const recentRequests = allRequests.filter(t => now - t < this.WINDOW_MS);
    
    if (recentRequests.length >= this.RPM_LIMIT) {
      const oldestRequest = Math.min(...recentRequests);
      const waitTime = this.WINDOW_MS - (now - oldestRequest);
      
      throw new RateLimitError(
        `AI features temporarily limited. Available again in ${Math.ceil(waitTime / 1000)}s.`,
        waitTime
      );
    }

    // Record this request
    const typeRequests = this.requests.get(apiType) || [];
    typeRequests.push(now);
    this.requests.set(apiType, typeRequests);
    
    // Cleanup old requests
    this.cleanup();
  }

  private cleanup(): void {
    const now = Date.now();
    for (const [type, requests] of this.requests.entries()) {
      this.requests.set(
        type,
        requests.filter(t => now - t < this.WINDOW_MS)
      );
    }
  }
}

const sharedLimiter = new SharedRateLimiter();

// Usage
await sharedLimiter.checkLimit('explain');
const explanation = await explainText(request);
```

---

## Performance Targets

- **Response Time**: < 2 seconds (non-streaming)
- **Cache Hit Rate**: > 60% (repeated highlights across users)
- **Success Rate**: > 99%
- **Explanation Quality**: Human-evaluated periodically

---

## Testing Strategy

### Unit Tests

```typescript
describe('Explain Text API', () => {
  it('should generate explanation for selected text', async () => {
    const request: ExplainTextRequest = {
      selectedText: 'lambda function',
      pageContext: {
        url: '/python-basics',
        title: 'Python Basics',
        surroundingContext: 'In Python, you can use a lambda function for...'
      }
    };
    
    const explanation = await explainText(request);
    expect(explanation).toContain('lambda');
    expect(explanation.length).toBeGreaterThan(50);
  });

  it('should use cache for repeated highlights', async () => {
    const request: ExplainTextRequest = { /* ... */ };
    
    const first = await explainTextWithCache(request);
    const second = await explainTextWithCache(request);
    
    expect(first).toEqual(second);
    // Verify only 1 API call was made (mock spy)
  });

  it('should handle rate limit errors gracefully', async () => {
    // Simulate 15+ requests
    await expect(
      explainTextWithErrorHandling(request)
    ).rejects.toThrow(ExplanationError);
  });
});
```

### Integration Tests

```typescript
describe('Explain Text Integration', () => {
  it('should get real explanation from Gemini API', async () => {
    const request: ExplainTextRequest = {
      selectedText: 'recursion',
      pageContext: {
        url: '/algorithms',
        title: 'Algorithms',
        surroundingContext: 'Recursion is a technique where a function calls itself...'
      }
    };
    
    const explanation = await explainText(request);
    expect(explanation).toContain('recursion');
    expect(validateExplanation(explanation)).toBe(true);
  }, 10000);  // 10s timeout
});
```

---

## Security Considerations

1. **Input Sanitization**: Strip HTML/scripts from selectedText
2. **Length Validation**: Enforce 10-1000 char limit before API call
3. **Context Validation**: Ensure surroundingContext doesn't leak sensitive data
4. **Cache Security**: Explanations stored in localStorage (user-specific, no cross-user data)
5. **API Key Protection**: Never expose client-side

---

Contract complete. Ready for implementation.
