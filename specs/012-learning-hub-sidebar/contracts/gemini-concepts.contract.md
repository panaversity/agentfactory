# API Contract: Gemini Key Concepts Extraction

**Feature**: 012-learning-hub-sidebar  
**Service**: Google Gemini 2.0 Flash  
**Purpose**: Extract 5-7 key concepts from page content

---

## Endpoint Information

**Provider**: Google AI (Generative Language API)  
**Model**: `gemini-2.0-flash-exp`  
**Method**: `generateContent` (non-streaming)  
**Rate Limits**: 15 RPM, 1M TPM, 1500 RPD

---

## Request Specification

### Input Parameters

```typescript
interface ExtractConceptsRequest {
  pageContext: {
    url: string;                 // Page URL
    title: string;               // Page title
    content: string;             // Full page content (max 20000 chars)
  };
  targetCount: number;           // Desired number of concepts (5-7)
}
```

### System Prompt Template

```typescript
const CONCEPTS_SYSTEM_PROMPT = `
You are an expert educational content analyst extracting key concepts from "${bookTitle}".

## Task
Identify and extract {targetCount} key concepts from the page content below. These concepts should be the most important ideas students need to understand.

## Requirements
1. Each concept must have:
   - A clear, concise title (5-100 chars)
   - A brief description explaining the concept (10-500 chars)
   - An importance score (1-10, where 10 is most critical)
   - Optional: sectionId (HTML ID to link to specific section)
2. Concepts should be ordered by importance (most important first)
3. Concepts should be distinct (no duplicates or overlaps)
4. Descriptions should be student-friendly and actionable

## Output Format (JSON)
Return ONLY valid JSON with this structure:
{
  "concepts": [
    {
      "title": "Concept Name",
      "description": "Brief explanation...",
      "importance": 9,
      "sectionId": "optional-section-id"
    }
  ]
}

## Page Content
Title: {pageTitle}
Content: {pageContent}

Extract the key concepts now:
`;
```

### API Call Structure

```typescript
import { GoogleGenerativeAI } from '@google/generative-ai';

async function extractConcepts(request: ExtractConceptsRequest): Promise<KeyConcept[]> {
  const genAI = new GoogleGenerativeAI(process.env.GEMINI_API_KEY!);
  const model = genAI.getGenerativeModel({ model: 'gemini-2.0-flash-exp' });

  const prompt = CONCEPTS_SYSTEM_PROMPT
    .replace('{targetCount}', request.targetCount.toString())
    .replace('{pageTitle}', request.pageContext.title)
    .replace('{pageContent}', request.pageContext.content.slice(0, 20000));

  const result = await model.generateContent({
    contents: [{ role: 'user', parts: [{ text: prompt }] }],
    generationConfig: {
      temperature: 0.5,        // Lower for consistent extraction
      topK: 40,
      topP: 0.9,
      maxOutputTokens: 1500,
    }
  });

  const response = await result.response;
  const jsonText = extractJSON(response.text());
  const parsed = JSON.parse(jsonText);

  // Generate content hash for caching
  const contentHash = CryptoJS.MD5(request.pageContext.content).toString();

  // Transform to internal format
  return parsed.concepts.map((c: any, index: number) => ({
    id: crypto.randomUUID(),
    pageUrl: request.pageContext.url,
    title: c.title,
    description: c.description,
    sectionId: c.sectionId || null,
    importance: c.importance || (10 - index),  // Fallback: descending order
    contentHash,
    cachedAt: Date.now()
  }));
}
```

---

## Response Specification

### Success Response

```typescript
interface KeyConcept {
  id: string;
  pageUrl: string;
  title: string;
  description: string;
  sectionId?: string;
  importance: number;  // 1-10
  contentHash: string;
  cachedAt: number;
}

type ExtractConceptsResponse = KeyConcept[];

// Example response
[
  {
    id: "a1b2c3d4-...",
    pageUrl: "/python-basics",
    title: "Variables and Data Types",
    description: "Variables store data values. Python has dynamic typing, so you don't need to declare variable types explicitly.",
    sectionId: "variables-section",
    importance: 10,
    contentHash: "abc123def456...",
    cachedAt: 1699123456789
  },
  {
    id: "e5f6g7h8-...",
    pageUrl: "/python-basics",
    title: "Functions",
    description: "Functions are reusable blocks of code defined with the 'def' keyword. They help organize code and reduce repetition.",
    sectionId: "functions-section",
    importance: 9,
    contentHash: "abc123def456...",
    cachedAt: 1699123456789
  },
  // ... 3-5 more concepts
]
```

### Response Validation

```typescript
function validateConceptsResponse(concepts: any[]): boolean {
  if (!Array.isArray(concepts)) return false;
  if (concepts.length < 3 || concepts.length > 10) return false;

  return concepts.every(c =>
    typeof c.title === 'string' &&
    c.title.length >= 5 &&
    c.title.length <= 100 &&
    typeof c.description === 'string' &&
    c.description.length >= 10 &&
    c.description.length <= 500 &&
    typeof c.importance === 'number' &&
    c.importance >= 1 &&
    c.importance <= 10 &&
    (c.sectionId === undefined || typeof c.sectionId === 'string')
  );
}

// Handle invalid responses
const concepts = await extractConcepts(request);
if (!validateConceptsResponse(concepts)) {
  throw new ConceptsError(
    'INVALID_RESPONSE',
    'Extracted concepts failed validation. Please try again.'
  );
}
```

---

## Caching Strategy

**Cache Key**: `learningHub_concepts_v1` → `{pageUrl}` → `CachedKeyConcepts`  
**Cache Duration**: 7 days  
**Invalidation**: Content hash mismatch OR TTL expired

### Cache Implementation

```typescript
interface CachedKeyConcepts {
  contentHash: string;
  cachedAt: number;
  expiresAt: number;  // cachedAt + 7 days
  concepts: KeyConcept[];
}

type KeyConceptsStorage = {
  [pageUrl: string]: CachedKeyConcepts;
};

async function extractConceptsWithCache(
  request: ExtractConceptsRequest
): Promise<KeyConcept[]> {
  const storage: KeyConceptsStorage = JSON.parse(
    localStorage.getItem('learningHub_concepts_v1') || '{}'
  );
  
  const currentHash = CryptoJS.MD5(request.pageContext.content).toString();
  const cached = storage[request.pageContext.url];
  
  // Check cache validity
  if (cached &&
      cached.contentHash === currentHash &&
      Date.now() < cached.expiresAt) {
    console.log('[ConceptsCache] Cache hit:', request.pageContext.url);
    return cached.concepts;
  }
  
  // Generate new concepts
  const concepts = await extractConceptsWithErrorHandling(request);
  
  // Store in cache
  const TTL_MS = 7 * 24 * 60 * 60 * 1000;  // 7 days
  storage[request.pageContext.url] = {
    contentHash: currentHash,
    cachedAt: Date.now(),
    expiresAt: Date.now() + TTL_MS,
    concepts
  };
  
  localStorage.setItem('learningHub_concepts_v1', JSON.stringify(storage));
  console.log('[ConceptsCache] Cache updated:', request.pageContext.url);
  
  return concepts;
}
```

### Cache Cleanup

```typescript
function cleanupConceptsCache(): void {
  const storage: KeyConceptsStorage = JSON.parse(
    localStorage.getItem('learningHub_concepts_v1') || '{}'
  );
  
  const now = Date.now();
  let cleaned = 0;
  
  for (const [pageUrl, cached] of Object.entries(storage)) {
    if (now > cached.expiresAt) {
      delete storage[pageUrl];
      cleaned++;
    }
  }
  
  if (cleaned > 0) {
    localStorage.setItem('learningHub_concepts_v1', JSON.stringify(storage));
    console.log(`[ConceptsCache] Cleaned ${cleaned} expired entries`);
  }
}

// Run cleanup on app mount
cleanupConceptsCache();
```

---

## Error Handling

| Error Code | Status | User Message | Recovery Action |
|------------|--------|--------------|-----------------|
| 429 | Too Many Requests | "Concept extraction limit reached. Using cached concepts if available." | Show cached data or retry later |
| 400 | Bad Request | "Could not extract concepts from this page. Content may be too short." | Show empty state |
| 401 | Unauthorized | "AI service authentication failed. Please refresh the page." | Reload page |
| 503 | Service Unavailable | "Concept extraction temporarily unavailable." | Retry button with delay |
| PARSE_ERROR | JSON Parse Error | "Failed to parse extracted concepts. Please try again." | Retry button |
| VALIDATION_ERROR | Invalid Concepts | "Extracted concepts failed quality checks. Please try again." | Retry button |

```typescript
class ConceptsError extends Error {
  constructor(
    public code: string,
    message: string,
    public userMessage?: string
  ) {
    super(message);
    this.name = 'ConceptsError';
  }
}

async function extractConceptsWithErrorHandling(
  request: ExtractConceptsRequest
): Promise<KeyConcept[]> {
  try {
    await sharedLimiter.checkLimit('concepts');
    const concepts = await extractConcepts(request);
    
    if (!validateConceptsResponse(concepts)) {
      throw new ConceptsError(
        'VALIDATION_ERROR',
        'Concepts validation failed',
        'Extracted concepts did not meet quality standards. Please try again.'
      );
    }
    
    return concepts;
  } catch (error) {
    if (error instanceof ConceptsError) {
      throw error;
    } else if (error.status === 429) {
      throw new ConceptsError(
        'RATE_LIMIT',
        'Rate limit exceeded',
        'Concept extraction temporarily limited. Try again shortly.'
      );
    } else if (error instanceof SyntaxError) {
      throw new ConceptsError(
        'PARSE_ERROR',
        'JSON parse error',
        'Concept extraction format was invalid. Please try again.'
      );
    } else {
      throw new ConceptsError(
        'UNKNOWN_ERROR',
        error.message,
        'An unexpected error occurred. Please try again.'
      );
    }
  }
}
```

---

## Rate Limiting Strategy

**Shared Limiter**: Uses SharedRateLimiter (combined 15 RPM)

### Fallback Behavior

If rate limit is reached:
1. Return cached concepts if available (even if slightly stale)
2. Show user-friendly message: "Using recently extracted concepts"
3. Retry extraction in background after cooldown

---

## Performance Targets

- **Extraction Time**: < 3 seconds
- **Cache Hit Rate**: > 80% (concepts change infrequently)
- **Success Rate**: > 98%
- **Concept Quality**: Human-evaluated for relevance and accuracy

---

## Testing Strategy

### Unit Tests

```typescript
describe('Extract Concepts API', () => {
  it('should extract 5-7 key concepts', async () => {
    const request: ExtractConceptsRequest = {
      pageContext: {
        url: '/python-basics',
        title: 'Python Basics',
        content: 'Python is a language. Variables store data. Functions are reusable...'
      },
      targetCount: 5
    };
    
    const concepts = await extractConcepts(request);
    expect(concepts.length).toBeGreaterThanOrEqual(3);
    expect(concepts.length).toBeLessThanOrEqual(10);
    expect(validateConceptsResponse(concepts)).toBe(true);
  });

  it('should use cache for repeated requests with same content', async () => {
    const request: ExtractConceptsRequest = { /* ... */ };
    
    const first = await extractConceptsWithCache(request);
    const second = await extractConceptsWithCache(request);
    
    expect(first).toEqual(second);
    // Verify only 1 API call (mock spy)
  });

  it('should invalidate cache on content change', async () => {
    const request1: ExtractConceptsRequest = {
      pageContext: { url: '/test', title: 'Test', content: 'Original content' },
      targetCount: 5
    };
    
    await extractConceptsWithCache(request1);
    
    const request2: ExtractConceptsRequest = {
      ...request1,
      pageContext: { ...request1.pageContext, content: 'Modified content' }
    };
    
    await extractConceptsWithCache(request2);
    
    // Verify 2 API calls (content hash changed)
  });
});
```

### Integration Tests

```typescript
describe('Extract Concepts Integration', () => {
  it('should extract real concepts from Gemini API', async () => {
    const request: ExtractConceptsRequest = {
      pageContext: {
        url: '/test-page',
        title: 'JavaScript Fundamentals',
        content: `
          JavaScript is a programming language used for web development.
          Variables can be declared with let, const, or var.
          Functions are first-class citizens in JavaScript.
          Arrays and objects are fundamental data structures.
          Asynchronous programming uses promises and async/await.
        `
      },
      targetCount: 5
    };
    
    const concepts = await extractConcepts(request);
    
    expect(concepts.length).toBeGreaterThanOrEqual(3);
    expect(concepts[0].importance).toBeGreaterThan(concepts[concepts.length - 1].importance);
    expect(validateConceptsResponse(concepts)).toBe(true);
  }, 15000);  // 15s timeout
});
```

---

## Prompt Engineering Notes

### Concept Selection Criteria

- **Coverage**: Concepts should span main topics of the page
- **Hierarchy**: High-level concepts before details
- **Actionability**: Students should know what to do with the concept
- **Context**: Tie concepts to practical applications when possible

### Title Guidelines

- Use noun phrases (e.g., "Variable Declaration", not "How to Declare Variables")
- Be specific (e.g., "Arrow Functions" not "Functions")
- Avoid redundant words (e.g., "Concept of X" → "X")

### Description Guidelines

- Start with definition or explanation
- Include why the concept matters (1 sentence)
- Avoid jargon unless explained
- Keep under 500 characters

---

## Security Considerations

1. **Input Validation**: Validate content length (max 20000 chars)
2. **Output Sanitization**: Strip HTML/scripts from titles and descriptions
3. **Cache Security**: localStorage is user-specific (no cross-user data)
4. **Rate Limiting**: Prevent extraction spam
5. **Content Hash Integrity**: Use MD5 for cache invalidation (not cryptographic security)

---

Contract complete. Ready for implementation.
