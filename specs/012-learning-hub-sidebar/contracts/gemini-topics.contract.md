# API Contract: Gemini Related Topics

**Feature**: 012-learning-hub-sidebar  
**Service**: Google Gemini 2.0 Flash  
**Purpose**: Recommend 3-5 related chapters/sections from the book

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
interface RecommendTopicsRequest {
  pageContext: {
    url: string;                 // Current page URL
    title: string;               // Current page title
    content: string;             // Current page content (max 20000 chars)
  };
  availableTopics: Array<{       // All available chapters/sections
    title: string;
    url: string;
    description?: string;        // Optional brief description
  }>;
  targetCount: number;           // Desired number of recommendations (3-5)
}
```

### System Prompt Template

```typescript
const TOPICS_SYSTEM_PROMPT = `
You are an expert curriculum designer for "${bookTitle}", helping students discover related learning materials.

## Task
Recommend {targetCount} related chapters or sections that a student should explore after reading the current page. These recommendations should help deepen understanding or provide necessary prerequisite knowledge.

## Current Page
Title: {pageTitle}
Content Summary: {pageContent}

## Available Topics
{availableTopicsList}

## Requirements
1. Each recommendation must include:
   - Title (exact match from available topics)
   - URL (exact match from available topics)
   - Brief description (explain why it's relevant, 10-300 chars)
   - Relevance score (0.0-1.0, where 1.0 is most relevant)
2. Recommendations should be ordered by relevance (most relevant first)
3. Do NOT recommend the current page itself
4. Consider both prerequisites (what to learn before) and follow-ups (what to learn next)
5. Only recommend topics that genuinely relate to the current content

## Output Format (JSON)
Return ONLY valid JSON with this structure:
{
  "topics": [
    {
      "title": "Exact topic title from list",
      "url": "/exact-url-from-list",
      "description": "Why this is relevant...",
      "relevanceScore": 0.95
    }
  ]
}

Generate the recommendations now:
`;
```

### API Call Structure

```typescript
import { GoogleGenerativeAI } from '@google/generative-ai';

async function recommendTopics(request: RecommendTopicsRequest): Promise<RelatedTopic[]> {
  const genAI = new GoogleGenerativeAI(process.env.GEMINI_API_KEY!);
  const model = genAI.getGenerativeModel({ model: 'gemini-2.0-flash-exp' });

  // Format available topics for prompt
  const topicsList = request.availableTopics
    .map(t => `- ${t.title} (${t.url})${t.description ? ': ' + t.description : ''}`)
    .join('\n');

  const prompt = TOPICS_SYSTEM_PROMPT
    .replace('{targetCount}', request.targetCount.toString())
    .replace('{pageTitle}', request.pageContext.title)
    .replace('{pageContent}', request.pageContext.content.slice(0, 5000))  // Summary only
    .replace('{availableTopicsList}', topicsList);

  const result = await model.generateContent({
    contents: [{ role: 'user', parts: [{ text: prompt }] }],
    generationConfig: {
      temperature: 0.6,        // Moderate creativity
      topK: 40,
      topP: 0.9,
      maxOutputTokens: 1000,
    }
  });

  const response = await result.response;
  const jsonText = extractJSON(response.text());
  const parsed = JSON.parse(jsonText);

  // Generate content hash for caching
  const contentHash = CryptoJS.MD5(request.pageContext.content).toString();

  // Transform to internal format
  return parsed.topics.map((t: any) => ({
    id: crypto.randomUUID(),
    pageUrl: request.pageContext.url,
    title: t.title,
    url: t.url,
    description: t.description,
    relevanceScore: t.relevanceScore,
    contentHash,
    cachedAt: Date.now()
  }));
}
```

---

## Response Specification

### Success Response

```typescript
interface RelatedTopic {
  id: string;
  pageUrl: string;         // Source page URL
  title: string;
  url: string;             // Link to related page
  description: string;
  relevanceScore: number;  // 0.0-1.0
  contentHash: string;
  cachedAt: number;
}

type RecommendTopicsResponse = RelatedTopic[];

// Example response
[
  {
    id: "a1b2c3d4-...",
    pageUrl: "/python-basics",
    title: "Python Data Structures",
    url: "/python-data-structures",
    description: "Builds on variables by covering lists, dictionaries, and sets for organizing data.",
    relevanceScore: 0.95,
    contentHash: "abc123def456...",
    cachedAt: 1699123456789
  },
  {
    id: "e5f6g7h8-...",
    pageUrl: "/python-basics",
    title: "Introduction to Programming",
    url: "/intro-to-programming",
    description: "Provides foundational concepts helpful for understanding Python syntax and logic.",
    relevanceScore: 0.80,
    contentHash: "abc123def456...",
    cachedAt: 1699123456789
  },
  // ... 1-3 more topics
]
```

### Response Validation

```typescript
function validateTopicsResponse(
  topics: any[],
  availableTopics: Array<{ title: string; url: string }>
): boolean {
  if (!Array.isArray(topics)) return false;
  if (topics.length < 1 || topics.length > 10) return false;

  // Build lookup map for validation
  const validUrls = new Set(availableTopics.map(t => t.url));

  return topics.every(t =>
    typeof t.title === 'string' &&
    t.title.length >= 5 &&
    t.title.length <= 200 &&
    typeof t.url === 'string' &&
    validUrls.has(t.url) &&  // Must be from available topics
    typeof t.description === 'string' &&
    t.description.length >= 10 &&
    t.description.length <= 300 &&
    typeof t.relevanceScore === 'number' &&
    t.relevanceScore >= 0.0 &&
    t.relevanceScore <= 1.0
  );
}

// Handle invalid responses
const topics = await recommendTopics(request);
if (!validateTopicsResponse(topics, request.availableTopics)) {
  throw new TopicsError(
    'INVALID_RESPONSE',
    'Recommended topics failed validation. Please try again.'
  );
}
```

---

## Caching Strategy

**Cache Key**: `learningHub_topics_v1` → `{pageUrl}` → `CachedRelatedTopics`  
**Cache Duration**: 7 days  
**Invalidation**: Content hash mismatch OR TTL expired

### Cache Implementation

```typescript
interface CachedRelatedTopics {
  contentHash: string;
  cachedAt: number;
  expiresAt: number;  // cachedAt + 7 days
  topics: RelatedTopic[];
}

type RelatedTopicsStorage = {
  [pageUrl: string]: CachedRelatedTopics;
};

async function recommendTopicsWithCache(
  request: RecommendTopicsRequest
): Promise<RelatedTopic[]> {
  const storage: RelatedTopicsStorage = JSON.parse(
    localStorage.getItem('learningHub_topics_v1') || '{}'
  );
  
  const currentHash = CryptoJS.MD5(request.pageContext.content).toString();
  const cached = storage[request.pageContext.url];
  
  // Check cache validity
  if (cached &&
      cached.contentHash === currentHash &&
      Date.now() < cached.expiresAt) {
    console.log('[TopicsCache] Cache hit:', request.pageContext.url);
    return cached.topics;
  }
  
  // Generate new recommendations
  const topics = await recommendTopicsWithErrorHandling(request);
  
  // Store in cache
  const TTL_MS = 7 * 24 * 60 * 60 * 1000;  // 7 days
  storage[request.pageContext.url] = {
    contentHash: currentHash,
    cachedAt: Date.now(),
    expiresAt: Date.now() + TTL_MS,
    topics
  };
  
  localStorage.setItem('learningHub_topics_v1', JSON.stringify(storage));
  console.log('[TopicsCache] Cache updated:', request.pageContext.url);
  
  return topics;
}
```

---

## Error Handling

| Error Code | Status | User Message | Recovery Action |
|------------|--------|--------------|-----------------|
| 429 | Too Many Requests | "Topic recommendations temporarily limited. Using cached data if available." | Show cached or empty state |
| 400 | Bad Request | "Could not generate topic recommendations for this page." | Show empty state |
| 401 | Unauthorized | "AI service authentication failed. Please refresh the page." | Reload page |
| 503 | Service Unavailable | "Topic recommendation service temporarily unavailable." | Retry button with delay |
| PARSE_ERROR | JSON Parse Error | "Failed to parse topic recommendations. Please try again." | Retry button |
| VALIDATION_ERROR | Invalid Topics | "Recommended topics failed validation. Please try again." | Retry button |

```typescript
class TopicsError extends Error {
  constructor(
    public code: string,
    message: string,
    public userMessage?: string
  ) {
    super(message);
    this.name = 'TopicsError';
  }
}

async function recommendTopicsWithErrorHandling(
  request: RecommendTopicsRequest
): Promise<RelatedTopic[]> {
  try {
    await sharedLimiter.checkLimit('topics');
    const topics = await recommendTopics(request);
    
    if (!validateTopicsResponse(topics, request.availableTopics)) {
      throw new TopicsError(
        'VALIDATION_ERROR',
        'Topics validation failed',
        'Recommended topics did not meet quality standards. Please try again.'
      );
    }
    
    return topics;
  } catch (error) {
    if (error instanceof TopicsError) {
      throw error;
    } else if (error.status === 429) {
      throw new TopicsError(
        'RATE_LIMIT',
        'Rate limit exceeded',
        'Topic recommendations temporarily limited. Try again shortly.'
      );
    } else if (error instanceof SyntaxError) {
      throw new TopicsError(
        'PARSE_ERROR',
        'JSON parse error',
        'Topic recommendation format was invalid. Please try again.'
      );
    } else {
      throw new TopicsError(
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
1. Return cached topics if available
2. Show user-friendly message: "Using recently recommended topics"
3. Retry in background after cooldown

---

## Performance Targets

- **Recommendation Time**: < 3 seconds
- **Cache Hit Rate**: > 85% (topics change infrequently)
- **Success Rate**: > 98%
- **Recommendation Quality**: Human-evaluated for relevance

---

## Testing Strategy

### Unit Tests

```typescript
describe('Recommend Topics API', () => {
  it('should recommend 3-5 related topics', async () => {
    const request: RecommendTopicsRequest = {
      pageContext: {
        url: '/python-basics',
        title: 'Python Basics',
        content: 'Python variables, functions, and control flow...'
      },
      availableTopics: [
        { title: 'Data Structures', url: '/data-structures' },
        { title: 'Algorithms', url: '/algorithms' },
        { title: 'OOP in Python', url: '/oop-python' },
        // ... more topics
      ],
      targetCount: 4
    };
    
    const topics = await recommendTopics(request);
    expect(topics.length).toBeGreaterThanOrEqual(1);
    expect(topics.length).toBeLessThanOrEqual(10);
    expect(validateTopicsResponse(topics, request.availableTopics)).toBe(true);
  });

  it('should validate URLs against available topics', () => {
    const topics = [
      { title: 'Test', url: '/invalid-url', description: 'Test', relevanceScore: 0.9 }
    ];
    const availableTopics = [{ title: 'Valid', url: '/valid-url' }];
    
    expect(validateTopicsResponse(topics, availableTopics)).toBe(false);
  });

  it('should use cache for repeated requests with same content', async () => {
    const request: RecommendTopicsRequest = { /* ... */ };
    
    const first = await recommendTopicsWithCache(request);
    const second = await recommendTopicsWithCache(request);
    
    expect(first).toEqual(second);
    // Verify only 1 API call (mock spy)
  });
});
```

### Integration Tests

```typescript
describe('Recommend Topics Integration', () => {
  it('should recommend real topics from Gemini API', async () => {
    const request: RecommendTopicsRequest = {
      pageContext: {
        url: '/javascript-intro',
        title: 'Introduction to JavaScript',
        content: 'JavaScript is a programming language for web development...'
      },
      availableTopics: [
        { title: 'HTML Basics', url: '/html-basics' },
        { title: 'CSS Styling', url: '/css-styling' },
        { title: 'DOM Manipulation', url: '/dom-manipulation' },
        { title: 'Async JavaScript', url: '/async-js' },
        { title: 'Python Basics', url: '/python-basics' }
      ],
      targetCount: 3
    };
    
    const topics = await recommendTopics(request);
    
    expect(topics.length).toBeGreaterThanOrEqual(1);
    expect(topics[0].relevanceScore).toBeGreaterThan(0.5);
    expect(validateTopicsResponse(topics, request.availableTopics)).toBe(true);
  }, 15000);  // 15s timeout
});
```

---

## Prompt Engineering Notes

### Recommendation Strategy

- **Prerequisites**: Topics needed to understand current content (lower in curriculum)
- **Follow-ups**: Natural next steps (higher in curriculum)
- **Parallels**: Related concepts at similar level
- **Applications**: Practical use cases of current concepts

### Relevance Scoring Guidelines

- **0.9-1.0**: Essential prerequisite or direct continuation
- **0.7-0.89**: Strongly related, recommended
- **0.5-0.69**: Moderately related, optional
- **< 0.5**: Weakly related, should not be recommended

### Description Guidelines

- Start with relationship type: "Builds on...", "Provides context for...", "Applies concepts from..."
- Keep under 300 characters
- Be specific about the connection

---

## Extracting Available Topics

To populate `availableTopics`, parse Docusaurus sidebar configuration:

```typescript
import sidebars from '@site/sidebars';

function extractAvailableTopics(): Array<{ title: string; url: string; description?: string }> {
  const topics: Array<{ title: string; url: string }> = [];
  
  // Parse sidebar structure recursively
  function parseSidebar(items: any[]) {
    for (const item of items) {
      if (typeof item === 'string') {
        // Document ID
        topics.push({
          title: getDocTitle(item),  // Fetch from MDX frontmatter
          url: `/docs/${item}`
        });
      } else if (item.type === 'doc') {
        topics.push({
          title: item.label || getDocTitle(item.id),
          url: `/docs/${item.id}`
        });
      } else if (item.type === 'category' && item.items) {
        parseSidebar(item.items);
      }
    }
  }
  
  Object.values(sidebars).forEach(sidebar => parseSidebar(sidebar));
  
  return topics;
}
```

---

## Security Considerations

1. **URL Validation**: Ensure recommended URLs are from `availableTopics` (prevent XSS)
2. **Content Truncation**: Only send first 5000 chars of content to API (performance + privacy)
3. **Cache Security**: localStorage is user-specific (no cross-user data)
4. **Rate Limiting**: Prevent recommendation spam
5. **Input Sanitization**: Strip HTML/scripts from descriptions

---

Contract complete. Ready for implementation.
