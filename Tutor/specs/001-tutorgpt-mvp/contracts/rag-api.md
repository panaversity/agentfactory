# RAG Search API Contract

**Endpoint Group**: RAG (Retrieval-Augmented Generation) Search
**Base Path**: `/api/rag`
**Purpose**: Search book content using vector similarity and metadata filtering

---

## Endpoints

### POST /api/rag/search

Search book content chunks using semantic search with multi-level scoping.

**Purpose**: Find relevant book content based on natural language queries, scoped to current lesson, chapter, or entire book.

---

#### Request

**Method**: `POST`
**Path**: `/api/rag/search`
**Content-Type**: `application/json`

**Headers**:
```
Content-Type: application/json
X-Request-ID: <optional-uuid>
```

**Body**:
```json
{
  "query": "How do I use async/await in Python?",
  "scope": "current_lesson",
  "n_results": 5,
  "current_chapter": "04-python",
  "current_lesson": "03-async"
}
```

**Fields**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `query` | string | Yes | Search query (natural language) |
| `scope` | enum | No | "current_lesson", "current_chapter", or "entire_book" (default: "current_lesson") |
| `n_results` | integer | No | Number of results to return (default: 5, max: 20) |
| `current_chapter` | string | Conditional | Required if scope is "current_lesson" or "current_chapter" |
| `current_lesson` | string | Conditional | Required if scope is "current_lesson" |

**Validation**:
- `query`: Min 1 char, max 500 chars
- `scope`: Must be one of: "current_lesson", "current_chapter", "entire_book"
- `n_results`: Integer between 1 and 20
- `current_chapter`: Required for lesson/chapter scope
- `current_lesson`: Required for lesson scope

---

#### Response (Success)

**Status**: `200 OK`
**Content-Type**: `application/json`

**Body**:
```json
{
  "query": "How do I use async/await in Python?",
  "scope": "current_lesson",
  "results": [
    {
      "chunk_id": "chunk_ch04_l03_005",
      "content": "The async/await syntax in Python allows you to write asynchronous code that looks and behaves like synchronous code. To define an async function, use the 'async def' keyword instead of just 'def'. Inside an async function, you can use the 'await' keyword to pause execution until an asynchronous operation completes...",
      "score": 0.92,
      "metadata": {
        "chapter": "04-python",
        "chapter_number": 4,
        "chapter_title": "Python Fundamentals",
        "lesson": "03-async",
        "lesson_number": 3,
        "lesson_title": "Asynchronous Programming",
        "file_path": "book-source/learn-ai-native-software-development-for-beginners/04-python/03-async/readme.md",
        "chunk_index": 5,
        "content_type": "text",
        "heading": "Using Async/Await",
        "topics": ["async", "await", "coroutines"],
        "difficulty": "intermediate"
      }
    },
    {
      "chunk_id": "chunk_ch04_l03_008",
      "content": "Here's a complete example of async/await in action:\n\n```python\nimport asyncio\n\nasync def fetch_data():\n    await asyncio.sleep(1)\n    return 'Data fetched'\n\nasync def main():\n    result = await fetch_data()\n    print(result)\n\nasyncio.run(main())\n```",
      "score": 0.87,
      "metadata": {
        "chapter": "04-python",
        "chapter_number": 4,
        "chapter_title": "Python Fundamentals",
        "lesson": "03-async",
        "lesson_number": 3,
        "lesson_title": "Asynchronous Programming",
        "file_path": "book-source/learn-ai-native-software-development-for-beginners/04-python/03-async/readme.md",
        "chunk_index": 8,
        "content_type": "code",
        "heading": "Complete Example",
        "topics": ["async", "asyncio", "example"],
        "difficulty": "intermediate"
      }
    }
  ],
  "total_results": 5,
  "search_time_ms": 45
}
```

**Response Fields**:
| Field | Type | Description |
|-------|------|-------------|
| `query` | string | Original query (echoed) |
| `scope` | string | Scope used for search |
| `results` | array | Array of search results |
| `results[].chunk_id` | string | Unique chunk identifier |
| `results[].content` | string | Chunk text content |
| `results[].score` | float | Similarity score (0.0 to 1.0) |
| `results[].metadata` | object | Chunk metadata (see below) |
| `total_results` | integer | Number of results returned |
| `search_time_ms` | integer | Search duration in milliseconds |

**Metadata Fields**:
- All fields from `BookChunkMetadata` in data-model.md
- Key fields: `chapter`, `lesson`, `file_path`, `heading`, `topics`, `difficulty`

---

#### Response (Errors)

**400 Bad Request** - Invalid request parameters
```json
{
  "error": "Invalid request",
  "details": [
    {
      "code": "MISSING_REQUIRED_FIELD",
      "message": "current_chapter is required when scope is 'current_lesson'",
      "field": "current_chapter"
    }
  ],
  "timestamp": "2025-01-08T14:30:00Z",
  "request_id": "req_abc123"
}
```

**422 Validation Error** - Query validation failed
```json
{
  "error": "Validation error",
  "details": [
    {
      "code": "INVALID_QUERY_LENGTH",
      "message": "Query must be between 1 and 500 characters",
      "field": "query"
    }
  ],
  "timestamp": "2025-01-08T14:30:00Z",
  "request_id": "req_xyz789"
}
```

**429 Rate Limited** - Too many search requests
```json
{
  "error": "Rate limit exceeded",
  "details": [
    {
      "code": "RATE_LIMIT_EXCEEDED",
      "message": "Too many search requests. Try again in 10 seconds."
    }
  ],
  "retry_after": 10,
  "timestamp": "2025-01-08T14:30:00Z"
}
```

**500 Server Error** - Search failed
```json
{
  "error": "Search failed",
  "details": [
    {
      "code": "SEARCH_ERROR",
      "message": "An error occurred during search. Please try again."
    }
  ],
  "timestamp": "2025-01-08T14:30:00Z",
  "request_id": "req_def456"
}
```

---

## Implementation Details

### Backend Flow

1. **Validate request** - Check query, scope, and context fields
2. **Check rate limits** - Verify session hasn't exceeded limits (60/min)
3. **Generate query embedding** - Use Gemini to embed the query
   - Task type: `RETRIEVAL_QUERY`
4. **Execute multi-level search**:
   - **Level 3** (if scope = "current_lesson"): Search with lesson filter
   - **Level 2** (if scope = "current_chapter"): Search with chapter filter
   - **Level 1** (if scope = "entire_book"): Search entire collection
5. **Rank and deduplicate** - Combine results, remove duplicates
6. **Format response** - Convert to API response format
7. **Track metrics** - Log search time, results count

### ChromaDB Query

```python
from google import genai
import chromadb

# Initialize clients
gemini_client = genai.Client(api_key=os.environ["GOOGLE_API_KEY"])
chroma_client = chromadb.PersistentClient(path="./data/embeddings")
collection = chroma_client.get_collection("book_content")

async def search_rag(
    query: str,
    scope: str,
    n_results: int,
    current_chapter: str | None = None,
    current_lesson: str | None = None
):
    # Generate query embedding
    result = gemini_client.models.embed_content(
        model="gemini-embedding-001",
        contents=query,
        task_type="RETRIEVAL_QUERY"  # Important!
    )
    query_embedding = result['embedding']

    # Build metadata filter
    where_filter = None
    if scope == "current_lesson" and current_lesson:
        where_filter = {"lesson": current_lesson}
    elif scope == "current_chapter" and current_chapter:
        where_filter = {"chapter": current_chapter}
    # entire_book: no filter

    # Execute search
    start_time = time.time()
    results = collection.query(
        query_embeddings=[query_embedding],
        n_results=n_results,
        where=where_filter
    )
    search_time_ms = int((time.time() - start_time) * 1000)

    # Format results
    formatted_results = []
    for i in range(len(results['ids'][0])):
        formatted_results.append({
            "chunk_id": results['ids'][0][i],
            "content": results['documents'][0][i],
            "score": 1.0 - results['distances'][0][i],  # Convert distance to score
            "metadata": results['metadatas'][0][i]
        })

    return {
        "query": query,
        "scope": scope,
        "results": formatted_results,
        "total_results": len(formatted_results),
        "search_time_ms": search_time_ms
    }
```

---

## Multi-Level Retrieval Strategy

### Level 3: Current Lesson (Most Specific)
```python
# Most relevant - current lesson context
where = {"lesson": "03-async"}
```

### Level 2: Current Chapter (Broader Context)
```python
# Related content from same chapter
where = {"chapter": "04-python"}
```

### Level 1: Entire Book (General Knowledge)
```python
# No filter - search all content
where = None
```

### Hybrid Approach (Agent Tool)
The agent tool can combine multiple levels:
1. Try Level 3 first (3 results)
2. If insufficient, add Level 2 (5 results)
3. Add Level 1 for context (2 results)

---

## Usage Example

### Frontend (TypeScript)

```typescript
async function searchBookContent(
  query: string,
  scope: 'current_lesson' | 'current_chapter' | 'entire_book'
) {
  const response = await fetch('/api/rag/search', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      query,
      scope,
      n_results: 5,
      current_chapter: getCurrentChapter(),
      current_lesson: getCurrentLesson()
    })
  });

  if (!response.ok) {
    throw new Error('Search failed');
  }

  const data = await response.json();
  return data.results;
}

// Usage
const results = await searchBookContent(
  "How do I use async/await?",
  "current_lesson"
);

results.forEach(result => {
  console.log(`Score: ${result.score}`);
  console.log(`Content: ${result.content}`);
  console.log(`Source: ${result.metadata.file_path}`);
});
```

### Agent Tool (Python)

```python
from agents import function_tool

@function_tool
async def search_book_content(
    query: str,
    scope: str = "entire_book"
) -> str:
    """
    Search the book content using RAG.

    Args:
        query: What to search for
        scope: "current_lesson", "current_chapter", or "entire_book"

    Returns:
        Formatted search results
    """
    # Call our RAG API
    response = await rag_search(query, scope, n_results=5)

    # Format for agent
    formatted = f"Search Results ({scope}):\n\n"
    for i, result in enumerate(response['results'], 1):
        formatted += f"[{i}] Score: {result['score']:.2f}\n"
        formatted += f"{result['content'][:200]}...\n"
        formatted += f"Source: {result['metadata']['file_path']}\n\n"

    formatted += f"Found {response['total_results']} results in {response['search_time_ms']}ms"
    return formatted
```

---

## Testing

### Test Cases

1. **✅ Current Lesson Search** - Query with lesson scope
2. **✅ Current Chapter Search** - Query with chapter scope
3. **✅ Entire Book Search** - Query with book scope
4. **✅ No Results** - Query that matches nothing
5. **✅ Invalid Scope** - Should return 400
6. **✅ Missing Context** - Lesson scope without current_lesson
7. **✅ Rate Limit** - Exceed 60 requests/minute
8. **✅ Long Query** - Exceed 500 characters
9. **✅ Empty Query** - Should return 400
10. **✅ Performance** - Search time < 100ms (p95)

### cURL Examples

**Search current lesson**:
```bash
curl -X POST http://localhost:8000/api/rag/search \
  -H "Content-Type: application/json" \
  -d '{
    "query": "How do I use async/await in Python?",
    "scope": "current_lesson",
    "n_results": 5,
    "current_chapter": "04-python",
    "current_lesson": "03-async"
  }'
```

**Search entire book**:
```bash
curl -X POST http://localhost:8000/api/rag/search \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is machine learning?",
    "scope": "entire_book",
    "n_results": 10
  }'
```

---

## Performance

**Target Metrics**:
- Search time: < 100ms (p95)
- Embedding generation: < 50ms
- ChromaDB query: < 50ms
- Rate limit: 60 requests/minute per session

**Optimization Strategies**:
- Cache frequent queries (future)
- Batch embedding requests (future)
- Use appropriate `n_results` (5-10 typical)
- Index metadata fields (automatic in ChromaDB)

---

## Security Considerations

1. **Query Sanitization** - Prevent injection attacks
2. **Rate Limiting** - Prevent abuse
3. **Result Filtering** - Only return indexed content
4. **No PII** - Don't include student data in responses
5. **CORS** - Restrict to allowed domains

---

## Future Enhancements (Post-MVP)

1. **Hybrid Search** - Combine vector + keyword search
2. **Query Rewriting** - Improve search quality
3. **Result Reranking** - Use LLM to rerank results
4. **Caching** - Cache frequent queries
5. **Analytics** - Track popular queries
6. **Feedback Loop** - Learn from student interactions
