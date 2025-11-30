# Search Strategy Analysis

## Current Search Strategy

### How It Works

1. **Query Embedding**: User query → OpenAI `text-embedding-3-small` → 1536-dim vector
2. **Vector Similarity**: Cosine similarity between query vector and chunk embeddings
3. **Filtering**: Apply filters (chapter, module, etc.) BEFORE similarity search
4. **Ranking**: Results ordered by cosine similarity score (0.0-1.0)

### Current Implementation

```python
# 1. Embed query
query_vector = embedder.embed_single("robots will see how")

# 2. Build filters (AND logic)
filters = [
    book_id == "physical-ai-robotics",
    chapter >= 0 AND chapter <= 2
]

# 3. Vector search with filters
results = qdrant.query_points(
    query=query_vector,
    query_filter=filters,
    limit=5
)

# 4. Return top 5 by similarity score
```

## Issues with Your Query

### Query: "robots will see how"

**Problems:**
1. **Grammatically awkward**: Should be "how robots see" or "how do robots see"
2. **Missing key terms**: Doesn't include "sensors", "vision", "perception"
3. **Word order matters**: Embeddings are sensitive to word order

### Results Analysis

| Rank | Score | Content | Relevance | Issue |
|------|-------|---------|-----------|-------|
| 1 | 0.619 | "How Robots See" (Ch 2, L1) | ✅ Perfect | Best match |
| 2 | 0.471 | Module 3 README (Isaac) | ❌ Not relevant | Matches "robots" + "see" but wrong topic |
| 3 | 0.469 | "Humanoid Revolution" (Ch 1, L3) | ❌ Not relevant | Matches "robots" but not about seeing |
| 4 | 0.464 | "Sensors: What They Measure" (Ch 2, L1) | ✅ Relevant | Same lesson, different chunk |
| 5 | 0.459 | "Humanoid Revolution" (Ch 1, L3) | ❌ Not relevant | Duplicate of #3 |

**Key Issues:**
- Results 2, 3, 5 are false positives (match on "robots" + "see" but wrong context)
- No keyword matching boost for exact phrase matches
- No re-ranking based on lesson_title relevance
- Module 3 result shouldn't appear (it's about Isaac, not ROS 2 sensors)

## Why This Happens

### 1. Pure Vector Search Limitations

**Problem**: Vector embeddings capture semantic meaning but:
- Can match on partial concepts ("robots" + "see" separately)
- Don't understand query intent ("how robots see" vs "robots will see how")
- No keyword/phrase matching boost

**Example**: 
- Query: "robots will see how"
- Matches: "robots" (high weight) + "see" (medium weight) = matches "Humanoid Revolution" which mentions robots and "see" in different contexts

### 2. No Query Expansion

**Problem**: Query is used as-is without:
- Synonym expansion ("see" → "vision", "perception", "sensors")
- Query rewriting ("robots will see how" → "how robots see")
- Key term extraction

### 3. No Hybrid Search

**Problem**: Only semantic search, no keyword matching:
- Can't boost exact phrase matches ("how robots see")
- Can't boost title matches (lesson_title contains query terms)
- Can't combine keyword + semantic scores

### 4. No Re-ranking

**Problem**: Results returned in Qdrant order:
- No boost for lesson_title matches
- No boost for section_title matches
- No boost for exact phrase matches
- No diversity (multiple chunks from same lesson)

## Recommended Improvements

### 1. Query Preprocessing

```python
def preprocess_query(query: str) -> str:
    """Improve query before embedding."""
    # Fix grammar
    query = query.replace("robots will see how", "how robots see")
    
    # Add synonyms for key terms
    synonyms = {
        "see": "vision perception sensors",
        "robots": "robotics robot",
    }
    
    # Extract key terms and expand
    return expanded_query
```

### 2. Hybrid Search (Keyword + Semantic)

```python
# Option A: Qdrant hybrid search (if available)
results = qdrant.query_points(
    query=query_vector,
    query_filter=filters,
    using="hybrid",  # Combines keyword + vector
    limit=10
)

# Option B: Post-process with keyword boost
for result in results:
    score = result.score
    if query.lower() in result.text.lower():
        score *= 1.2  # Boost exact matches
    if query.lower() in result.lesson_title.lower():
        score *= 1.5  # Boost title matches
    result.score = score
```

### 3. Re-ranking

```python
def rerank_results(query: str, results: list) -> list:
    """Re-rank results by multiple signals."""
    for result in results:
        score = result.score
        
        # Boost title matches
        if query.lower() in result.lesson_title.lower():
            score *= 1.5
        
        # Boost section matches
        if result.section_title and query.lower() in result.section_title.lower():
            score *= 1.3
        
        # Boost exact phrase matches
        if query.lower() in result.text.lower():
            score *= 1.2
        
        # Penalize duplicates (same lesson)
        # ...
        
        result.score = score
    
    return sorted(results, key=lambda x: x.score, reverse=True)
```

### 4. Query Expansion

```python
def expand_query(query: str) -> str:
    """Expand query with synonyms and related terms."""
    expansions = {
        "see": ["vision", "perception", "sensors", "sensing"],
        "robots": ["robotics", "robot"],
        "how": ["method", "way", "process"],
    }
    
    # Add synonyms for key terms
    expanded = query
    for term, synonyms in expansions.items():
        if term in query.lower():
            expanded += " " + " ".join(synonyms)
    
    return expanded
```

### 5. Better Filtering

```python
# Consider: If query is about "how robots see", 
# maybe filter to module="ros2" automatically?
# Or boost ROS 2 results?
```

## Immediate Fixes

### Quick Win: Query Normalization

```python
def normalize_query(query: str) -> str:
    """Normalize common query patterns."""
    # Fix common grammar issues
    patterns = {
        r"robots will see how": "how robots see",
        r"how do robots see": "how robots see",
        r"robots see": "how robots see",
    }
    
    for pattern, replacement in patterns.items():
        query = re.sub(pattern, replacement, query, flags=re.IGNORECASE)
    
    return query
```

### Quick Win: Re-rank by Title Match

```python
# After getting results from Qdrant
for result in results:
    original_score = result.score
    
    # Boost if query terms in title
    query_terms = set(query.lower().split())
    title_terms = set(result.lesson_title.lower().split())
    
    if query_terms.intersection(title_terms):
        result.score = original_score * 1.3  # 30% boost
```

## Summary

**Current Strategy**: Pure vector similarity search
- ✅ Works well for semantic queries
- ❌ Fails on grammatically awkward queries
- ❌ No keyword/phrase matching
- ❌ No re-ranking

**Recommended**: Hybrid search with re-ranking
- Semantic search (current)
- + Keyword matching boost
- + Title/section boost
- + Query expansion
- + Re-ranking

