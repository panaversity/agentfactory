# Qdrant Vector Database - Knowledge Reference

> Quick reference for RoboLearn RAG implementation. Optimized for class teaching and production use.

## Overview

Qdrant is an AI-native vector database for semantic search, RAG applications, and similarity matching. Key strengths:

- **Multimodal**: Text, images, and mixed embeddings in same collection
- **Hybrid Search**: Dense + sparse vectors (BM25 + semantic)
- **Rich Filtering**: Payload-based filtering during vector search
- **Production-ready**: Distributed deployment, strict mode, rate limiting

---

## 1. Python Client Setup

### Installation
```bash
pip install qdrant-client
```

### Connection Options

```python
from qdrant_client import QdrantClient

# Local in-memory (testing/prototyping)
client = QdrantClient(":memory:")

# Local with persistence
client = QdrantClient(path="./qdrant_data")

# Remote (REST)
client = QdrantClient(url="http://localhost:6333")

# Remote with API key (Qdrant Cloud)
client = QdrantClient(
    url="https://your-cluster.cloud.qdrant.io",
    api_key="your-api-key"
)

# gRPC (faster for bulk operations)
client = QdrantClient(host="localhost", grpc_port=6334, prefer_grpc=True)
```

### Async Client
```python
from qdrant_client import AsyncQdrantClient
import asyncio

async def main():
    client = AsyncQdrantClient(url="http://localhost:6333")
    # All operations are awaitable
    await client.create_collection(...)
    await client.upsert(...)
    results = await client.query_points(...)

asyncio.run(main())
```

---

## 2. Collections

### Create Basic Collection
```python
from qdrant_client import models

client.create_collection(
    collection_name="documents",
    vectors_config=models.VectorParams(
        size=1536,  # OpenAI ada-002 dimension
        distance=models.Distance.COSINE
    )
)
```

### Distance Metrics
- `COSINE` - Normalized similarity (most common for text)
- `EUCLID` - Euclidean distance
- `DOT` - Dot product (for non-normalized vectors)

### Named Vectors (Multimodal)
```python
client.create_collection(
    collection_name="multimodal",
    vectors_config={
        "text": models.VectorParams(size=384, distance=models.Distance.COSINE),
        "image": models.VectorParams(size=512, distance=models.Distance.COSINE),
    }
)
```

### Hybrid Search Collection (Dense + Sparse)
```python
client.create_collection(
    collection_name="hybrid_docs",
    vectors_config={
        "dense": models.VectorParams(size=1536, distance=models.Distance.COSINE)
    },
    sparse_vectors_config={
        "sparse": models.SparseVectorParams(
            index=models.SparseIndexParams(on_disk=False)
        )
    }
)
```

---

## 3. Ingestion (Upsert)

### Basic Upsert
```python
from qdrant_client.models import PointStruct

client.upsert(
    collection_name="documents",
    wait=True,  # Wait for indexing
    points=[
        PointStruct(
            id=1,
            vector=[0.05, 0.61, 0.76, ...],  # 1536 dims
            payload={
                "text": "Original document text",
                "source": "chapter1.md",
                "page": 5,
                "module": "ros2_basics"
            }
        ),
        PointStruct(
            id=2,
            vector=[0.19, 0.81, 0.75, ...],
            payload={"text": "Another chunk", "source": "chapter1.md", "page": 6}
        )
    ]
)
```

### Batch Upsert with NumPy
```python
import numpy as np

vectors = np.random.rand(100, 1536)  # 100 documents
client.upsert(
    collection_name="documents",
    points=[
        PointStruct(
            id=idx,
            vector=vector.tolist(),
            payload={"doc_id": idx, "category": "robotics"}
        )
        for idx, vector in enumerate(vectors)
    ]
)
```

### Multimodal Upsert
```python
client.upsert(
    collection_name="multimodal",
    points=[
        PointStruct(
            id=1,
            vector={
                "text": text_embedding,
                "image": image_embedding
            },
            payload={"caption": "Robot arm diagram", "file": "arm.png"}
        )
    ]
)
```

---

## 4. Search & Retrieval

### Basic Similarity Search
```python
results = client.query_points(
    collection_name="documents",
    query=[0.2, 0.1, 0.9, ...],  # Query vector
    limit=5
)

for point in results.points:
    print(f"ID: {point.id}, Score: {point.score}")
    print(f"Payload: {point.payload}")
```

### Search with Payload Filter
```python
from qdrant_client.models import Filter, FieldCondition, MatchValue, Range

# Exact match filter
results = client.query_points(
    collection_name="documents",
    query=query_vector,
    query_filter=Filter(
        must=[
            FieldCondition(key="module", match=MatchValue(value="ros2_basics")),
            FieldCondition(key="page", range=Range(gte=1, lte=50))
        ]
    ),
    limit=10
)
```

### Filter Conditions

| Condition | Example | Use Case |
|-----------|---------|----------|
| `MatchValue` | `match=MatchValue(value="ros2")` | Exact match |
| `MatchAny` | `match=MatchAny(any=["ros2", "gazebo"])` | Match any value |
| `Range` | `range=Range(gte=10, lt=100)` | Numeric range |
| `IsEmpty` | `is_empty=IsEmpty()` | Field is null/empty |
| `HasId` | Points with specific IDs | Filter by point ID |

### Boolean Logic
```python
Filter(
    must=[...],      # AND - all conditions required
    should=[...],    # OR - at least one required
    must_not=[...]   # NOT - exclude matches
)
```

---

## 5. Hybrid Search (Dense + Sparse)

### Prepare Sparse Vector (BM25-style)
```python
# Using a sparse encoder (e.g., SPLADE, BM25)
def compute_sparse_vector(text):
    # Returns (indices, values) for non-zero dimensions
    # indices: which vocabulary terms appear
    # values: their weights
    return indices, values
```

### Execute Hybrid Search
```python
query_text = "How do ROS2 nodes communicate?"

# Compute both vector types
dense_vector = embedding_model.encode(query_text)
sparse_indices, sparse_values = compute_sparse_vector(query_text)

# Batch search across both
results = client.search_batch(
    collection_name="hybrid_docs",
    requests=[
        models.SearchRequest(
            vector=models.NamedVector(name="dense", vector=dense_vector),
            limit=10
        ),
        models.SearchRequest(
            vector=models.NamedSparseVector(
                name="sparse",
                vector=models.SparseVector(
                    indices=sparse_indices,
                    values=sparse_values
                )
            ),
            limit=10
        )
    ]
)

# Combine results (RRF, linear combination, etc.)
dense_results, sparse_results = results
```

### LangChain Hybrid (Simplified)
```python
from langchain_qdrant import QdrantVectorStore, FastEmbedSparse, RetrievalMode

sparse_embeddings = FastEmbedSparse(model_name="Qdrant/bm25")

qdrant = QdrantVectorStore.from_documents(
    docs,
    embedding=dense_embeddings,
    sparse_embedding=sparse_embeddings,
    location=":memory:",
    collection_name="hybrid_docs",
    retrieval_mode=RetrievalMode.HYBRID
)

results = qdrant.similarity_search("ROS2 communication")
```

---

## 6. Multimodal Capabilities

### Supported Models
| Model | Dimensions | Use Case |
|-------|------------|----------|
| `clip-ViT-B-32` | 512 | Image-text (basic) |
| `llamaindex/vdr-2b-multi-v1` | Varies | Image + text (advanced) |
| `Qdrant/colpali-v1.3-fp16` | Late interaction | Document screenshots |

### Image Embedding Example
```python
from PIL import Image
from sentence_transformers import SentenceTransformer

model = SentenceTransformer("clip-ViT-B-32")

# Embed image
image_embedding = model.encode(Image.open("robot.png"))

# Embed text query
text_embedding = model.encode("robotic arm manipulation")

# Both in same vector space - can cross-search
```

### FastEmbed Multimodal
```python
from fastembed import LateInteractionMultimodalEmbedding

model = LateInteractionMultimodalEmbedding(model_name="Qdrant/colpali-v1.3-fp16")

# Embed document screenshots
doc_embeddings = list(model.embed_image(["page1.jpg", "page2.jpg"]))

# Query with text
query_embedding = model.embed_text("What is Qdrant?")
```

---

## 7. RAG Integration Pattern

### Complete RAG Flow
```python
from qdrant_client import QdrantClient
from openai import OpenAI

qdrant = QdrantClient(url="...", api_key="...")
openai = OpenAI()

def embed_text(text: str) -> list[float]:
    response = openai.embeddings.create(
        input=text,
        model="text-embedding-ada-002"
    )
    return response.data[0].embedding

def retrieve(query: str, top_k: int = 5) -> list[str]:
    query_vector = embed_text(query)
    results = qdrant.query_points(
        collection_name="documents",
        query=query_vector,
        limit=top_k,
        with_payload=True
    )
    return [point.payload["text"] for point in results.points]

def generate(query: str, context: list[str]) -> str:
    context_str = "\n\n".join(context)
    response = openai.chat.completions.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": f"Context:\n{context_str}"},
            {"role": "user", "content": query}
        ]
    )
    return response.choices[0].message.content

# Usage
query = "How do I create a ROS2 publisher?"
context = retrieve(query)
answer = generate(query, context)
```

---

## 8. Agents SDK Tool Pattern

### As OpenAI Function Tool
```python
from agents import Agent, function_tool

@function_tool
def search_documentation(
    query: str,
    module: str | None = None,
    limit: int = 5
) -> str:
    """Search RoboLearn documentation for relevant content.

    Args:
        query: Natural language search query
        module: Filter by module (ros2_basics, gazebo, isaac, vla)
        limit: Number of results to return
    """
    query_vector = embed_text(query)

    filters = None
    if module:
        filters = Filter(must=[
            FieldCondition(key="module", match=MatchValue(value=module))
        ])

    results = qdrant.query_points(
        collection_name="robolearn_docs",
        query=query_vector,
        query_filter=filters,
        limit=limit,
        with_payload=True
    )

    # Format for LLM consumption
    formatted = []
    for point in results.points:
        formatted.append(f"[{point.payload['source']}]\n{point.payload['text']}")

    return "\n\n---\n\n".join(formatted)

# Use in agent
agent = Agent(
    name="RoboLearn Assistant",
    model="gpt-4o",
    tools=[search_documentation]
)
```

---

## 9. Production Configuration

### Strict Mode (Rate Limiting)
```python
client.create_collection(
    collection_name="production",
    vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
    strict_mode_config={
        "enabled": True,
        "max_query_limit": 100,
        "max_timeout": 30,
        "read_rate_limit": 1000,  # per minute
        "write_rate_limit": 100
    }
)
```

### On-Disk Storage (Large Collections)
```python
client.create_collection(
    collection_name="large_docs",
    vectors_config=models.VectorParams(
        size=1536,
        distance=models.Distance.COSINE,
        on_disk=True  # Store vectors on disk
    ),
    optimizers_config=models.OptimizersConfigDiff(
        memmap_threshold=20000  # Threshold for memmap
    )
)
```

---

## 10. MCP Server (Official)

Qdrant provides an official MCP server for Claude Code integration:

```bash
# Install
pip install mcp-server-qdrant

# Run
mcp-server-qdrant --qdrant-url http://localhost:6333
```

### Default Embedding
Uses `sentence-transformers/all-MiniLM-L6-v2` (384 dims) via FastEmbed.

### Tools Provided
- `qdrant_store`: Store information with semantic embedding
- `qdrant_find`: Retrieve information by semantic search

---

## Quick Reference Card

| Operation | Method | Key Parameters |
|-----------|--------|----------------|
| Connect | `QdrantClient(url=...)` | `url`, `api_key`, `prefer_grpc` |
| Create collection | `create_collection()` | `vectors_config`, `sparse_vectors_config` |
| Insert | `upsert()` | `points`, `wait` |
| Search | `query_points()` | `query`, `query_filter`, `limit` |
| Batch search | `search_batch()` | `requests` (list of SearchRequest) |
| Delete | `delete()` | `points_selector` or `filter` |
| Get by ID | `retrieve()` | `ids`, `with_payload`, `with_vectors` |

---

## RoboLearn-Specific Notes

### Recommended Setup
- **Embedding Model**: `text-embedding-ada-002` (1536 dims) or `all-MiniLM-L6-v2` (384 dims)
- **Collection Structure**: One collection per book, payload includes `module`, `chapter`, `page`
- **Filtering**: Filter by module for focused retrieval
- **Hybrid**: Consider BM25 sparse for exact term matching (code snippets, commands)

### Chunk Strategy
- 512-1024 tokens per chunk
- Overlap: 50-100 tokens
- Preserve code blocks as single chunks
- Include heading hierarchy in payload
