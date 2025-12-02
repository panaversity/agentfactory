# RAG Fundamentals: From Text to Vectors to Intelligent Search

> **Duration**: 2 Hours
> **Prerequisites**: AI-201 (OpenAI Agents SDK, Pydantic, function_tool)
> **Outcome**: Build a RAG tool that gives your agents access to your own documents

---

## What You'll Learn

1. Why your agents can't answer questions about YOUR documents
2. What embeddings are and how they capture meaning
3. How vector databases store and search embeddings
4. How to build a search tool for your agents

---

# PART 1: THE PROBLEM (15 min)

## 1.1 Your Agents Have Amnesia

Remember building agents in AI-201? They're powerful, but they have a limitation:

```python
from agents import Agent, Runner

agent = Agent(
    name="RoboLearn Helper",
    instructions="Help students learn robotics from the RoboLearn textbook."
)
```

**Question**: What happens when a student asks "What's the difference between ROS 2 topics and services?"

**Answer**: The agent will either:
- Make something up (hallucinate)
- Say "I don't know"

**Why?** Because the agent has never seen your textbook! It only knows what's in its training data (which ended months ago).

## 1.2 The Naive Solution (Doesn't Work)

"Can't we just put the whole textbook in the prompt?"

```python
agent = Agent(
    name="RoboLearn Helper",
    instructions="""Help students learn robotics.

    Here is the entire textbook:
    {entire_textbook_content}  # 500,000 words
    """
)
```

**Problems:**
| Issue | Why It's Bad |
|-------|--------------|
| Token limit | GPT-4 max is ~128K tokens. Your textbook is bigger. |
| Cost | Sending 100K tokens every message = expensive |
| Slow | More tokens = slower responses |
| Irrelevant | Most content isn't relevant to the current question |

## 1.3 The Smart Solution: RAG

**RAG** = **R**etrieval-**A**ugmented **G**eneration

Instead of sending everything, we:
1. **Search** for relevant content first
2. **Send only** the relevant parts to the agent
3. Agent **answers** based on that context

```
Student Question: "What's the difference between topics and services?"
                           ↓
                    Search textbook
                           ↓
            Find 3 relevant paragraphs
                           ↓
              Send those to the agent
                           ↓
         Agent answers with accurate info
```

**But how do we "search" effectively?** That's where embeddings come in.

---

# PART 2: UNDERSTANDING EMBEDDINGS (25 min)

This is the most important concept. Take your time here.

## 2.1 The Problem with Keyword Search

Traditional search uses keywords:

```
Query: "How do I create a publisher?"
Search: Find documents containing "create" AND "publisher"
```

**What's wrong with this?**

| Query | Document | Keyword Match? | Actually Relevant? |
|-------|----------|----------------|-------------------|
| "create a publisher" | "To make a publisher, use..." | ❌ No ("make" ≠ "create") | ✅ Yes! |
| "create a publisher" | "The publisher creates news..." | ✅ Yes | ❌ No (wrong context) |

Keywords don't understand **meaning**.

## 2.2 What is an Embedding?

An **embedding** is a list of numbers that represents the **meaning** of text.

```python
# This sentence:
"How do I create a ROS 2 publisher?"

# Becomes this list of numbers:
[0.023, -0.041, 0.089, 0.012, -0.056, 0.034, ..., -0.033]
#  ↑ 1536 numbers total (for OpenAI's model)
```

**Key insight**: These aren't random numbers. They're carefully calculated so that:
- **Similar meanings → Similar numbers**
- **Different meanings → Different numbers**

## 2.3 How Do Embeddings Capture Meaning?

Think of each number as a "score" for a concept:

```
                    Dimension 1:    Dimension 2:    Dimension 3:
                    "robotics"      "programming"   "communication"

"ROS 2 topics"        0.9              0.7             0.8
"ROS 2 pub/sub"       0.85             0.65            0.85
"Python lists"        0.1              0.9             0.2
"Weather forecast"    0.05             0.1             0.1
```

(Real embeddings have 1536 dimensions, not 3, but the concept is the same)

**"ROS 2 topics" and "ROS 2 pub/sub" have similar scores** → they're semantically similar!

## 2.4 Visualizing Embeddings

Imagine a 2D map where similar things are close together:

```
                      ↑
                      |     • "ROS 2 topics"
    ROBOTICS          |   • "ROS 2 pub/sub"
    CONCEPTS          | • "ROS 2 nodes"
                      |
                      |
                      |                    • "Python functions"
                      |                  • "Python classes"
                      |                     PROGRAMMING
                      |
   • "Weather in NYC" |
                      |
   ──────────────────────────────────────→
         UNRELATED
```

**Semantic search** = Find the points closest to your query point.

## 2.5 What Creates Embeddings?

**Embedding models** are neural networks trained to convert text to vectors.

| Model | Provider | Dimensions | Cost |
|-------|----------|------------|------|
| `text-embedding-3-small` | OpenAI | 1536 | $0.02 / 1M tokens |
| `text-embedding-3-large` | OpenAI | 3072 | $0.13 / 1M tokens |
| `text-embedding-ada-002` | OpenAI | 1536 | $0.10 / 1M tokens |
| `all-MiniLM-L6-v2` | HuggingFace | 384 | Free (local) |

**We'll use `text-embedding-3-small`** - it's cheap, fast, and good quality.

## 2.6 Hands-On: Generate Your First Embedding

Let's actually do it:

```bash
pip install openai
```

```python
from openai import OpenAI

# Initialize client (uses OPENAI_API_KEY environment variable)
client = OpenAI()

# Generate an embedding
response = client.embeddings.create(
    input="How do I create a ROS 2 publisher?",
    model="text-embedding-3-small"
)

# Get the vector
embedding = response.data[0].embedding

print(f"Type: {type(embedding)}")           # <class 'list'>
print(f"Length: {len(embedding)}")          # 1536
print(f"First 5 numbers: {embedding[:5]}")  # [0.023, -0.041, ...]
print(f"Last 5 numbers: {embedding[-5:]}")  # [..., 0.012, -0.033]
```

**Run this now!** You should see:
- A list of 1536 floating-point numbers
- Values typically between -1 and 1

## 2.7 See Similarity in Action

Let's prove that similar texts get similar embeddings:

```python
def get_embedding(text: str) -> list[float]:
    """Convert text to embedding vector."""
    response = client.embeddings.create(
        input=text,
        model="text-embedding-3-small"
    )
    return response.data[0].embedding

def cosine_similarity(a: list[float], b: list[float]) -> float:
    """Calculate similarity between two vectors (0 to 1)."""
    dot_product = sum(x * y for x, y in zip(a, b))
    magnitude_a = sum(x ** 2 for x in a) ** 0.5
    magnitude_b = sum(x ** 2 for x in b) ** 0.5
    return dot_product / (magnitude_a * magnitude_b)

# Get embeddings for different sentences
emb1 = get_embedding("How do I create a ROS 2 publisher?")
emb2 = get_embedding("Creating publishers in ROS 2")
emb3 = get_embedding("What's the weather in New York?")

# Compare similarities
print(f"'create publisher' vs 'creating publishers': {cosine_similarity(emb1, emb2):.3f}")
print(f"'create publisher' vs 'weather in NY':       {cosine_similarity(emb1, emb3):.3f}")
```

**Expected output:**
```
'create publisher' vs 'creating publishers': 0.847  # High! Similar meaning
'create publisher' vs 'weather in NY':       0.123  # Low! Different meaning
```

**This is the magic of embeddings!** They understand that "create" and "creating" mean the same thing.

---

# PART 3: VECTOR DATABASES (20 min)

Now we need somewhere to store all these embeddings.

## 3.1 What is a Vector Database?

A **vector database** is optimized for:
1. Storing millions of vectors (embeddings)
2. Finding the most similar vectors to a query **fast**
3. Filtering results by metadata

| Regular Database | Vector Database |
|------------------|-----------------|
| `SELECT * FROM docs WHERE title = 'ROS2'` | Find docs **similar to** "How do topics work?" |
| Exact match | Semantic match |
| SQL | Vector similarity |

## 3.2 Why Qdrant?

There are many vector databases. We use **Qdrant** because:

| Feature | Qdrant | Why It Matters |
|---------|--------|----------------|
| Filtering | ✅ Excellent | Filter by module, tier, etc. |
| Free tier | ✅ 1GB free | Good for learning |
| Python client | ✅ Easy | Clean API |
| Speed | ✅ Fast | Rust-based |

## 3.3 Qdrant Concepts

```
┌─────────────────────────────────────────────┐
│               COLLECTION                     │
│            "robolearn_docs"                  │
├─────────────────────────────────────────────┤
│  POINT 1                                     │
│  ├── id: 1                                   │
│  ├── vector: [0.02, -0.04, 0.08, ...]       │ ← The embedding
│  └── payload: {                              │ ← Metadata
│        "text": "ROS 2 topics enable...",     │
│        "module": "ros2_basics",              │
│        "chapter": 2,                         │
│        "hardware_tier": 1                    │
│      }                                       │
├─────────────────────────────────────────────┤
│  POINT 2                                     │
│  ├── id: 2                                   │
│  ├── vector: [0.05, 0.03, -0.02, ...]       │
│  └── payload: {...}                          │
├─────────────────────────────────────────────┤
│  ... thousands more points ...               │
└─────────────────────────────────────────────┘
```

**Key terms:**
- **Collection** = like a table (holds all your vectors)
- **Point** = one vector + its metadata
- **Vector** = the embedding (list of numbers)
- **Payload** = metadata (text, source, module, etc.)

## 3.4 Setup Qdrant

```bash
pip install qdrant-client
```

```python
from qdrant_client import QdrantClient

# Option 1: In-memory (for learning - no server needed!)
client = QdrantClient(":memory:")
print("Connected to in-memory Qdrant!")

# Option 2: Qdrant Cloud (for production)
# client = QdrantClient(
#     url="https://your-cluster.cloud.qdrant.io",
#     api_key="your-api-key"
# )
```

**For this lesson, use `:memory:`** - it works without any setup!

## 3.5 Create a Collection

```python
from qdrant_client.models import Distance, VectorParams

# Create a collection for our documents
client.create_collection(
    collection_name="robolearn_docs",
    vectors_config=VectorParams(
        size=1536,              # Must match embedding model output size!
        distance=Distance.COSINE  # How to measure similarity
    )
)

print("Collection created!")
```

**Why these settings?**
- `size=1536`: OpenAI's text-embedding-3-small outputs 1536 dimensions
- `Distance.COSINE`: Best for text (measures angle between vectors, ignores magnitude)

---

# PART 4: INGESTING DOCUMENTS (20 min)

Now let's store some actual content!

## 4.1 Prepare Your Documents

Here's real content from the RoboLearn textbook:

```python
# Sample documents from RoboLearn
documents = [
    {
        "id": 1,
        "text": "ROS 2 (Robot Operating System) is a middleware that transforms robots from monolithic programs into modular, distributed networks. Think of ROS 2 as a nervous system for your robot: sensory neurons (sensors) feed data to the brain (perception), which sends signals to muscles (actuators).",
        "source": "module-1-ros2/index.md",
        "module": "ros2_basics",
        "chapter": 1,
        "hardware_tier": 1
    },
    {
        "id": 2,
        "text": "Nodes are modular processes that run independently and communicate via ROS 2. Each node performs a specific task (reading a sensor, controlling a motor) and can be started, stopped, or replaced without affecting other nodes.",
        "source": "module-1-ros2/index.md",
        "module": "ros2_basics",
        "chapter": 2,
        "hardware_tier": 1
    },
    {
        "id": 3,
        "text": "Topics enable one-way asynchronous communication using publish/subscribe pattern. Publishers send messages without knowing who receives them. Subscribers receive messages without knowing who sends them. This decouples senders and receivers.",
        "source": "module-1-ros2/index.md",
        "module": "ros2_basics",
        "chapter": 2,
        "hardware_tier": 1
    },
    {
        "id": 4,
        "text": "Services provide synchronous request/reply communication. Unlike topics where publishers don't wait, service clients send a request and wait for a response. Use services when you need confirmation (e.g., 'did the motor move?').",
        "source": "module-1-ros2/index.md",
        "module": "ros2_basics",
        "chapter": 2,
        "hardware_tier": 1
    },
    {
        "id": 5,
        "text": "NVIDIA Isaac Sim provides physics-accurate simulation for training robots. It uses RTX ray tracing for photorealistic rendering and supports synthetic data generation for computer vision training. Requires RTX GPU with 8GB+ VRAM.",
        "source": "module-3-isaac/index.md",
        "module": "isaac_sim",
        "chapter": 8,
        "hardware_tier": 2
    },
    {
        "id": 6,
        "text": "Gazebo is an open-source physics simulator that integrates with ROS 2. It simulates gravity, collisions, and sensor data (LiDAR, cameras, IMU). Use Gazebo before testing on real hardware to catch bugs safely.",
        "source": "module-2-simulation/index.md",
        "module": "gazebo",
        "chapter": 6,
        "hardware_tier": 1
    },
    {
        "id": 7,
        "text": "Vision-Language-Action (VLA) models convert natural language commands into robot actions. For example, 'pick up the red cup' is processed by an LLM that generates a sequence of motor commands.",
        "source": "module-4-vla/index.md",
        "module": "vla",
        "chapter": 12,
        "hardware_tier": 1
    },
    {
        "id": 8,
        "text": "The Unitree Go2 is a quadruped robot used for real-world testing. It features 12 motors (3 per leg), an IMU for balance, and Intel RealSense cameras for perception.",
        "source": "module-4-vla/index.md",
        "module": "vla",
        "chapter": 11,
        "hardware_tier": 4
    }
]
```

Notice the **metadata** (payload):
- `module`: Which module this content belongs to
- `chapter`: Which chapter
- `hardware_tier`: Minimum hardware needed (1=laptop, 2=GPU, 4=robot)

## 4.2 Convert Documents to Points

```python
from qdrant_client.models import PointStruct

def get_embedding(text: str) -> list[float]:
    """Convert text to embedding vector."""
    response = client.embeddings.create(
        input=text,
        model="text-embedding-3-small"
    )
    return response.data[0].embedding

# Convert each document to a Qdrant point
points = []
for doc in documents:
    print(f"Embedding document {doc['id']}: {doc['source'][:30]}...")

    # Generate embedding for the text
    vector = get_embedding(doc["text"])

    # Create a point with vector + payload
    point = PointStruct(
        id=doc["id"],
        vector=vector,
        payload={
            "text": doc["text"],
            "source": doc["source"],
            "module": doc["module"],
            "chapter": doc["chapter"],
            "hardware_tier": doc["hardware_tier"]
        }
    )
    points.append(point)

print(f"\nCreated {len(points)} points!")
```

## 4.3 Upload to Qdrant

```python
# Upload all points to the collection
client.upsert(
    collection_name="robolearn_docs",
    wait=True,  # Wait for indexing to complete
    points=points
)

print(f"✅ Uploaded {len(points)} documents to Qdrant!")
```

**Run this now!** You should see each document being embedded, then the success message.

---

# PART 5: SEARCHING (20 min)

Now the fun part - semantic search!

## 5.1 Basic Search

```python
def search(query: str, limit: int = 3):
    """Search for documents similar to the query."""
    # Step 1: Convert query to embedding
    query_vector = get_embedding(query)

    # Step 2: Search Qdrant for similar vectors
    results = client.query_points(
        collection_name="robolearn_docs",
        query=query_vector,
        limit=limit,
        with_payload=True  # Include the metadata
    )

    return results.points
```

## 5.2 Test Basic Search

```python
# Search for information about topics
results = search("How do topics work in ROS 2?")

print("🔍 Search Results:")
print("=" * 60)
for i, point in enumerate(results, 1):
    print(f"\n#{i} Score: {point.score:.3f}")
    print(f"   Source: {point.payload['source']}")
    print(f"   Module: {point.payload['module']}")
    print(f"   Text: {point.payload['text'][:100]}...")
print("=" * 60)
```

**Expected output:**
```
🔍 Search Results:
============================================================

#1 Score: 0.892
   Source: module-1-ros2/index.md
   Module: ros2_basics
   Text: Topics enable one-way asynchronous communication using publish/subscribe pattern...

#2 Score: 0.834
   Source: module-1-ros2/index.md
   Module: ros2_basics
   Text: Services provide synchronous request/reply communication...

#3 Score: 0.756
   Source: module-1-ros2/index.md
   Module: ros2_basics
   Text: Nodes are modular processes that run independently...
============================================================
```

**It works!** The document about topics scored highest, even though we asked "How do topics work?" and the document says "Topics enable..."

## 5.3 Try Different Queries

```python
queries = [
    "What's the difference between topics and services?",
    "How do I simulate a robot?",
    "What hardware do I need for Isaac Sim?",
    "How does a robot understand voice commands?"
]

for query in queries:
    results = search(query, limit=1)
    best = results[0]
    print(f"\n❓ {query}")
    print(f"✅ Best match ({best.score:.2f}): {best.payload['text'][:80]}...")
```

## 5.4 Filtered Search (Key Feature!)

Sometimes you want to search **within** a specific module or hardware tier.

```python
from qdrant_client.models import Filter, FieldCondition, MatchValue, Range

def search_filtered(
    query: str,
    module: str = None,
    max_hardware_tier: int = None,
    limit: int = 3
):
    """Search with optional filters."""
    query_vector = get_embedding(query)

    # Build filter conditions
    conditions = []

    if module:
        conditions.append(
            FieldCondition(
                key="module",
                match=MatchValue(value=module)
            )
        )

    if max_hardware_tier:
        conditions.append(
            FieldCondition(
                key="hardware_tier",
                range=Range(lte=max_hardware_tier)  # <= max tier
            )
        )

    # Create filter (None if no conditions)
    query_filter = Filter(must=conditions) if conditions else None

    results = client.query_points(
        collection_name="robolearn_docs",
        query=query_vector,
        query_filter=query_filter,
        limit=limit,
        with_payload=True
    )

    return results.points
```

## 5.5 Test Filtered Search

```python
# Without filter - returns from any module
print("🔍 Without filter (all modules):")
results = search_filtered("robot simulation")
for r in results:
    print(f"   [{r.payload['module']}] {r.payload['text'][:50]}...")

# With module filter - only ROS2 content
print("\n🔍 With module='ros2_basics':")
results = search_filtered("robot simulation", module="ros2_basics")
for r in results:
    print(f"   [{r.payload['module']}] {r.payload['text'][:50]}...")

# With hardware filter - only Tier 1 content (laptop users)
print("\n🔍 With max_hardware_tier=1 (laptop only):")
results = search_filtered("robot simulation", max_hardware_tier=1)
for r in results:
    print(f"   [Tier {r.payload['hardware_tier']}] {r.payload['text'][:50]}...")
```

**This is personalization!**
- Tier 1 students see Gazebo (laptop-friendly)
- Tier 2 students also see Isaac Sim (GPU required)

---

# PART 6: RAG AS AN AGENT TOOL (20 min)

Now we connect everything to what you learned in AI-201!

## 6.1 Define Pydantic Models

You know this from AI-201:

```python
from pydantic import BaseModel, Field

class SearchResult(BaseModel):
    """A single search result from the knowledge base."""
    content: str = Field(description="The relevant text content")
    source: str = Field(description="Source file path")
    module: str = Field(description="Module name")
    relevance_score: float = Field(description="Similarity score 0-1")
```

## 6.2 Create the Function Tool

```python
from agents import function_tool
from typing import Annotated

@function_tool
def search_robolearn(
    query: Annotated[str, "The student's question about robotics"],
    module: Annotated[str | None, "Filter by module: ros2_basics, gazebo, isaac_sim, vla"] = None,
    hardware_tier: Annotated[int, "Student's hardware tier (1=laptop, 2=GPU, 3=edge, 4=robot)"] = 1
) -> list[SearchResult]:
    """
    Search the RoboLearn textbook for relevant content.

    Use this tool BEFORE answering any technical questions about:
    - ROS 2 (nodes, topics, services, actions)
    - Simulation (Gazebo, Isaac Sim)
    - Physical robots (Unitree Go2)
    - VLA and voice commands

    The tool filters results based on the student's hardware tier,
    so they only see content they can actually use.
    """
    query_vector = get_embedding(query)

    # Build filter
    conditions = [
        FieldCondition(key="hardware_tier", range=Range(lte=hardware_tier))
    ]
    if module:
        conditions.append(
            FieldCondition(key="module", match=MatchValue(value=module))
        )

    results = client.query_points(
        collection_name="robolearn_docs",
        query=query_vector,
        query_filter=Filter(must=conditions),
        limit=3,
        with_payload=True
    )

    return [
        SearchResult(
            content=p.payload["text"],
            source=p.payload["source"],
            module=p.payload["module"],
            relevance_score=p.score
        )
        for p in results.points
    ]
```

## 6.3 Create the RAG Agent

```python
from agents import Agent, Runner

rag_agent = Agent(
    name="RoboLearn Assistant",
    instructions="""You are a helpful teaching assistant for the RoboLearn Physical AI textbook.

IMPORTANT RULES:
1. ALWAYS use the search_robolearn tool FIRST before answering technical questions
2. Base your answers on the search results, not your general knowledge
3. Cite your sources: "According to module-1-ros2/index.md..."
4. If search returns no relevant results, say "I don't have information about that in the textbook"
5. Be concise but thorough
6. Use code examples when helpful

The student's hardware tier is passed to the search tool, so you'll only see
content appropriate for their setup.""",
    tools=[search_robolearn]
)
```

## 6.4 Test the Agent

```python
import asyncio

async def ask_robolearn(question: str, hardware_tier: int = 1):
    """Ask the RoboLearn assistant a question."""
    # Add hardware context to the question
    full_question = f"{question}\n\n[Student hardware tier: {hardware_tier}]"

    result = await Runner.run(rag_agent, full_question)

    print(f"❓ Question: {question}")
    print(f"🖥️ Hardware tier: {hardware_tier}")
    print(f"\n🤖 Answer:\n{result.final_output}")
    print("-" * 60)

# Test it!
asyncio.run(ask_robolearn("What's the difference between topics and services in ROS 2?"))
```

**What happens:**
1. Agent receives the question
2. Agent calls `search_robolearn` tool
3. Tool returns relevant documents
4. Agent synthesizes an answer from those documents
5. Answer is grounded in YOUR content!

---

# PART 7: COMPLETE WORKING EXAMPLE (5 min)

Copy this entire file to test everything:

```python
"""
RAG Tool for RoboLearn - Complete Example
File: rag_tool.py
Run: python rag_tool.py
"""

from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance, VectorParams, PointStruct,
    Filter, FieldCondition, MatchValue, Range
)
from openai import OpenAI
from pydantic import BaseModel, Field
from agents import Agent, Runner, function_tool
from typing import Annotated
import asyncio

# ============================================================
# SETUP
# ============================================================

# Initialize clients
qdrant = QdrantClient(":memory:")
openai_client = OpenAI()

# Embedding function
def get_embedding(text: str) -> list[float]:
    response = openai_client.embeddings.create(
        input=text,
        model="text-embedding-3-small"
    )
    return response.data[0].embedding

# ============================================================
# CREATE COLLECTION & INGEST DATA
# ============================================================

# Create collection
qdrant.create_collection(
    collection_name="robolearn",
    vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
)

# Sample documents
documents = [
    {"id": 1, "text": "Topics enable one-way async communication via publish/subscribe. Publishers don't know subscribers.",
     "module": "ros2_basics", "hardware_tier": 1, "source": "ch2.md"},
    {"id": 2, "text": "Services provide sync request/reply. Client sends request and waits for response.",
     "module": "ros2_basics", "hardware_tier": 1, "source": "ch2.md"},
    {"id": 3, "text": "Nodes are modular processes. Each performs one task and communicates via topics/services.",
     "module": "ros2_basics", "hardware_tier": 1, "source": "ch2.md"},
    {"id": 4, "text": "Isaac Sim requires RTX GPU (8GB+ VRAM) for physics simulation and synthetic data.",
     "module": "isaac_sim", "hardware_tier": 2, "source": "ch8.md"},
    {"id": 5, "text": "Gazebo is open-source simulator. Works on laptop. Simulates physics, sensors, collisions.",
     "module": "gazebo", "hardware_tier": 1, "source": "ch6.md"},
]

# Ingest
points = [
    PointStruct(id=d["id"], vector=get_embedding(d["text"]), payload=d)
    for d in documents
]
qdrant.upsert(collection_name="robolearn", points=points, wait=True)
print(f"✅ Ingested {len(documents)} documents")

# ============================================================
# PYDANTIC MODEL
# ============================================================

class SearchResult(BaseModel):
    content: str
    source: str
    module: str
    score: float

# ============================================================
# FUNCTION TOOL
# ============================================================

@function_tool
def search_robolearn(
    query: Annotated[str, "Student's question"],
    hardware_tier: Annotated[int, "Hardware tier 1-4"] = 1
) -> list[SearchResult]:
    """Search RoboLearn textbook. Always use before answering technical questions."""
    results = qdrant.query_points(
        collection_name="robolearn",
        query=get_embedding(query),
        query_filter=Filter(must=[
            FieldCondition(key="hardware_tier", range=Range(lte=hardware_tier))
        ]),
        limit=2,
        with_payload=True
    )
    return [
        SearchResult(
            content=p.payload["text"],
            source=p.payload["source"],
            module=p.payload["module"],
            score=p.score
        )
        for p in results.points
    ]

# ============================================================
# AGENT
# ============================================================

agent = Agent(
    name="RoboLearn Assistant",
    instructions="Use search_robolearn before answering. Cite sources. Be concise.",
    tools=[search_robolearn]
)

# ============================================================
# RUN
# ============================================================

async def main():
    questions = [
        "What's the difference between topics and services?",
        "How do I simulate a robot on my laptop?",
    ]

    for q in questions:
        print(f"\n❓ {q}")
        result = await Runner.run(agent, q)
        print(f"🤖 {result.final_output}")
        print("-" * 50)

if __name__ == "__main__":
    asyncio.run(main())
```

---

# KEY TAKEAWAYS

| Concept | What It Is | Why It Matters |
|---------|------------|----------------|
| **Embedding** | List of numbers representing meaning | Enables semantic search |
| **Embedding Model** | Neural network that creates embeddings | We use `text-embedding-3-small` |
| **Vector Database** | Storage optimized for embeddings | Fast similarity search |
| **Qdrant** | Our vector database | Stores vectors + metadata |
| **Payload** | Metadata on each vector | Enables filtering |
| **RAG** | Search → Inject → Generate | Grounds agents in YOUR data |

---

# EXERCISES

## Exercise 1: Add More Documents (10 min)
Add 3 more documents from different chapters. Test that search finds them.

## Exercise 2: Multi-Module Search (10 min)
Modify `search_robolearn` to accept a list of modules (e.g., search both ros2_basics AND gazebo).

## Exercise 3: Structured Output (15 min)
Create a `RAGResponse` model with `answer`, `sources`, and `confidence`. Add `output_type=RAGResponse` to the agent.

---

# NEXT STEPS

After this lesson:
1. ☐ Sign up for Qdrant Cloud (free tier)
2. ☐ Ingest your full textbook content
3. ☐ Wrap as MCP server for production
4. ☐ Add chunking for long documents

---

# QUICK REFERENCE

```python
# === Embeddings ===
from openai import OpenAI
client = OpenAI()
embedding = client.embeddings.create(input="text", model="text-embedding-3-small").data[0].embedding

# === Qdrant ===
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue, Range

client = QdrantClient(":memory:")
client.create_collection("name", vectors_config=VectorParams(size=1536, distance=Distance.COSINE))
client.upsert("name", points=[PointStruct(id=1, vector=[...], payload={...})])
client.query_points("name", query=[...], query_filter=Filter(must=[...]), limit=5)

# === Agent Tool ===
@function_tool
def search(query: Annotated[str, "description"]) -> list[Result]:
    """Docstring tells agent when to use this."""
    ...
```
