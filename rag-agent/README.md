# RoboLearn Backend

Modular backend for RAG search, ChatKit, and Agent integration.

## Structure

```
rag-agent/
├── app.py              # Main FastAPI application
├── core/               # Shared utilities and configuration
│   ├── config.py       # Settings management
│   └── __init__.py
├── rag/                # RAG retrieval and ingestion
│   ├── search.py       # Semantic search implementation
│   ├── tools.py        # Search tool for agent integration
│   ├── ingestion/      # Content ingestion pipeline
│   └── __init__.py
├── chatkit/            # ChatKit server (agent works within ChatKit)
│   └── __init__.py
├── api/                # API routes
│   ├── search.py       # Search endpoint
│   ├── health.py       # Health check
│   └── __init__.py
├── models/             # Pydantic models
│   ├── search.py       # Search query/response models
│   └── document.py     # Document/chunk models
└── scripts/            # CLI tools
    └── ingest.py       # Ingestion CLI
```

## Quick Start

### 1. Install Dependencies

```bash
cd rag-agent
uv sync
```

### 2. Configure Environment

Create `.env` file (copy from `.env.example`):

```bash
# Qdrant Cloud
QDRANT_URL=https://your-cluster.cloud.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key

# OpenAI
OPENAI_API_KEY=sk-your-openai-key

# Book Configuration
BOOK_ID=physical-ai-robotics
DOCS_PATH=../robolearn-interface/docs

# Optional: Frontend URL for lesson links
FRONTEND_BASE_URL=https://mjunaidca.github.io/robolearn
```

uv run uvicorn app:app --port 8000 --reload

### 3. Ingest Content

**First-time ingestion (complete re-index):**

```bash
# Delete collection and re-ingest all content from scratch
uv run python scripts/ingest.py ingest --recreate
```

**What happens during ingestion:**

1. **Collection Setup**: Creates Qdrant collection with proper indexes
2. **File Discovery**: Crawls docs directory, finds all markdown files
3. **Change Detection**: Identifies new/modified/deleted files
4. **Parsing**: Extracts frontmatter metadata from each file
5. **Chunking**: Splits content into semantic chunks (by section headers)
6. **Embedding**: Generates vector embeddings using OpenAI
7. **Upload**: Stores chunks in Qdrant with metadata

**Expected output:**
```
✅ Found 64 files across 4 modules (ros2, gazebo, isaac, vla)
✅ Created 527 chunks from 64 files
✅ Embedded 527 chunks
✅ Uploaded 527 points to Qdrant
✅ Collection status: green
```

**Incremental updates (only changed files):**

```bash
# Only re-process files that changed
uv run python scripts/ingest.py ingest
```

**Check what would be updated:**

```bash
# See what files would be processed (no changes made)
uv run python scripts/ingest.py status
```

### 4. Start API

```bash
uvicorn app:app --port 8000 --reload
```

## Integrating Your ChatKit Code

**Note:** Agent works within ChatKit, so you only need to integrate ChatKit.

### Step 1: Copy Your ChatKit Code

   ```bash
# Copy your ChatKit server code
cp -r /path/to/your/chatkit/* rag-agent/chatkit_integration/
```

### Step 2: Register ChatKit Router

The router is automatically registered in `app.py` if the `chatkit` module is available:

```python
# In app.py (already configured)
try:
    from chatkit import router as chatkit_router
    app.include_router(chatkit_router)
    logger.info("ChatKit router registered")
except ImportError:
    logger.info("ChatKit router not found (expected if not yet integrated)")
```

### Step 3: Add Search Tool to Your Agent (within ChatKit)

In your agent code (within ChatKit), import and register the search tool:

```python
from rag.tools import search_tool, SEARCH_TOOL_SCHEMA

# Register with your agent (works within ChatKit)
agent.add_tool(SEARCH_TOOL_SCHEMA, search_tool)
```

## Resilient Startup

The backend is designed to start even if Qdrant is unavailable:
- **Lifespan**: Non-blocking initialization, logs warnings but doesn't fail
- **Search Tool**: Returns error message if Qdrant unavailable
- **Search API**: Returns 503 if Qdrant unavailable (service degraded, not crashed)

## API Endpoints

### Search

```bash
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -d '{
    "query": "How do I create a ROS 2 node?",
  "limit": 5
  }'
```

### Health Check

```bash
curl http://localhost:8000/health
```

## Search Tool for Agents

The `search_tool` function is available for agent integration:

```python
from rag.tools import search_tool

# Simple search
result = search_tool("How do I create a ROS 2 node?")

# With filters
result = search_tool(
    query="ROS 2 nodes",
    hardware_tier=2,
    module="ros2",
    chapter_min=4,
    chapter_max=6,
    limit=5
)

# Result includes:
# - results: List of search results with text, score, citation, lesson_url
# - total_found: Total matching documents
# - query: Original query
```

## Tool Schema

The `SEARCH_TOOL_SCHEMA` provides the OpenAI function calling schema:

```python
from rag.tools import SEARCH_TOOL_SCHEMA

# Use with OpenAI Agents SDK or ChatKit
agent.add_tool(SEARCH_TOOL_SCHEMA, search_tool)
```

## Ingestion Details

### What Happened During Your Ingestion

When you ran `ingest --recreate`, the pipeline executed these steps:

1. **Collection Setup** (Step 1/6)
   - Created Qdrant collection: `robolearn_platform`
   - Set up payload indexes for fast filtering (hardware_tier, module, chapter, lesson, etc.)

2. **File Discovery** (Step 2/6)
   - Crawled docs directory
   - Found **64 markdown files** across 4 modules:
     - `module-1-ros2` → ros2
     - `module-2-simulation` → gazebo
     - `module-3-isaac` → isaac
     - `module-4-vla` → vla

3. **Change Detection** (Step 3/6)
   - In `--recreate` mode: Processes all files (ignores state)
   - Computed file hashes for future change detection

4. **Parsing & Chunking** (Step 5/6)
   - Parsed YAML frontmatter from each file
   - Split content into semantic chunks by section headers (`##`)
   - Created **527 chunks** from 64 files:
     - **ros2**: 291 chunks
     - **gazebo**: 232 chunks
     - **isaac**: 2 chunks
     - **vla**: 2 chunks

5. **Embedding & Upload** (Step 6/6)
   - Generated vector embeddings using OpenAI `text-embedding-3-small`
   - Embedded all 527 chunks (100% progress)
   - Uploaded 527 points to Qdrant with full metadata

**Result:**
- ✅ Collection status: **green** (healthy)
- ✅ Total points in Qdrant: **527**
- ✅ Files tracked: **64**
- ✅ Ready for search queries

### Ingestion Commands Reference

| Command | Purpose | When to Use |
|---------|---------|-------------|
| `ingest --recreate` | Delete collection + full re-index | First time, major changes, troubleshooting |
| `ingest --full` | Full re-index (keeps collection) | Re-process all files without deleting |
| `ingest` | Incremental update | Regular updates (only changed files) |
| `status` | Show what would be updated | Check before ingesting |
| `info` | Show collection info | Check current state |
| `crawl` | Discover files only | Test file discovery |

### Ingestion Pipeline Steps

The ingestion process follows this pipeline:

1. **Crawling**: Discovers all `.md` files in docs directory
   - Extracts module/chapter/lesson from path structure
   - Computes file hashes for change detection

2. **Parsing**: Extracts metadata from YAML frontmatter
   - Lesson title, description
   - Hardware tier, proficiency level, teaching layer
   - Module, chapter, lesson numbers

3. **Chunking**: Splits content into semantic chunks
   - Chunks by section headers (`##`)
   - Target size: 400-512 tokens per chunk
   - 15% overlap between chunks for context

4. **Embedding**: Generates vector embeddings
   - Uses OpenAI `text-embedding-3-small` (1536 dimensions)
   - Batch processing for efficiency

5. **Upload**: Stores in Qdrant with full metadata
   - Each chunk includes: text, metadata, embedding, relationships
   - Indexed for fast filtering (hardware tier, module, chapter, etc.)

## Testing

### Run Automated Tests

```bash
uv run pytest tests/ -v
```

Expected: All 37 tests pass ✅

### Manual Testing

See [TESTING.md](TESTING.md) for complete manual testing guide.

Quick test:
```bash
# Start server
uvicorn app:app --port 8000 --reload

# In another terminal, test endpoints
curl http://localhost:8000/
curl http://localhost:8000/health
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -d '{"query": "How do I create a ROS 2 node?", "limit": 5}'
```
