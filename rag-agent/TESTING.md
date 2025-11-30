# Testing Guide

## Automated Tests

Run all tests:

```bash
cd rag-agent
uv run pytest tests/ -v
```

Expected: All 11 tests pass ✅

## Manual Testing Guide

### 1. Start the Server

```bash
cd rag-agent
uvicorn app:app --port 8000 --reload
```

You should see:
```
INFO:     Started server process
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://127.0.0.1:8000
```

**Note:** If Qdrant is unavailable, you'll see a warning but the server will still start.

### 2. Test Root Endpoint

```bash
curl http://localhost:8000/
```

Expected response:
```json
{
  "service": "RoboLearn Backend",
  "version": "1.0.0",
  "modules": {
    "rag": "✅ Active",
    "chatkit": "⏳ Not integrated"
  },
  "endpoints": {
    "search": "POST /search",
    "health": "GET /health"
  },
  "tool": {
    "name": "search_robolearn_content",
    "import": "from rag.tools import search_tool, SEARCH_TOOL_SCHEMA",
    "note": "Add to your agent within ChatKit"
  }
}
```

### 3. Test Health Check

```bash
curl http://localhost:8000/health
```

Expected responses:
- **If Qdrant is available:**
  ```json
  {
    "status": "healthy",
    "qdrant": "connected",
    "collection": "robolearn_platform"
  }
  ```

- **If Qdrant is unavailable:**
  ```json
  {
    "status": "degraded",
    "qdrant": "error",
    "collection": "connection error message"
  }
  ```

Both are valid - the server should not crash.

### 4. Test Search Endpoint

#### Basic Search (if Qdrant is available)

```bash
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -d '{
    "query": "How do I create a ROS 2 node?",
    "limit": 5
  }'
```

Expected response:
```json
{
  "query": "How do I create a ROS 2 node?",
  "results": [
    {
      "text": "...",
      "score": 0.85,
      "citation": "Workspaces and Packages | Module ROS2",
      "lesson_url": "https://mjunaidca.github.io/robolearn/docs/module-1-ros2/chapter-4-first-code/01-workspaces-packages",
      "module": "ros2",
      "chapter": 4,
      "lesson": 1,
      ...
    }
  ],
  "total_found": 5,
  "book_id": "physical-ai-robotics"
}
```

#### Search with Filters

```bash
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -d '{
    "query": "ROS 2 nodes",
    "hardware_tier": 2,
    "module": "ros2",
    "chapter_min": 1,
    "chapter_max": 5,
    "lesson": 1,
    "limit": 3
  }'
```

#### If Qdrant is Unavailable

You'll get a 503 response:
```json
{
  "detail": "Search service unavailable (Qdrant connection failed)"
}
```

This is expected - the server should not crash.

### 5. Test Validation

#### Invalid Query (too short)

```bash
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -d '{
    "query": "ab"
  }'
```

Expected: 422 status with validation error

#### Invalid Hardware Tier

```bash
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -d '{
    "query": "test query",
    "hardware_tier": 5
  }'
```

Expected: 422 status with validation error

### 6. Test Search Tool (Python)

Test the tool function directly:

```python
from rag.tools import search_tool, SEARCH_TOOL_SCHEMA

# Simple search
result = search_tool("How do I create a ROS 2 node?")
print(result)

# With filters
result = search_tool(
    query="ROS 2 nodes",
    hardware_tier=2,
    module="ros2",
    chapter_min=1,
    chapter_max=5,
    limit=3
)
print(result)

# Check schema
print(SEARCH_TOOL_SCHEMA)
```

## Pre-Deployment Checklist

- [ ] All automated tests pass (`uv run pytest tests/ -v`)
- [ ] Server starts without errors (even if Qdrant unavailable)
- [ ] Root endpoint returns service info
- [ ] Health check works (returns status even if degraded)
- [ ] Search endpoint validates input correctly
- [ ] Search endpoint handles Qdrant unavailability gracefully (503, not crash)
- [ ] Search tool function works (if Qdrant available)
- [ ] No import errors when starting server

## Deployment Notes

The backend is designed to:
- ✅ Start even if Qdrant is unavailable
- ✅ Return appropriate error codes (503) when services are down
- ✅ Log warnings but not crash on initialization failures
- ✅ Auto-register ChatKit router when available (graceful ImportError handling)

This makes it safe to deploy before ChatKit integration.

