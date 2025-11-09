# Setup Instructions for TutorGPT RAG System

## Prerequisites

1. **Python 3.11+**
2. **Google Gemini API Key** with embedding permissions
   - Get your key from: https://aistudio.google.com/app/apikey
   - Ensure the key has permissions for both:
     - `gemini-2.0-flash-exp` (for agent LLM)
     - `gemini-embedding-001` (for embeddings)

## Installation Steps

### 1. Navigate to Backend Directory
```bash
cd Tutor/backend
```

### 2. Create Virtual Environment
```bash
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
```

### 3. Install Dependencies
```bash
uv pip install -e .
```

Or manually:
```bash
uv pip install aiosqlite chromadb fastapi google-genai openai-agents \
    pydantic pytest pytest-asyncio python-dotenv python-frontmatter \
    tqdm uvicorn
```

### 4. Configure Environment Variables

Copy `.env.example` to `.env`:
```bash
cp .env.example .env
```

**IMPORTANT:** Edit `.env` and add your valid Google Gemini API key:
```bash
# Edit .env file
GOOGLE_API_KEY=your_actual_api_key_here
GEMINI_API_KEY=your_actual_api_key_here  # Same key works for both
```

To get a valid API key:
1. Go to https://aistudio.google.com/app/apikey
2. Create a new API key
3. Ensure it has permissions for:
   - Generative Language API (for LLM)
   - Embedding API (for embeddings)

### 5. Ingest Book Content

This step parses all 107 lessons and generates embeddings:

```bash
python scripts/ingest_book.py --reset --test
```

**Expected output:**
```
================================================================================
TutorGPT Book Content Ingestion
================================================================================

[1/5] Initializing services...
[2/5] Resetting vector store...
[3/5] Parsing all lessons...
Found 107 markdown files
✓ Parsed 2026 chunks from book content

[4/5] Generating embeddings (batch size: 50)...
Embedding batches: 100%|██████████| 41/41 [03:25<00:00, 5.0s/it]
✓ Generated 2026 embeddings

[5/5] Storing in vector database...
✓ Vector store stats:
  - Collection: book_content
  - Total chunks: 2026
  - Embedding dimension: 768

================================================================================
✅ Ingestion complete!
================================================================================
```

**Note:** The ingestion process will take 3-5 minutes depending on API rate limits. It generates 2026 embeddings from the book content.

### 6. Test RAG Search

After ingestion, test the search:

```bash
# Test via Python
python -c "
from app.services.rag_service import RAGService, RAGSearchRequest

rag = RAGService()
request = RAGSearchRequest(
    query='What is Python?',
    scope='entire_book',
    n_results=3
)
response = rag.search_sync(request)
print(f'Found {response.total_results} results in {response.search_time_ms}ms')
for r in response.results:
    print(f'  - Score: {r.score:.2f} | {r.metadata[\"chapter_title\"]}')
"
```

### 7. Start FastAPI Server

```bash
uvicorn app.main:app --reload
```

Visit:
- API Docs: http://localhost:8000/docs
- Health Check: http://localhost:8000/health
- RAG Health: http://localhost:8000/api/rag/health

### 8. Test API Endpoint

```bash
curl -X POST http://localhost:8000/api/rag/search \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is Python?",
    "scope": "entire_book",
    "n_results": 3
  }'
```

## Troubleshooting

### Error: 403 Forbidden from Gemini API

**Problem:** The API key doesn't have embedding permissions or is invalid.

**Solution:**
1. Get a new API key from https://aistudio.google.com/app/apikey
2. Ensure the project has Embedding API enabled
3. Update `.env` with the new key
4. Try again

### Error: Collection does not exist

**Problem:** Vector database hasn't been populated yet.

**Solution:**
```bash
python scripts/ingest_book.py --reset --test
```

### Error: Module not found

**Problem:** Dependencies not installed.

**Solution:**
```bash
source .venv/bin/activate
uv pip install -e .
```

### Slow Embedding Generation

**Problem:** Gemini API rate limits.

**Solution:**
- The script uses batch processing (50 chunks/batch)
- Total time: ~3-5 minutes for 2026 chunks
- This is normal - embeddings are cached in ChromaDB

## Verification Checklist

- [ ] Virtual environment created and activated
- [ ] All dependencies installed
- [ ] `.env` file created with valid API keys
- [ ] Book content ingested successfully (2026 chunks)
- [ ] RAG search returns results
- [ ] FastAPI server starts without errors
- [ ] API endpoint responds to requests

## Next Steps

After setup is complete:

1. **Test Agent Integration:**
   ```bash
   python test_agent_live.py
   ```

2. **Integrate with Docusaurus:**
   - See `RAG_IMPLEMENTATION.md` for ChatKit integration steps

3. **Deploy:**
   - Configure production environment variables
   - Set up persistent storage for `data/embeddings`
   - Configure CORS for production domains

## File Locations

- **Vector DB:** `./data/embeddings/` (created after ingestion)
- **Session DB:** `./data/sessions.db` (created on first session)
- **Logs:** `./data/logs/tutorgpt.log`
- **Config:** `.env`

## API Key Requirements

Your Google Gemini API key must have access to:

1. **Generative Language API** (for agent LLM)
   - `gemini-2.0-flash-exp`
   - `gemini-2.0-flash`
   - `gemini-1.5-pro`

2. **Embedding API** (for RAG embeddings)
   - `gemini-embedding-001`

To verify your key has these permissions:
```bash
curl -H "Content-Type: application/json" \
  -d '{"contents":[{"parts":[{"text":"test"}]}]}' \
  "https://generativelanguage.googleapis.com/v1beta/models/gemini-embedding-001:embedContent?key=YOUR_API_KEY"
```

If you get a 403 error, the key doesn't have embedding permissions.

## Success Indicators

When everything is working:

1. ✅ Ingestion completes with 2026 chunks
2. ✅ RAG search returns relevant results
3. ✅ API endpoint returns 200 OK
4. ✅ Agent tool integration works (no "TODO" responses)
5. ✅ Health check shows populated vector store

## Support

If you encounter issues:

1. Check the logs in `./data/logs/tutorgpt.log`
2. Verify API key permissions
3. Ensure book-source directory is present
4. Review error messages in ingestion output

## Performance Notes

- **Ingestion Time:** ~3-5 minutes (one-time setup)
- **Search Time:** <100ms (p95)
- **Startup Time:** <2 seconds
- **Memory Usage:** ~500MB (with embeddings loaded)
