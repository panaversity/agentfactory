# TutorGPT MVP - Quick Start Guide

**Date**: 2025-01-08
**Purpose**: Get TutorGPT running locally for development
**Estimated Time**: 15 minutes

---

## Prerequisites

### Required Software

1. **Python 3.11+**
   ```bash
   python --version  # Should be 3.11 or higher
   ```

2. **UV Package Manager** (Recommended)
   ```bash
   # Install UV
   curl -LsSf https://astral.sh/uv/install.sh | sh

   # Or on Windows (PowerShell)
   powershell -c "irm https://astral.sh/uv/install.ps1 | iex"

   # Verify installation
   uv --version
   ```

3. **Node.js 18+** (for Docusaurus/ChatKit)
   ```bash
   node --version  # Should be 18 or higher
   npm --version
   ```

4. **Git**
   ```bash
   git --version
   ```

### Required API Keys

1. **OpenAI API Key**
   - Get from: https://platform.openai.com/api-keys
   - Used for: ChatKit, GPT-4 agent

2. **Google AI API Key**
   - Get from: https://aistudio.google.com/app/apikey
   - Used for: Gemini embeddings

---

## Quick Start (5 Minutes)

### 1. Clone Repository

```bash
cd "P:\Ai native Book\ai-native-software-development\Tutor"
```

### 2. Set Up Backend

```bash
# Create .env file with your API keys
cat > .env << EOF
OPENAI_API_KEY=your_openai_api_key_here
GOOGLE_API_KEY=your_google_ai_api_key_here
ENVIRONMENT=development
LOG_LEVEL=INFO
EOF

# Initialize UV project and install dependencies
uv init
uv add fastapi uvicorn[standard] google-genai chromadb aiosqlite openai pydantic python-dotenv

# Or use requirements.txt (if created)
uv pip install -r requirements.txt
```

### 3. Initialize Database & Embeddings

```bash
# Create data directories
mkdir -p data/embeddings
mkdir -p data/database

# Run initialization script (will be created in implementation)
uv run python scripts/init_database.py
```

### 4. Index Book Content

```bash
# Index the book content into ChromaDB
# This will chunk and embed all lessons
uv run python scripts/index_book_content.py

# Expected output:
# ✓ Found 107 lesson files
# ✓ Created 2,543 chunks
# ✓ Generated embeddings (this may take 5-10 minutes)
# ✓ Indexed to ChromaDB
# ✓ Done! Ready to search.
```

### 5. Start Backend Server

```bash
# Run with UV
uv run uvicorn app.main:app --reload --port 8000

# Or activate virtual environment first
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
python -m uvicorn app.main:app --reload --port 8000

# Server should start at: http://localhost:8000
# API docs at: http://localhost:8000/docs
```

### 6. Test Backend

```bash
# Health check
curl http://localhost:8000/health

# Expected response:
# {
#   "status": "healthy",
#   "version": "1.0.0",
#   "services": {
#     "database": "connected",
#     "vectorstore": "connected",
#     "embeddings": "connected"
#   }
# }

# Test RAG search
curl -X POST http://localhost:8000/api/rag/search \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is Python?",
    "scope": "entire_book",
    "n_results": 3
  }'
```

---

## Development Workflow

### Backend Development

```bash
# Run server with auto-reload
uv run uvicorn app.main:app --reload --port 8000

# Run tests
uv run pytest tests/ -v

# Run linter
uv run ruff check app/

# Format code
uv run black app/

# Type checking
uv run mypy app/
```

### Frontend Integration (Docusaurus)

```bash
# Navigate to book site directory
cd ../../../book-source/learn-ai-native-software-development-for-beginners

# Install dependencies
npm install
npm install @openai/chatkit-react

# Add ChatKit widget (see integration guide)
# Edit: src/theme/Root.tsx

# Start dev server
npm start

# Site should open at: http://localhost:3000
```

---

## Project Structure

```
Tutor/
├── .env                      # Environment variables (DO NOT COMMIT)
├── .env.example              # Example environment file
├── pyproject.toml            # UV project configuration
├── uv.lock                   # UV lock file
├── README.md                 # Project README
│
├── app/                      # FastAPI application
│   ├── main.py               # Application entry point
│   ├── config.py             # Configuration management
│   │
│   ├── api/                  # API endpoints
│   │   ├── chatkit.py        # ChatKit session endpoints
│   │   ├── rag.py            # RAG search endpoints
│   │   └── profile.py        # Profile endpoints
│   │
│   ├── models/               # Pydantic models
│   │   ├── requests.py       # Request models
│   │   ├── responses.py      # Response models
│   │   └── database.py       # Database models
│   │
│   ├── services/             # Business logic
│   │   ├── rag_service.py    # RAG search logic
│   │   ├── session_service.py # Session management
│   │   ├── embeddings.py     # Gemini embeddings
│   │   └── agent_tools.py    # Agent tool implementations
│   │
│   ├── db/                   # Database layer
│   │   ├── sqlite.py         # SQLite client
│   │   └── chromadb.py       # ChromaDB client
│   │
│   └── utils/                # Utilities
│       ├── logging.py        # Logging setup
│       └── errors.py         # Error handling
│
├── data/                     # Data storage (gitignored)
│   ├── embeddings/           # ChromaDB persistence
│   └── database/             # SQLite database files
│
├── scripts/                  # Utility scripts
│   ├── init_database.py      # Initialize database schema
│   ├── index_book_content.py # Index book to ChromaDB
│   └── test_setup.py         # Verify setup
│
├── tests/                    # Test suite
│   ├── test_api/             # API endpoint tests
│   ├── test_services/        # Service tests
│   └── conftest.py           # Pytest configuration
│
└── specs/                    # Specifications
    └── 001-tutorgpt-mvp/     # MVP spec
        ├── spec.md           # Requirements
        ├── plan.md           # Architecture plan
        ├── data-model.md     # Data models
        ├── research.md       # Technology research
        ├── quickstart.md     # This file
        └── contracts/        # API contracts
```

---

## Configuration

### Environment Variables

Create `.env` file in project root:

```bash
# API Keys (REQUIRED)
OPENAI_API_KEY=sk-...
GOOGLE_API_KEY=AI...

# Environment
ENVIRONMENT=development  # development, staging, production
LOG_LEVEL=INFO          # DEBUG, INFO, WARNING, ERROR

# Server
HOST=0.0.0.0
PORT=8000

# Database
DATABASE_PATH=./data/database/tutorgpt.db
EMBEDDINGS_PATH=./data/embeddings

# CORS (comma-separated)
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:8000

# Book Content
BOOK_SOURCE_PATH=../../../book-source/learn-ai-native-software-development-for-beginners

# Embeddings
GEMINI_MODEL=gemini-embedding-001
EMBEDDING_DIMENSION=768  # or 3072 for max quality
CHUNK_SIZE=512
CHUNK_OVERLAP=50

# Rate Limiting
RATE_LIMIT_CHATKIT_SESSION=10  # requests per minute
RATE_LIMIT_RAG_SEARCH=60       # requests per minute
RATE_LIMIT_PROFILE_UPDATE=30   # requests per minute
```

### UV Configuration (pyproject.toml)

```toml
[project]
name = "tutorgpt-mvp"
version = "1.0.0"
description = "AI tutor for AI-Native Software Development book"
requires-python = ">=3.11"

dependencies = [
    "fastapi>=0.104.0",
    "uvicorn[standard]>=0.24.0",
    "google-genai>=0.1.0",
    "chromadb>=0.4.0",
    "aiosqlite>=0.19.0",
    "openai>=1.0.0",
    "pydantic>=2.5.0",
    "python-dotenv>=1.0.0",
]

[project.optional-dependencies]
dev = [
    "pytest>=7.4.0",
    "pytest-asyncio>=0.21.0",
    "httpx>=0.25.0",
    "ruff>=0.1.0",
    "black>=23.0.0",
    "mypy>=1.7.0",
]

[tool.uv]
dev-dependencies = [
    "pytest>=7.4.0",
    "pytest-asyncio>=0.21.0",
    "httpx>=0.25.0",
    "ruff>=0.1.0",
    "black>=23.0.0",
    "mypy>=1.7.0",
]
```

---

## Common Commands

### Using UV

```bash
# Install all dependencies
uv sync

# Add a new package
uv add package-name

# Add dev dependency
uv add --dev package-name

# Run Python script
uv run python script.py

# Run FastAPI server
uv run uvicorn app.main:app --reload

# Run tests
uv run pytest

# Update dependencies
uv sync --upgrade
```

### Database Management

```bash
# Initialize database
uv run python scripts/init_database.py

# Reset database (WARNING: deletes all data)
uv run python scripts/reset_database.py

# Backup database
cp data/database/tutorgpt.db data/database/tutorgpt.backup.db

# View database
sqlite3 data/database/tutorgpt.db
```

### Embeddings Management

```bash
# Index book content
uv run python scripts/index_book_content.py

# Re-index specific chapter
uv run python scripts/index_book_content.py --chapter 04-python

# Check embedding stats
uv run python scripts/check_embeddings.py

# Reset embeddings (WARNING: deletes all embeddings)
rm -rf data/embeddings/*
uv run python scripts/index_book_content.py
```

---

## Troubleshooting

### Common Issues

**1. Import Error: No module named 'google.genai'**
```bash
# Solution: Install with UV
uv add google-genai
```

**2. ChromaDB PersistentClient Error**
```bash
# Solution: Create embeddings directory
mkdir -p data/embeddings
```

**3. SQLite Database Locked**
```bash
# Solution: Close all database connections
# Or restart the server
```

**4. Rate Limit Errors from Gemini**
```bash
# Solution: Implement exponential backoff in embeddings.py
# Or reduce batch size in indexing script
```

**5. ChatKit Widget Not Rendering**
```bash
# Solution: Check OpenAI domain whitelist
# Go to: https://platform.openai.com/settings/organization/chatkit
# Add your domain: http://localhost:3000
```

---

## Next Steps

After getting the server running:

1. **Test all endpoints** - Use FastAPI docs at http://localhost:8000/docs
2. **Integrate ChatKit** - Add widget to Docusaurus site
3. **Test RAG search** - Query book content
4. **Check logs** - Monitor app.log for issues
5. **Run tests** - Ensure everything works: `uv run pytest`

---

## Development Tips

### Using UV Effectively

```bash
# Create virtual environment
uv venv

# Activate environment
source .venv/bin/activate  # Linux/Mac
.venv\Scripts\activate     # Windows

# Run commands in environment
uv run <command>

# Or activate once and run directly
source .venv/bin/activate
python app/main.py
```

### FastAPI Development

- **Auto-reload**: Changes are picked up automatically
- **API Docs**: http://localhost:8000/docs (Swagger UI)
- **ReDoc**: http://localhost:8000/redoc (alternative docs)
- **Debug**: Use `LOG_LEVEL=DEBUG` in .env

### Database Inspection

```bash
# Open SQLite database
sqlite3 data/database/tutorgpt.db

# Useful commands:
.tables                      # List tables
.schema student_sessions     # Show table schema
SELECT * FROM student_sessions LIMIT 5;  # View data
.quit                        # Exit
```

### ChromaDB Inspection

```python
# In Python REPL
import chromadb

client = chromadb.PersistentClient(path="./data/embeddings")
collection = client.get_collection("book_content")

print(f"Total chunks: {collection.count()}")
print(f"Sample: {collection.peek()}")
```

---

## Production Deployment (Future)

### Environment Setup

```bash
# Set production environment
export ENVIRONMENT=production
export LOG_LEVEL=WARNING

# Use production database
export DATABASE_PATH=/var/lib/tutorgpt/tutorgpt.db

# Set allowed origins
export ALLOWED_ORIGINS=https://book.example.com
```

### Run with Gunicorn

```bash
# Install gunicorn
uv add gunicorn

# Run with workers
gunicorn app.main:app -w 4 -k uvicorn.workers.UvicornWorker --bind 0.0.0.0:8000
```

---

## Support

- **Spec**: See `specs/001-tutorgpt-mvp/spec.md`
- **API Contracts**: See `specs/001-tutorgpt-mvp/contracts/`
- **Data Model**: See `specs/001-tutorgpt-mvp/data-model.md`
- **Issues**: Create issue in repository

---

**Ready to build!** 🚀

Continue to implementation phase (`/sp.tasks`) to generate step-by-step tasks.
