# Data Model: TutorGPT MVP

**Date**: 2025-01-08
**Purpose**: Define all data structures, schemas, and models for TutorGPT
**Status**: Design Phase

---

## Overview

This document defines the complete data model for TutorGPT, including:
- Database schemas (SQLite)
- Vector store structure (ChromaDB)
- API request/response models (Pydantic)
- Agent state models (OpenAI Agents SDK)

---

## 1. Database Schema (SQLite)

### 1.1 Student Sessions Table

Stores persistent student session information.

```sql
CREATE TABLE student_sessions (
    -- Primary Key
    session_id TEXT PRIMARY KEY,

    -- Student Information
    student_name TEXT,
    student_email TEXT,

    -- Session Metadata
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    last_active_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

    -- Learning Context
    current_chapter TEXT,
    current_lesson TEXT,
    learning_level TEXT CHECK(learning_level IN ('beginner', 'intermediate', 'advanced')),

    -- Session State
    total_messages INTEGER DEFAULT 0,
    total_rag_queries INTEGER DEFAULT 0,
    is_active BOOLEAN DEFAULT 1,

    -- Indexes
    UNIQUE(student_email)
);

CREATE INDEX idx_sessions_email ON student_sessions(student_email);
CREATE INDEX idx_sessions_active ON student_sessions(is_active, last_active_at);
```

**Pydantic Model**:
```python
from pydantic import BaseModel, EmailStr, Field
from datetime import datetime
from typing import Literal

class StudentSession(BaseModel):
    """Student session model."""
    session_id: str = Field(..., description="Unique session identifier")
    student_name: str | None = None
    student_email: EmailStr | None = None
    created_at: datetime = Field(default_factory=datetime.utcnow)
    last_active_at: datetime = Field(default_factory=datetime.utcnow)
    current_chapter: str | None = None
    current_lesson: str | None = None
    learning_level: Literal['beginner', 'intermediate', 'advanced'] = 'beginner'
    total_messages: int = 0
    total_rag_queries: int = 0
    is_active: bool = True

    class Config:
        json_schema_extra = {
            "example": {
                "session_id": "sess_abc123xyz",
                "student_name": "Jane Doe",
                "student_email": "jane@example.com",
                "current_chapter": "04-python",
                "current_lesson": "01-intro",
                "learning_level": "intermediate",
                "total_messages": 25,
                "total_rag_queries": 8,
                "is_active": True
            }
        }
```

---

### 1.2 Interaction History Table

Tracks all student interactions for analytics and personalization.

```sql
CREATE TABLE interaction_history (
    -- Primary Key
    interaction_id INTEGER PRIMARY KEY AUTOINCREMENT,

    -- Foreign Key
    session_id TEXT NOT NULL,

    -- Interaction Details
    interaction_type TEXT CHECK(interaction_type IN ('question', 'explanation_request', 'rag_search', 'feedback')),
    user_message TEXT,
    agent_response TEXT,

    -- Context
    page_path TEXT,
    highlighted_text TEXT,
    current_chapter TEXT,
    current_lesson TEXT,

    -- RAG Information
    rag_query TEXT,
    rag_results_count INTEGER,
    rag_scope TEXT CHECK(rag_scope IN ('current_lesson', 'current_chapter', 'entire_book')),

    -- Metadata
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    response_time_ms INTEGER,
    tokens_used INTEGER,

    -- Foreign Key Constraint
    FOREIGN KEY (session_id) REFERENCES student_sessions(session_id) ON DELETE CASCADE
);

CREATE INDEX idx_interactions_session ON interaction_history(session_id, timestamp);
CREATE INDEX idx_interactions_type ON interaction_history(interaction_type);
CREATE INDEX idx_interactions_timestamp ON interaction_history(timestamp);
```

**Pydantic Model**:
```python
class InteractionHistory(BaseModel):
    """Interaction history model."""
    interaction_id: int | None = None
    session_id: str
    interaction_type: Literal['question', 'explanation_request', 'rag_search', 'feedback']
    user_message: str | None = None
    agent_response: str | None = None
    page_path: str | None = None
    highlighted_text: str | None = None
    current_chapter: str | None = None
    current_lesson: str | None = None
    rag_query: str | None = None
    rag_results_count: int | None = None
    rag_scope: Literal['current_lesson', 'current_chapter', 'entire_book'] | None = None
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    response_time_ms: int | None = None
    tokens_used: int | None = None

    class Config:
        json_schema_extra = {
            "example": {
                "session_id": "sess_abc123xyz",
                "interaction_type": "question",
                "user_message": "What is async programming?",
                "agent_response": "Async programming allows...",
                "page_path": "/docs/chapter-04/lesson-03-async",
                "current_chapter": "04-python",
                "current_lesson": "03-async",
                "rag_query": "async programming python",
                "rag_results_count": 5,
                "rag_scope": "current_lesson",
                "response_time_ms": 1250,
                "tokens_used": 450
            }
        }
```

---

### 1.3 Student Progress Table

Tracks student learning progress and mastery.

```sql
CREATE TABLE student_progress (
    -- Primary Key
    progress_id INTEGER PRIMARY KEY AUTOINCREMENT,

    -- Foreign Key
    session_id TEXT NOT NULL,

    -- Learning Progress
    chapter TEXT NOT NULL,
    lesson TEXT NOT NULL,

    -- Engagement Metrics
    visit_count INTEGER DEFAULT 1,
    total_time_seconds INTEGER DEFAULT 0,
    questions_asked INTEGER DEFAULT 0,

    -- Mastery Indicators
    understood BOOLEAN DEFAULT 0,
    needs_review BOOLEAN DEFAULT 0,
    completed BOOLEAN DEFAULT 0,

    -- Timestamps
    first_visited_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    last_visited_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

    -- Constraints
    FOREIGN KEY (session_id) REFERENCES student_sessions(session_id) ON DELETE CASCADE,
    UNIQUE(session_id, chapter, lesson)
);

CREATE INDEX idx_progress_session ON student_progress(session_id);
CREATE INDEX idx_progress_lesson ON student_progress(chapter, lesson);
```

**Pydantic Model**:
```python
class StudentProgress(BaseModel):
    """Student progress tracking model."""
    progress_id: int | None = None
    session_id: str
    chapter: str
    lesson: str
    visit_count: int = 1
    total_time_seconds: int = 0
    questions_asked: int = 0
    understood: bool = False
    needs_review: bool = False
    completed: bool = False
    first_visited_at: datetime = Field(default_factory=datetime.utcnow)
    last_visited_at: datetime = Field(default_factory=datetime.utcnow)

    class Config:
        json_schema_extra = {
            "example": {
                "session_id": "sess_abc123xyz",
                "chapter": "04-python",
                "lesson": "01-intro",
                "visit_count": 3,
                "total_time_seconds": 1800,
                "questions_asked": 7,
                "understood": True,
                "needs_review": False,
                "completed": True
            }
        }
```

---

## 2. Vector Store Schema (ChromaDB)

### 2.1 Book Content Collection

Stores embedded book content chunks with metadata.

**Collection Name**: `book_content`

**Collection Configuration**:
```python
import chromadb

client = chromadb.PersistentClient(path="./data/embeddings")

collection = client.get_or_create_collection(
    name="book_content",
    embedding_function=GeminiEmbeddingFunction(api_key="..."),
    metadata={
        "hnsw:space": "cosine",  # Distance metric
        "description": "AI-Native Software Development book content"
    }
)
```

**Document Structure**:
```python
{
    "id": "chunk_ch04_l01_001",  # Unique chunk ID
    "embedding": [0.1, 0.2, ..., 0.N],  # 768 or 3072 dimensions
    "metadata": {
        # Chapter/Lesson Info
        "chapter": "04-python",
        "chapter_number": 4,
        "chapter_title": "Python Fundamentals",
        "lesson": "01-intro",
        "lesson_number": 1,
        "lesson_title": "Introduction to Python",

        # File Info
        "file_path": "book-source/learn-ai-native-software-development-for-beginners/04-python/01-intro/readme.md",
        "file_name": "readme.md",

        # Content Info
        "chunk_index": 0,  # Position in lesson
        "chunk_size": 512,  # Tokens
        "content_type": "text",  # "text", "code", "heading"

        # Semantic Info
        "heading": "What is Python?",
        "topics": ["python", "introduction", "basics"],
        "difficulty": "beginner",

        # Timestamps
        "indexed_at": "2025-01-08T12:00:00Z",
        "updated_at": "2025-01-08T12:00:00Z"
    },
    "document": "Python is a high-level, interpreted programming language..."  # Actual text
}
```

**Pydantic Model**:
```python
from typing import List

class BookChunkMetadata(BaseModel):
    """Metadata for book content chunks."""
    chapter: str
    chapter_number: int
    chapter_title: str
    lesson: str
    lesson_number: int
    lesson_title: str
    file_path: str
    file_name: str
    chunk_index: int
    chunk_size: int
    content_type: Literal['text', 'code', 'heading']
    heading: str | None = None
    topics: List[str] = []
    difficulty: Literal['beginner', 'intermediate', 'advanced'] = 'beginner'
    indexed_at: datetime
    updated_at: datetime

class BookChunk(BaseModel):
    """Complete book chunk model."""
    id: str
    embedding: List[float]
    metadata: BookChunkMetadata
    document: str  # Actual text content

    class Config:
        json_schema_extra = {
            "example": {
                "id": "chunk_ch04_l01_001",
                "embedding": [0.1, 0.2, 0.3],  # Shortened for example
                "metadata": {
                    "chapter": "04-python",
                    "chapter_number": 4,
                    "chapter_title": "Python Fundamentals",
                    "lesson": "01-intro",
                    "lesson_number": 1,
                    "lesson_title": "Introduction to Python",
                    "file_path": "book-source/.../readme.md",
                    "chunk_index": 0,
                    "content_type": "text",
                    "topics": ["python", "basics"],
                    "difficulty": "beginner"
                },
                "document": "Python is a high-level programming language..."
            }
        }
```

---

## 3. API Models (FastAPI)

### 3.1 ChatKit Session Creation

**Request Model**:
```python
class ChatKitSessionRequest(BaseModel):
    """Request to create a ChatKit session."""
    # Page Context
    page_path: str | None = None
    page_title: str | None = None
    current_chapter: str | None = None
    current_lesson: str | None = None
    highlighted_text: str | None = None

    # Student Info (optional)
    student_email: EmailStr | None = None
    student_name: str | None = None

    # Session Preferences
    learning_level: Literal['beginner', 'intermediate', 'advanced'] | None = 'beginner'

    class Config:
        json_schema_extra = {
            "example": {
                "page_path": "/docs/chapter-04/lesson-01-intro",
                "page_title": "Introduction to Python",
                "current_chapter": "04-python",
                "current_lesson": "01-intro",
                "highlighted_text": "async def main():",
                "student_email": "jane@example.com",
                "learning_level": "intermediate"
            }
        }
```

**Response Model**:
```python
class ChatKitSessionResponse(BaseModel):
    """Response with ChatKit session credentials."""
    client_secret: str = Field(..., description="OpenAI ChatKit client secret")
    session_id: str = Field(..., description="Our internal session ID")
    expires_at: datetime | None = None

    class Config:
        json_schema_extra = {
            "example": {
                "client_secret": "sk_chatkit_abc123...",
                "session_id": "sess_xyz789",
                "expires_at": "2025-01-08T18:00:00Z"
            }
        }
```

---

### 3.2 RAG Search

**Request Model**:
```python
class RAGSearchRequest(BaseModel):
    """Request for RAG search."""
    query: str = Field(..., min_length=1, max_length=500)
    scope: Literal['current_lesson', 'current_chapter', 'entire_book'] = 'current_lesson'
    n_results: int = Field(default=5, ge=1, le=20)

    # Context (required for lesson/chapter scope)
    current_chapter: str | None = None
    current_lesson: str | None = None

    class Config:
        json_schema_extra = {
            "example": {
                "query": "How do I use async/await in Python?",
                "scope": "current_lesson",
                "n_results": 5,
                "current_chapter": "04-python",
                "current_lesson": "03-async"
            }
        }
```

**Response Model**:
```python
class RAGSearchResult(BaseModel):
    """Single RAG search result."""
    chunk_id: str
    content: str
    score: float = Field(..., ge=0.0, le=1.0, description="Similarity score")
    metadata: BookChunkMetadata

class RAGSearchResponse(BaseModel):
    """Response from RAG search."""
    query: str
    scope: str
    results: List[RAGSearchResult]
    total_results: int
    search_time_ms: int

    class Config:
        json_schema_extra = {
            "example": {
                "query": "How do I use async/await?",
                "scope": "current_lesson",
                "results": [
                    {
                        "chunk_id": "chunk_ch04_l03_005",
                        "content": "Async/await syntax allows...",
                        "score": 0.92,
                        "metadata": {"chapter": "04-python", "lesson": "03-async"}
                    }
                ],
                "total_results": 5,
                "search_time_ms": 45
            }
        }
```

---

### 3.3 Student Profile

**Response Model**:
```python
class StudentProfileResponse(BaseModel):
    """Student learning profile."""
    session_id: str
    student_name: str | None
    student_email: EmailStr | None
    learning_level: Literal['beginner', 'intermediate', 'advanced']

    # Current Context
    current_chapter: str | None
    current_lesson: str | None

    # Progress Summary
    total_messages: int
    total_lessons_visited: int
    completed_lessons: List[str] = []
    struggling_topics: List[str] = []

    # Recent Activity
    last_active_at: datetime
    recent_questions: List[str] = []

    class Config:
        json_schema_extra = {
            "example": {
                "session_id": "sess_abc123",
                "student_email": "jane@example.com",
                "learning_level": "intermediate",
                "current_chapter": "04-python",
                "current_lesson": "03-async",
                "total_messages": 47,
                "total_lessons_visited": 12,
                "completed_lessons": ["04-python/01-intro", "04-python/02-variables"],
                "struggling_topics": ["async programming", "decorators"],
                "last_active_at": "2025-01-08T14:30:00Z",
                "recent_questions": ["What is async/await?", "How do decorators work?"]
            }
        }
```

---

## 4. Agent Tool Models

### 4.1 Search Book Content Tool

```python
@function_tool
def search_book_content(
    query: str,
    scope: str = "entire_book",
    current_chapter: str | None = None,
    current_lesson: str | None = None
) -> str:
    """
    Search the book content using RAG.

    Args:
        query: Search query (what to look for)
        scope: Search scope - "current_lesson", "current_chapter", or "entire_book"
        current_chapter: Current chapter ID (required for lesson/chapter scope)
        current_lesson: Current lesson ID (required for lesson scope)

    Returns:
        Formatted search results with sources
    """
    # Implementation will format results as text for agent
    pass
```

**Tool Return Format** (for agent consumption):
```
Search Results (scope: current_lesson):

[1] Chapter 04 / Lesson 03 (Score: 0.92)
Async/await syntax in Python allows you to write concurrent code that looks
synchronous. The 'async def' keyword defines a coroutine function...
Source: book-source/.../03-async/readme.md

[2] Chapter 04 / Lesson 03 (Score: 0.87)
To use async/await, you need to understand the event loop. The asyncio
module provides the event loop implementation...
Source: book-source/.../03-async/readme.md

---
Found 5 results in 45ms
```

---

### 4.2 Get Student Profile Tool

```python
@function_tool
def get_student_profile(session_id: str) -> str:
    """
    Get student learning profile and progress.

    Args:
        session_id: Student session ID

    Returns:
        Formatted student profile information
    """
    pass
```

**Tool Return Format**:
```
Student Profile:
- Level: Intermediate
- Current Location: Chapter 04 (Python) / Lesson 03 (Async Programming)
- Total Messages: 47
- Lessons Completed: 12
- Recently Struggled With: async programming, decorators
- Last Active: 30 minutes ago
- Recent Questions:
  1. "What is async/await?"
  2. "How do decorators work?"
```

---

## 5. Chunking Strategy

### 5.1 Chunk Parameters

```python
class ChunkingConfig(BaseModel):
    """Configuration for text chunking."""
    chunk_size: int = 512  # Tokens per chunk
    chunk_overlap: int = 50  # Overlapping tokens
    separator: str = "\n\n"  # Split on paragraphs

    # Metadata extraction
    extract_headings: bool = True
    extract_code_blocks: bool = True
    preserve_structure: bool = True
```

### 5.2 Chunk ID Format

```
chunk_{chapter}_{lesson}_{index}

Examples:
- chunk_ch04_l01_000
- chunk_ch04_l01_001
- chunk_ch05_l03_012
```

---

## 6. Error Models

### 6.1 API Error Response

```python
class ErrorDetail(BaseModel):
    """Detailed error information."""
    code: str
    message: str
    field: str | None = None

class ErrorResponse(BaseModel):
    """Standard API error response."""
    error: str
    details: List[ErrorDetail] = []
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    request_id: str | None = None

    class Config:
        json_schema_extra = {
            "example": {
                "error": "Invalid session",
                "details": [
                    {
                        "code": "SESSION_NOT_FOUND",
                        "message": "Session sess_abc123 does not exist"
                    }
                ],
                "timestamp": "2025-01-08T14:30:00Z",
                "request_id": "req_xyz789"
            }
        }
```

---

## Summary

### Database Tables
1. ✅ `student_sessions` - Session management
2. ✅ `interaction_history` - All interactions
3. ✅ `student_progress` - Learning progress

### Vector Store Collections
1. ✅ `book_content` - Embedded book chunks

### API Models
1. ✅ ChatKit session creation (request/response)
2. ✅ RAG search (request/response)
3. ✅ Student profile (response)
4. ✅ Error responses

### Agent Tools
1. ✅ `search_book_content` - RAG tool
2. ✅ `get_student_profile` - Profile tool

All models are defined with:
- Type safety (Pydantic)
- Validation rules
- Example data
- Clear documentation

Ready for implementation in Phase 2 (Tasks).
