"""Pytest configuration and fixtures for PanaversityFS tests.

Updated for FR-002: Journal-backed content operations.
"""

import pytest
import asyncio
import tempfile
import shutil
import os
from pathlib import Path
from datetime import datetime, timezone

# Add src to path
import sys
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))


@pytest.fixture(scope="session")
def event_loop():
    """Create event loop for async tests."""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


@pytest.fixture
def temp_storage_root():
    """Create temporary storage directory."""
    temp_dir = tempfile.mkdtemp(prefix="panaversity-test-")
    yield temp_dir
    shutil.rmtree(temp_dir, ignore_errors=True)


@pytest.fixture
async def setup_db():
    """Setup in-memory SQLite database for testing."""
    from sqlalchemy.ext.asyncio import create_async_engine, async_sessionmaker
    from panaversity_fs.database.models import Base
    from panaversity_fs.database import connection

    # Use in-memory SQLite for tests
    test_engine = create_async_engine(
        "sqlite+aiosqlite:///:memory:",
        echo=False
    )

    # Create all tables
    async with test_engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)

    # Create session factory
    test_factory = async_sessionmaker(
        test_engine,
        expire_on_commit=False
    )

    # Monkey-patch the connection module to use test factory
    original_factory = connection._session_factory
    connection._session_factory = test_factory

    yield test_factory

    # Cleanup
    connection._session_factory = original_factory
    await test_engine.dispose()


@pytest.fixture
async def setup_fs_backend(temp_storage_root, setup_db):
    """Setup filesystem backend for testing with database.

    FR-002: Includes database setup for journal-backed operations.
    """
    os.environ['PANAVERSITY_STORAGE_BACKEND'] = 'fs'
    os.environ['PANAVERSITY_STORAGE_ROOT'] = temp_storage_root

    # Clear any cached config/operator
    from panaversity_fs import storage, config
    storage._operator = None
    config._config = None

    yield temp_storage_root

    # Cleanup
    del os.environ['PANAVERSITY_STORAGE_BACKEND']
    del os.environ['PANAVERSITY_STORAGE_ROOT']
    storage._operator = None
    config._config = None


@pytest.fixture
async def sample_book_data(setup_fs_backend):
    """Create sample book data for testing.

    ADR-0018: Updated to use Docusaurus-aligned content/ structure.
    FR-002: Creates journal entries alongside storage files.

    Note: setup_fs_backend includes setup_db, so database is already configured.
    """
    from panaversity_fs.storage import get_operator
    from panaversity_fs.storage_utils import compute_sha256
    from panaversity_fs.database.models import FileJournal
    from panaversity_fs.database.connection import get_session_factory

    op = get_operator()
    session_factory = get_session_factory()

    # Create registry
    registry = """books:
  - book_id: test-book
    title: Test Book
    storage_backend: fs
    created_at: "2025-01-01T00:00:00Z"
    status: active
"""
    await op.write("registry.yaml", registry.encode('utf-8'))

    # Create sample lesson (ADR-0018: content/ structure)
    lesson = """---
title: Test Lesson
chapter: 1
---

# Test Lesson

This is a test lesson with OpenDAL references.

```python
def test():
    return "Hello"
```
"""
    lesson_bytes = lesson.encode('utf-8')
    lesson_hash = compute_sha256(lesson_bytes)
    await op.write("books/test-book/content/01-Part/01-Chapter/01-lesson.md", lesson_bytes)

    # Create sample lesson summary (ADR-0018: .summary.md naming convention)
    summary = """# Lesson 1 Summary

Test summary content.
"""
    summary_bytes = summary.encode('utf-8')
    summary_hash = compute_sha256(summary_bytes)
    await op.write("books/test-book/content/01-Part/01-Chapter/01-lesson.summary.md", summary_bytes)

    # Create journal entries (FR-002)
    async with session_factory() as session:
        session.add(FileJournal(
            book_id="test-book",
            path="content/01-Part/01-Chapter/01-lesson.md",
            user_id="__base__",
            sha256=lesson_hash,
            last_written_at=datetime.now(timezone.utc),
            storage_backend="fs"
        ))
        session.add(FileJournal(
            book_id="test-book",
            path="content/01-Part/01-Chapter/01-lesson.summary.md",
            user_id="__base__",
            sha256=summary_hash,
            last_written_at=datetime.now(timezone.utc),
            storage_backend="fs"
        ))
        await session.commit()

    return {
        "book_id": "test-book",
        "lesson_path": "content/01-Part/01-Chapter/01-lesson.md",
        "summary_path": "content/01-Part/01-Chapter/01-lesson.summary.md",
        "lesson_hash": lesson_hash,
        "summary_hash": summary_hash
    }


@pytest.fixture
def sample_lesson_content():
    """Sample lesson markdown content."""
    return """---
title: Python Basics
chapter: 1
lesson: 1
---

# Lesson 1: Python Basics

Learn Python fundamentals.

```python
def hello():
    print("Hello, World!")
```
"""


@pytest.fixture
def sample_summary_content():
    """Sample summary markdown content."""
    return """# Chapter Summary

Key concepts covered in this chapter.
"""
