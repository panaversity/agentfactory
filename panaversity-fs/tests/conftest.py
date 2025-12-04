"""Pytest configuration and fixtures for PanaversityFS tests."""

import pytest
import asyncio
import tempfile
import shutil
import os
from pathlib import Path

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
def setup_fs_backend(temp_storage_root):
    """Setup filesystem backend for testing."""
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
    """
    from panaversity_fs.storage import get_operator

    op = get_operator()

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
    await op.write("books/test-book/content/01-Part/01-Chapter/01-lesson.md", lesson.encode('utf-8'))

    # Create sample lesson summary (ADR-0018: .summary.md naming convention)
    summary = """# Lesson 1 Summary

Test summary content.
"""
    await op.write("books/test-book/content/01-Part/01-Chapter/01-lesson.summary.md", summary.encode('utf-8'))

    return {
        "book_id": "test-book",
        "lesson_path": "content/01-Part/01-Chapter/01-lesson.md",
        "summary_path": "content/01-Part/01-Chapter/01-lesson.summary.md"
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
