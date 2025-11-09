"""
Pytest configuration and shared fixtures for TutorGPT tests.

This file contains:
- Pytest configuration
- Shared fixtures for all test types
- Mock objects for testing
"""

import pytest
import sys
from pathlib import Path

# Add app directory to Python path for imports
backend_dir = Path(__file__).parent.parent
sys.path.insert(0, str(backend_dir))


@pytest.fixture
def mock_openai_api_key(monkeypatch):
    """Mock OpenAI API key for tests."""
    monkeypatch.setenv("OPENAI_API_KEY", "test-openai-key-12345")
    return "test-openai-key-12345"


@pytest.fixture
def mock_google_api_key(monkeypatch):
    """Mock Google API key for tests."""
    monkeypatch.setenv("GOOGLE_API_KEY", "test-google-key-12345")
    return "test-google-key-12345"


@pytest.fixture
def test_config(mock_openai_api_key, mock_google_api_key):
    """Provide test configuration with mocked API keys."""
    return {
        "openai_api_key": mock_openai_api_key,
        "google_api_key": mock_google_api_key,
        "environment": "test",
        "database_path": ":memory:",  # In-memory SQLite for tests
        "chromadb_path": "./data/test_embeddings",
    }
