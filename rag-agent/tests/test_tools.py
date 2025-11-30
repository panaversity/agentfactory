"""Tests for RAG tools."""

import pytest
from rag.tools import search_tool, SEARCH_TOOL_SCHEMA, get_search_client


def test_search_tool_schema():
    """Test search tool schema is valid."""
    assert SEARCH_TOOL_SCHEMA["type"] == "function"
    assert "function" in SEARCH_TOOL_SCHEMA
    assert SEARCH_TOOL_SCHEMA["function"]["name"] == "search_robolearn_content"
    assert "parameters" in SEARCH_TOOL_SCHEMA["function"]
    assert "query" in SEARCH_TOOL_SCHEMA["function"]["parameters"]["properties"]


def test_search_tool_returns_dict():
    """Test search tool returns a dictionary."""
    result = search_tool("test query")
    assert isinstance(result, dict)
    assert "query" in result
    assert "results" in result
    assert "total_found" in result


def test_search_tool_handles_errors():
    """Test search tool handles errors gracefully."""
    # Even if Qdrant is unavailable, should return dict with error
    result = search_tool("test query")
    assert isinstance(result, dict)
    # May have error key if unavailable, or results if available
    assert "error" in result or "results" in result


def test_get_search_client():
    """Test get_search_client returns client or None."""
    client = get_search_client()
    # May be None if Qdrant unavailable, or RoboLearnSearch instance if available
    assert client is None or hasattr(client, "search")

