"""Tests for RAG search functionality."""

import pytest
from unittest.mock import Mock, patch, MagicMock
from rag.search import RoboLearnSearch
from models.search import SearchQuery
from core.config import get_settings


@pytest.fixture
def mock_qdrant_client():
    """Mock Qdrant client."""
    with patch('rag.search.QdrantClient') as mock_client:
        client_instance = MagicMock()
        mock_client.return_value = client_instance
        yield client_instance


@pytest.fixture
def mock_embedder():
    """Mock embedder."""
    with patch('rag.search.OpenAIEmbedder') as mock_embedder:
        embedder_instance = MagicMock()
        embedder_instance.embed.return_value = [0.1] * 1536  # Mock embedding vector
        mock_embedder.return_value = embedder_instance
        yield embedder_instance


@pytest.fixture
def search_client(mock_qdrant_client, mock_embedder):
    """Create search client with mocked dependencies."""
    return RoboLearnSearch()


def test_search_client_initialization(search_client):
    """Test search client initializes correctly."""
    assert search_client is not None
    assert hasattr(search_client, 'client')
    assert hasattr(search_client, 'embedder')
    assert hasattr(search_client, 'collection_name')


def test_search_basic_query(search_client, mock_qdrant_client):
    """Test basic search query."""
    # Mock Qdrant response
    mock_point = MagicMock()
    mock_point.score = 0.85
    mock_point.payload = {
        "text": "Test content",
        "source_file": "module-1-ros2/chapter-4/test.md",
        "module": "ros2",
        "chapter": 4,
        "lesson": 1,
        "lesson_title": "Test Lesson",
        "section_title": "Test Section",
        "hardware_tier": 1,
        "proficiency_level": "A2",
        "layer": "L1",
    }
    
    mock_response = MagicMock()
    mock_response.points = [mock_point]
    mock_qdrant_client.query_points.return_value = mock_response
    
    # Create search query
    settings = get_settings()
    query = SearchQuery(
        text="test query",
        book_id=settings.book_id,
        limit=5
    )
    
    # Execute search
    result = search_client.search(query)
    
    # Verify
    assert result is not None
    assert result.query == "test query"
    assert len(result.results) == 1
    assert result.results[0].score == 0.85
    assert result.results[0].text == "Test content"
    assert result.results[0].module == "ros2"
    assert result.results[0].citation is not None
    assert result.results[0].lesson_url is not None


def test_search_with_hardware_tier_filter(search_client, mock_qdrant_client):
    """Test search with hardware tier filter."""
    mock_response = MagicMock()
    mock_response.points = []
    mock_qdrant_client.query_points.return_value = mock_response
    
    settings = get_settings()
    query = SearchQuery(
        text="test",
        book_id=settings.book_id,
        hardware_tier_filter=2,
        limit=5
    )
    
    result = search_client.search(query)
    
    # Verify filter was applied
    call_args = mock_qdrant_client.query_points.call_args
    assert call_args is not None
    # Check that filter conditions were created
    filter_obj = call_args.kwargs.get('query_filter')
    assert filter_obj is not None


def test_search_with_module_filter(search_client, mock_qdrant_client):
    """Test search with module filter."""
    mock_response = MagicMock()
    mock_response.points = []
    mock_qdrant_client.query_points.return_value = mock_response
    
    settings = get_settings()
    query = SearchQuery(
        text="test",
        book_id=settings.book_id,
        module_filter="ros2",
        limit=5
    )
    
    result = search_client.search(query)
    
    # Verify filter was applied
    call_args = mock_qdrant_client.query_points.call_args
    filter_obj = call_args.kwargs.get('query_filter')
    assert filter_obj is not None


def test_search_with_chapter_range(search_client, mock_qdrant_client):
    """Test search with chapter range filter."""
    mock_response = MagicMock()
    mock_response.points = []
    mock_qdrant_client.query_points.return_value = mock_response
    
    settings = get_settings()
    query = SearchQuery(
        text="test",
        book_id=settings.book_id,
        chapter_min=1,
        chapter_max=5,
        limit=5
    )
    
    result = search_client.search(query)
    
    # Verify filter was applied
    call_args = mock_qdrant_client.query_points.call_args
    filter_obj = call_args.kwargs.get('query_filter')
    assert filter_obj is not None


def test_search_with_lesson_filter(search_client, mock_qdrant_client):
    """Test search with lesson filter."""
    mock_response = MagicMock()
    mock_response.points = []
    mock_qdrant_client.query_points.return_value = mock_response
    
    settings = get_settings()
    query = SearchQuery(
        text="test",
        book_id=settings.book_id,
        lesson_filter=3,
        limit=5
    )
    
    result = search_client.search(query)
    
    # Verify filter was applied
    call_args = mock_qdrant_client.query_points.call_args
    filter_obj = call_args.kwargs.get('query_filter')
    assert filter_obj is not None


def test_search_handles_empty_results(search_client, mock_qdrant_client):
    """Test search handles empty results gracefully."""
    mock_response = MagicMock()
    mock_response.points = []
    mock_qdrant_client.query_points.return_value = mock_response
    
    settings = get_settings()
    query = SearchQuery(
        text="nonexistent query that returns nothing",
        book_id=settings.book_id,
        limit=5
    )
    
    result = search_client.search(query)
    
    assert result is not None
    assert len(result.results) == 0
    assert result.total_found == 0


def test_search_generates_citation(search_client, mock_qdrant_client):
    """Test that search results include citations."""
    mock_point = MagicMock()
    mock_point.score = 0.9
    mock_point.payload = {
        "text": "Content",
        "source_file": "module-1-ros2/chapter-4/01-test.md",
        "module": "ros2",
        "chapter": 4,
        "lesson": 1,
        "lesson_title": "Test Lesson Title",
        "section_title": "Test Section",
        "hardware_tier": 1,
        "proficiency_level": "A2",
        "layer": "L1",
    }
    
    mock_response = MagicMock()
    mock_response.points = [mock_point]
    mock_qdrant_client.query_points.return_value = mock_response
    
    settings = get_settings()
    query = SearchQuery(text="test", book_id=settings.book_id, limit=1)
    result = search_client.search(query)
    
    assert len(result.results) == 1
    citation = result.results[0].citation
    assert citation is not None
    assert "Test Lesson Title" in citation or "Lesson 4.1" in citation
    assert "Module ROS2" in citation


def test_search_generates_lesson_url(search_client, mock_qdrant_client):
    """Test that search results include lesson URLs."""
    mock_point = MagicMock()
    mock_point.score = 0.9
    mock_point.payload = {
        "text": "Content",
        "source_file": "module-1-ros2/chapter-4/01-test.md",
        "module": "ros2",
        "chapter": 4,
        "lesson": 1,
        "hardware_tier": 1,
        "proficiency_level": "A2",
        "layer": "L1",
    }
    
    mock_response = MagicMock()
    mock_response.points = [mock_point]
    mock_qdrant_client.query_points.return_value = mock_response
    
    settings = get_settings()
    query = SearchQuery(text="test", book_id=settings.book_id, limit=1)
    result = search_client.search(query)
    
    assert len(result.results) == 1
    url = result.results[0].lesson_url
    assert url is not None
    assert url.startswith(settings.frontend_base_url)
    assert "/docs/" in url
    assert "module-1-ros2" in url

