"""Tests for Pydantic models."""

import pytest
from pydantic import ValidationError
from models.search import SearchQuery, SearchResult, SearchResponse
from models.document import DocumentMetadata, DocumentChunk, EmbeddedChunk


class TestSearchQuery:
    """Tests for SearchQuery model."""
    
    def test_valid_query(self):
        """Test valid search query."""
        query = SearchQuery(
            text="test query",
            book_id="physical-ai-robotics"
        )
        assert query.text == "test query"
        assert query.book_id == "physical-ai-robotics"
        assert query.limit == 5  # default
    
    def test_query_min_length(self):
        """Test query must be at least 3 characters."""
        with pytest.raises(ValidationError):
            SearchQuery(text="ab", book_id="test")
    
    def test_hardware_tier_range(self):
        """Test hardware tier must be 1-4."""
        # Valid
        query = SearchQuery(text="test", book_id="test", hardware_tier_filter=1)
        assert query.hardware_tier_filter == 1
        
        query = SearchQuery(text="test", book_id="test", hardware_tier_filter=4)
        assert query.hardware_tier_filter == 4
        
        # Invalid
        with pytest.raises(ValidationError):
            SearchQuery(text="test", book_id="test", hardware_tier_filter=0)
        
        with pytest.raises(ValidationError):
            SearchQuery(text="test", book_id="test", hardware_tier_filter=5)
    
    def test_module_filter_enum(self):
        """Test module filter must be valid module."""
        valid_modules = ["ros2", "gazebo", "isaac", "vla"]
        for module in valid_modules:
            query = SearchQuery(text="test", book_id="test", module_filter=module)
            assert query.module_filter == module
        
        # Invalid module should raise error
        with pytest.raises(ValidationError):
            SearchQuery(text="test", book_id="test", module_filter="invalid")
    
    def test_chapter_range(self):
        """Test chapter range validation."""
        query = SearchQuery(
            text="test",
            book_id="test",
            chapter_min=1,
            chapter_max=5
        )
        assert query.chapter_min == 1
        assert query.chapter_max == 5
        
        # Invalid range
        with pytest.raises(ValidationError):
            SearchQuery(text="test", book_id="test", chapter_min=-1)
        
        with pytest.raises(ValidationError):
            SearchQuery(text="test", book_id="test", chapter_max=25)
    
    def test_limit_range(self):
        """Test limit must be 1-20."""
        query = SearchQuery(text="test", book_id="test", limit=1)
        assert query.limit == 1
        
        query = SearchQuery(text="test", book_id="test", limit=20)
        assert query.limit == 20
        
        with pytest.raises(ValidationError):
            SearchQuery(text="test", book_id="test", limit=0)
        
        with pytest.raises(ValidationError):
            SearchQuery(text="test", book_id="test", limit=21)


class TestSearchResult:
    """Tests for SearchResult model."""
    
    def test_valid_result(self):
        """Test valid search result."""
        result = SearchResult(
            text="Test content",
            score=0.85,
            source_file="module-1-ros2/chapter-4/test.md",
            section_title=None,
            module="ros2",
            chapter=4,
            lesson=1,
            hardware_tier=1,
            proficiency_level="A2",
            layer="L1"
        )
        assert result.text == "Test content"
        assert result.score == 0.85
        assert result.module == "ros2"
        assert result.chapter == 4
        assert result.lesson == 1
    
    def test_score_range(self):
        """Test score must be 0.0-1.0."""
        result = SearchResult(
            text="test",
            score=0.0,
            source_file="test.md",
            section_title=None,
            module="ros2",
            chapter=1,
            lesson=1,
            hardware_tier=1,
            proficiency_level="A2",
            layer="L1"
        )
        assert result.score == 0.0
        
        result = SearchResult(
            text="test",
            score=1.0,
            source_file="test.md",
            section_title=None,
            module="ros2",
            chapter=1,
            lesson=1,
            hardware_tier=1,
            proficiency_level="A2",
            layer="L1"
        )
        assert result.score == 1.0
        
        with pytest.raises(ValidationError):
            SearchResult(
                text="test",
                score=-0.1,
                source_file="test.md",
                section_title=None,
                module="ros2",
                chapter=1,
                lesson=1,
                hardware_tier=1,
                proficiency_level="A2",
                layer="L1"
            )
        
        with pytest.raises(ValidationError):
            SearchResult(
                text="test",
                score=1.1,
                source_file="test.md",
                section_title=None,
                module="ros2",
                chapter=1,
                lesson=1,
                hardware_tier=1,
                proficiency_level="A2",
                layer="L1"
            )
    
    def test_optional_fields(self):
        """Test optional fields."""
        result = SearchResult(
            text="test",
            score=0.5,
            source_file="test.md",
            module="ros2",
            chapter=1,
            lesson=1,
            hardware_tier=1,
            proficiency_level="A2",
            layer="L1",
            section_title="Test Section",
            lesson_title="Test Lesson",
            citation="Test Citation",
            lesson_url="https://example.com/test"
        )
        assert result.section_title == "Test Section"
        assert result.lesson_title == "Test Lesson"
        assert result.citation == "Test Citation"
        assert result.lesson_url == "https://example.com/test"


class TestSearchResponse:
    """Tests for SearchResponse model."""
    
    def test_valid_response(self):
        """Test valid search response."""
        result = SearchResult(
            text="test",
            score=0.5,
            source_file="test.md",
            section_title=None,
            module="ros2",
            chapter=1,
            lesson=1,
            hardware_tier=1,
            proficiency_level="A2",
            layer="L1"
        )
        
        response = SearchResponse(
            query="test query",
            results=[result],
            total_found=1,
            book_id="physical-ai-robotics",
            module_filter=None
        )
        assert response.query == "test query"
        assert len(response.results) == 1
        assert response.total_found == 1
        assert response.book_id == "physical-ai-robotics"
    
    def test_empty_results(self):
        """Test response with no results."""
        response = SearchResponse(
            query="test",
            results=[],
            total_found=0,
            book_id="test",
            module_filter=None
        )
        assert len(response.results) == 0
        assert response.total_found == 0
    
    def test_total_found_validation(self):
        """Test total_found must be >= 0."""
        response = SearchResponse(
            query="test",
            results=[],
            total_found=0,
            book_id="test",
            module_filter=None
        )
        assert response.total_found == 0
        
        with pytest.raises(ValidationError):
            SearchResponse(
                query="test",
                results=[],
                total_found=-1,
                book_id="test",
                module_filter=None
            )

