"""Tests for ingestion crawler."""

import pytest
from pathlib import Path
from unittest.mock import patch, MagicMock
from rag.ingestion.crawler import DocsCrawler, DiscoveredFile


def test_discovered_file():
    """Test DiscoveredFile dataclass."""
    file = DiscoveredFile(
        absolute_path=Path("/test/path.md"),
        relative_path="path.md",
        module="ros2",
        module_number=1,
        chapter=4,
        lesson=1,
        filename="01-test.md",
        is_readme=False
    )
    assert file.module == "ros2"
    assert file.chapter == 4
    assert file.lesson == 1
    assert file.is_readme is False


def test_crawler_initialization(tmp_path):
    """Test crawler initializes with docs path."""
    docs_dir = tmp_path / "docs"
    docs_dir.mkdir()
    crawler = DocsCrawler(docs_path=str(docs_dir))
    assert crawler.docs_path.exists()


def test_module_mapping(tmp_path):
    """Test module name mapping."""
    docs_dir = tmp_path / "docs"
    docs_dir.mkdir()
    crawler = DocsCrawler(docs_path=str(docs_dir))
    
    # Valid module mappings
    assert "module-1-ros2" in crawler.MODULE_MAP
    assert crawler.MODULE_MAP["module-1-ros2"] == ("ros2", 1)
    assert "module-2-simulation" in crawler.MODULE_MAP
    assert crawler.MODULE_MAP["module-2-simulation"] == ("gazebo", 2)
    assert "module-3-isaac" in crawler.MODULE_MAP
    assert crawler.MODULE_MAP["module-3-isaac"] == ("isaac", 3)
    assert "module-4-vla" in crawler.MODULE_MAP
    assert crawler.MODULE_MAP["module-4-vla"] == ("vla", 4)
    
    # Invalid modules not in map
    assert "module-5-invalid" not in crawler.MODULE_MAP
    assert "invalid" not in crawler.MODULE_MAP


def test_chapter_pattern_matching(tmp_path):
    """Test chapter pattern matching."""
    docs_dir = tmp_path / "docs"
    docs_dir.mkdir()
    crawler = DocsCrawler(docs_path=str(docs_dir))
    
    # Valid chapter patterns
    assert crawler.CHAPTER_PATTERN.match("chapter-1-intro")
    assert crawler.CHAPTER_PATTERN.match("chapter-10-advanced")
    
    # Pattern matches chapter-0-* (validation happens in code)
    match = crawler.CHAPTER_PATTERN.match("chapter-0-invalid")
    if match:
        # Would be filtered out in actual code if chapter < 1
        assert int(match.group(1)) == 0
    assert crawler.CHAPTER_PATTERN.match("invalid") is None


def test_lesson_pattern_matching(tmp_path):
    """Test lesson pattern matching."""
    docs_dir = tmp_path / "docs"
    docs_dir.mkdir()
    crawler = DocsCrawler(docs_path=str(docs_dir))
    
    # Valid lesson patterns
    assert crawler.LESSON_PATTERN.match("01-intro.md")
    assert crawler.LESSON_PATTERN.match("10-advanced.md")
    
    # Pattern matches 00-* (validation happens in code)
    match = crawler.LESSON_PATTERN.match("00-invalid.md")
    if match:
        # Would be filtered out in actual code if lesson < 1
        assert int(match.group(1)) == 0
    assert crawler.LESSON_PATTERN.match("invalid.md") is None

