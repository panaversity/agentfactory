"""Tests for path_mapper module."""

import pytest
import sys
from pathlib import Path

# Add scripts to path for imports
scripts_dir = Path(__file__).parent.parent.parent / "scripts"
sys.path.insert(0, str(scripts_dir.parent))

from scripts.ingest.path_mapper import (
    ContentType,
    MappedPath,
    map_source_to_storage,
    map_and_validate,
    validate_storage_path,
    PART_PATTERN,
    CHAPTER_PATTERN,
    LESSON_PATTERN,
)


class TestContentType:
    """Tests for ContentType enum."""

    def test_enum_values(self):
        """Test all enum values exist."""
        assert ContentType.MARKDOWN.value == "markdown"
        assert ContentType.SUMMARY.value == "summary"
        assert ContentType.ASSET.value == "asset"
        assert ContentType.README.value == "readme"
        assert ContentType.UNKNOWN.value == "unknown"


class TestPatterns:
    """Tests for regex patterns."""

    def test_part_pattern_valid(self):
        """Test Part pattern matches valid formats."""
        assert PART_PATTERN.match("Part-01")
        assert PART_PATTERN.match("Part-1")
        assert PART_PATTERN.match("Part-10")
        assert PART_PATTERN.match("Part-99")

    def test_part_pattern_invalid(self):
        """Test Part pattern rejects invalid formats."""
        assert not PART_PATTERN.match("part-01")  # lowercase
        assert not PART_PATTERN.match("Part01")  # no hyphen
        assert not PART_PATTERN.match("Part-")  # no number
        assert not PART_PATTERN.match("Chapter-01")  # wrong prefix

    def test_chapter_pattern_valid(self):
        """Test Chapter pattern matches valid formats."""
        assert CHAPTER_PATTERN.match("Chapter-01")
        assert CHAPTER_PATTERN.match("Chapter-1")
        assert CHAPTER_PATTERN.match("Chapter-10")

    def test_chapter_pattern_invalid(self):
        """Test Chapter pattern rejects invalid formats."""
        assert not CHAPTER_PATTERN.match("chapter-01")
        assert not CHAPTER_PATTERN.match("Chapter01")
        assert not CHAPTER_PATTERN.match("Part-01")

    def test_lesson_pattern_valid(self):
        """Test Lesson pattern matches valid formats."""
        assert LESSON_PATTERN.match("01-introduction.md")
        assert LESSON_PATTERN.match("02-getting-started.md")
        assert LESSON_PATTERN.match("10-advanced-topics.md")
        assert LESSON_PATTERN.match("01-intro.summary.md")

    def test_lesson_pattern_captures_correctly(self):
        """Test Lesson pattern captures groups correctly."""
        match = LESSON_PATTERN.match("02-getting-started.md")
        assert match.group(1) == "02"  # lesson number
        assert match.group(2) == "getting-started"  # lesson name
        assert match.group(3) is None  # no summary suffix

        match = LESSON_PATTERN.match("01-intro.summary.md")
        assert match.group(1) == "01"
        assert match.group(2) == "intro"
        assert match.group(3) == ".summary"


class TestMapSourceToStorage:
    """Tests for map_source_to_storage function."""

    def test_valid_markdown_path(self):
        """Test mapping a valid markdown path."""
        result = map_source_to_storage("Part-01/Chapter-02/03-lesson.md")

        assert result.valid is True
        assert result.storage_path == "content/01-Part/02-Chapter/03-lesson.md"
        assert result.content_type == ContentType.MARKDOWN
        assert result.error is None

    def test_valid_summary_path(self):
        """Test mapping a valid summary path."""
        result = map_source_to_storage("Part-01/Chapter-01/02-intro.summary.md")

        assert result.valid is True
        assert result.storage_path == "content/01-Part/01-Chapter/02-intro.summary.md"
        assert result.content_type == ContentType.SUMMARY

    def test_valid_asset_path(self):
        """Test mapping a valid asset path (img normalizes to images)."""
        result = map_source_to_storage("Part-01/Chapter-01/img/diagram.png")

        assert result.valid is True
        assert result.storage_path == "static/images/diagram.png"
        assert result.content_type == ContentType.ASSET

    def test_readme_path_skipped(self):
        """Test README files are skipped."""
        result = map_source_to_storage("Part-01/Chapter-01/README.md")

        assert result.valid is False
        assert result.content_type == ContentType.README
        assert "README" in result.error

    def test_case_insensitive_readme(self):
        """Test README detection is case-insensitive."""
        result = map_source_to_storage("Part-01/Chapter-01/readme.md")
        assert result.valid is False
        assert result.content_type == ContentType.README

    def test_invalid_part_format(self):
        """Test invalid Part format is rejected."""
        result = map_source_to_storage("part-01/Chapter-01/01-lesson.md")

        assert result.valid is False
        assert "Part" in result.error

    def test_invalid_chapter_format(self):
        """Test invalid Chapter format is rejected."""
        result = map_source_to_storage("Part-01/chapter-01/01-lesson.md")

        assert result.valid is False
        assert "Chapter" in result.error

    def test_invalid_filename_format(self):
        """Test invalid filename format is rejected."""
        result = map_source_to_storage("Part-01/Chapter-01/lesson.md")

        assert result.valid is False
        assert "filename" in result.error.lower()

    def test_path_too_short(self):
        """Test paths without Part/Chapter structure are rejected."""
        result = map_source_to_storage("01-lesson.md")

        assert result.valid is False
        assert "too short" in result.error.lower()

    def test_unknown_file_type(self):
        """Test unknown file types are rejected."""
        result = map_source_to_storage("Part-01/Chapter-01/data.json")

        assert result.valid is False
        assert result.content_type == ContentType.UNKNOWN

    def test_number_padding(self):
        """Test numbers are properly zero-padded."""
        result = map_source_to_storage("Part-1/Chapter-2/3-lesson.md")

        assert result.valid is True
        assert result.storage_path == "content/01-Part/02-Chapter/03-lesson.md"

    def test_windows_path_separators(self):
        """Test Windows-style path separators are normalized."""
        result = map_source_to_storage("Part-01\\Chapter-01\\01-lesson.md")

        assert result.valid is True
        assert result.storage_path == "content/01-Part/01-Chapter/01-lesson.md"


class TestValidateStoragePath:
    """Tests for validate_storage_path function."""

    def test_valid_content_path(self):
        """Test valid content paths pass validation."""
        valid, error = validate_storage_path("content/01-Part/02-Chapter/03-lesson.md")
        assert valid is True
        assert error is None

    def test_valid_summary_path(self):
        """Test valid summary paths pass validation."""
        valid, error = validate_storage_path("content/01-Part/01-Chapter/01-intro.summary.md")
        assert valid is True

    def test_valid_asset_path(self):
        """Test valid asset paths pass validation."""
        valid, error = validate_storage_path("static/img/diagram.png")
        assert valid is True

        valid, error = validate_storage_path("static/slides/presentation.pdf")
        assert valid is True

    def test_invalid_content_path_format(self):
        """Test invalid content paths fail validation."""
        # Missing Part/Chapter structure
        valid, error = validate_storage_path("content/01-lesson.md")
        assert valid is False
        assert error is not None

    def test_invalid_asset_type(self):
        """Test invalid asset types fail validation."""
        valid, error = validate_storage_path("static/unknown/file.txt")
        assert valid is False

    def test_unknown_path_type(self):
        """Test unknown path types fail validation."""
        valid, error = validate_storage_path("random/path/file.md")
        assert valid is False


class TestMapAndValidate:
    """Tests for map_and_validate combined function."""

    def test_valid_path_passes(self):
        """Test valid paths pass both mapping and validation."""
        result = map_and_validate("Part-01/Chapter-01/01-intro.md")

        assert result.valid is True
        assert result.storage_path == "content/01-Part/01-Chapter/01-intro.md"

    def test_invalid_source_fails(self):
        """Test invalid source paths fail."""
        result = map_and_validate("invalid.md")

        assert result.valid is False
        assert result.error is not None


class TestParametrized:
    """Parametrized tests using fixtures."""

    @pytest.mark.parametrize("source,expected,valid", [
        ("Part-01/Chapter-01/01-intro.md", "content/01-Part/01-Chapter/01-intro.md", True),
        ("Part-02/Chapter-03/05-advanced.md", "content/02-Part/03-Chapter/05-advanced.md", True),
        ("Part-01/Chapter-01/01-intro.summary.md", "content/01-Part/01-Chapter/01-intro.summary.md", True),
        ("Part-01/Chapter-01/img/test.png", "static/images/test.png", True),  # img normalizes to images
        ("Part-01/Chapter-01/README.md", None, False),
        ("invalid.md", None, False),
    ])
    def test_path_mapping(self, source: str, expected: str, valid: bool):
        """Test path mapping with various inputs."""
        result = map_source_to_storage(source)
        assert result.valid is valid
        if valid:
            assert result.storage_path == expected


class TestSecurityValidation:
    """Security tests for path traversal prevention."""

    def test_path_traversal_in_lesson_name_blocked(self):
        """Test that path traversal in lesson name is blocked."""
        result = map_source_to_storage("Part-01/Chapter-01/01-../../etc/passwd.md")

        # Should be rejected due to path traversal
        assert result.valid is False
        assert result.error is not None
        assert "traversal" in result.error.lower() or "invalid" in result.error.lower()

    def test_path_traversal_with_double_dots_blocked(self):
        """Test that double dots are sanitized from paths."""
        result = map_source_to_storage("Part-01/Chapter-01/01-test..test.md")

        # Should either be sanitized or rejected
        if result.valid:
            # If accepted, double dots should be removed from storage path
            assert ".." not in result.storage_path
        else:
            # Or it should be rejected entirely
            assert result.error is not None

    def test_path_traversal_in_asset_blocked(self):
        """Test that path traversal in asset paths is blocked."""
        result = map_source_to_storage("Part-01/Chapter-01/img/../../secrets.png")

        # Should be rejected or sanitized
        if result.valid:
            # If valid, should not allow escaping static directory
            assert result.storage_path.startswith("static/")
            assert ".." not in result.storage_path
        else:
            assert result.error is not None

    def test_null_byte_in_path_sanitized(self):
        """Test that null bytes are removed from paths."""
        # Null byte injection attempt
        result = map_source_to_storage("Part-01/Chapter-01/01-test\x00evil.md")

        # Should either be sanitized or rejected
        if result.valid:
            assert "\x00" not in result.storage_path
        else:
            assert result.error is not None

    def test_slash_in_lesson_name_sanitized(self):
        """Test that slashes in lesson names are sanitized."""
        result = map_source_to_storage("Part-01/Chapter-01/01-test/subdir.md")

        # Should be rejected (invalid format) or sanitized
        assert result.valid is False or "/" not in result.storage_path.split("/")[-1]

    def test_backslash_in_lesson_name_sanitized(self):
        """Test that backslashes in lesson names are sanitized."""
        result = map_source_to_storage("Part-01/Chapter-01/01-test\\subdir.md")

        # Should be sanitized or rejected
        if result.valid:
            assert "\\" not in result.storage_path
        else:
            assert result.error is not None
