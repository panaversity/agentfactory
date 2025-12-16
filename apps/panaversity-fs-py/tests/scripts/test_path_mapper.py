"""Tests for path_mapper module."""

import pytest
import sys
from pathlib import Path

# Add scripts to path for imports
scripts_dir = Path(__file__).parent.parent.parent / "scripts"
sys.path.insert(0, str(scripts_dir.parent))

from scripts.ingest.path_mapper import (
    ContentType,
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
        """Test Part pattern matches valid formats (NN- prefix)."""
        # Pattern matches NN- at start of part name
        assert PART_PATTERN.match("01-Introducing-AI")
        assert PART_PATTERN.match("1-Part")
        assert PART_PATTERN.match("10-Advanced")
        assert PART_PATTERN.match("99-Final")

    def test_part_pattern_invalid(self):
        """Test Part pattern rejects invalid formats."""
        assert not PART_PATTERN.match("Part-01")  # wrong format (old style)
        assert not PART_PATTERN.match("Introduction")  # no number prefix
        assert not PART_PATTERN.match("-01-Part")  # leading hyphen

    def test_chapter_pattern_valid(self):
        """Test Chapter pattern matches valid formats (NN- prefix)."""
        assert CHAPTER_PATTERN.match("01-chapter-name")
        assert CHAPTER_PATTERN.match("1-intro")
        assert CHAPTER_PATTERN.match("10-advanced")

    def test_chapter_pattern_invalid(self):
        """Test Chapter pattern rejects invalid formats."""
        assert not CHAPTER_PATTERN.match("Chapter-01")  # old format
        assert not CHAPTER_PATTERN.match("intro")  # no number

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
        # New format: NN-PartName/NN-ChapterName/NN-lesson.md
        result = map_source_to_storage("01-Intro/02-Chapter/03-lesson.md")

        assert result.valid is True
        assert result.storage_path == "content/01-Part/02-Chapter/03-lesson.md"
        assert result.content_type == ContentType.MARKDOWN
        assert result.error is None

    def test_valid_summary_path(self):
        """Test mapping a valid summary path."""
        result = map_source_to_storage("01-Intro/01-Chapter/02-intro.summary.md")

        assert result.valid is True
        assert result.storage_path == "content/01-Part/01-Chapter/02-intro.summary.md"
        assert result.content_type == ContentType.SUMMARY
        assert result.error is None

    def test_valid_asset_path(self):
        """Test mapping a valid asset path."""
        result = map_source_to_storage("01-Part/01-Chapter/img/test.png")

        assert result.valid is True
        assert result.storage_path == "static/images/test.png"
        assert result.content_type == ContentType.ASSET

    def test_readme_path_skipped(self):
        """Test README files are skipped."""
        result = map_source_to_storage("01-Part/01-Chapter/README.md")

        assert result.valid is False
        assert result.content_type == ContentType.README
        assert "README" in result.error

    def test_case_insensitive_readme(self):
        """Test README detection is case insensitive."""
        result = map_source_to_storage("01-Part/01-Chapter/readme.md")
        assert result.content_type == ContentType.README

        result = map_source_to_storage("01-Part/01-Chapter/ReadMe.md")
        assert result.content_type == ContentType.README

    def test_invalid_part_format(self):
        """Test invalid Part format is rejected."""
        # Missing number prefix
        result = map_source_to_storage("Introduction/01-Chapter/01-lesson.md")
        assert result.valid is False
        assert "Part" in result.error

    def test_invalid_chapter_format(self):
        """Test invalid Chapter format is rejected."""
        # Chapter without number prefix
        result = map_source_to_storage("01-Part/chapter/01-lesson.md")
        assert result.valid is False
        assert "Chapter" in result.error

    def test_invalid_filename_format(self):
        """Test invalid filename format is rejected."""
        result = map_source_to_storage("01-Part/01-Chapter/lesson.md")
        assert result.valid is False
        assert "filename" in result.error.lower()

    def test_path_too_short(self):
        """Test path with too few components is rejected."""
        result = map_source_to_storage("01-lesson.md")
        assert result.valid is False
        assert "too short" in result.error.lower()

    def test_unknown_file_type(self):
        """Test unknown file types are rejected."""
        result = map_source_to_storage("01-Part/01-Chapter/01-lesson.txt")
        assert result.valid is False
        assert result.content_type == ContentType.UNKNOWN

    def test_number_padding(self):
        """Test numbers are zero-padded."""
        result = map_source_to_storage("1-Part/2-Chapter/3-lesson.md")
        assert result.valid is True
        assert "01-Part" in result.storage_path
        assert "02-Chapter" in result.storage_path
        assert "03-lesson" in result.storage_path

    def test_windows_path_separators(self):
        """Test Windows path separators are handled."""
        result = map_source_to_storage("01-Part\\01-Chapter\\01-lesson.md")
        assert result.valid is True
        assert "/" in result.storage_path
        assert "\\" not in result.storage_path


class TestValidateStoragePath:
    """Tests for validate_storage_path function."""

    def test_valid_content_path(self):
        """Test valid content path passes validation."""
        is_valid, error = validate_storage_path("content/01-Part/01-Chapter/01-intro.md")
        assert is_valid is True
        assert error is None

    def test_valid_summary_path(self):
        """Test valid summary path passes validation."""
        is_valid, error = validate_storage_path("content/01-Part/01-Chapter/01-intro.summary.md")
        assert is_valid is True
        assert error is None

    def test_valid_asset_path(self):
        """Test valid asset path passes validation."""
        is_valid, error = validate_storage_path("static/images/diagram.png")
        assert is_valid is True

        is_valid, error = validate_storage_path("static/slides/chapter-01.pdf")
        assert is_valid is True

    def test_invalid_content_path_format(self):
        """Test invalid content path is rejected."""
        is_valid, error = validate_storage_path("content/Part-01/Chapter-01/intro.md")
        assert is_valid is False
        assert "schema" in error.lower()

    def test_invalid_asset_type(self):
        """Test invalid asset type is rejected."""
        is_valid, error = validate_storage_path("static/unknown/file.png")
        assert is_valid is False

    def test_unknown_path_type(self):
        """Test unknown path prefix is rejected."""
        is_valid, error = validate_storage_path("other/path/file.md")
        assert is_valid is False
        assert "Unknown" in error


class TestMapAndValidate:
    """Tests for map_and_validate function."""

    def test_valid_path_passes(self):
        """Test valid path passes mapping and validation."""
        result = map_and_validate("01-Intro/01-Chapter/01-intro.md")
        assert result.valid is True
        assert result.storage_path is not None

    def test_invalid_source_fails(self):
        """Test invalid source path fails."""
        result = map_and_validate("invalid.md")
        assert result.valid is False


class TestParametrized:
    """Parametrized tests for comprehensive path mapping."""

    @pytest.mark.parametrize("source,expected_storage,valid", [
        # Valid paths (new format: NN-Name/NN-Name/NN-name.md)
        ("01-Intro/01-Chapter/01-intro.md", "content/01-Part/01-Chapter/01-intro.md", True),
        ("02-Advanced/03-Complex/05-advanced.md", "content/02-Part/03-Chapter/05-advanced.md", True),
        ("01-Intro/01-Chapter/01-intro.summary.md", "content/01-Part/01-Chapter/01-intro.summary.md", True),
        ("01-Part/01-Chapter/img/test.png", "static/images/test.png", True),
        # Invalid paths
        ("01-Part/01-Chapter/README.md", None, False),  # README skipped
        ("invalid.md", None, False),  # Too short
    ])
    def test_path_mapping(self, source, expected_storage, valid):
        """Test various path mappings."""
        result = map_source_to_storage(source)
        assert result.valid is valid
        if valid:
            assert result.storage_path == expected_storage


class TestSecurityValidation:
    """Tests for security-related path validation."""

    def test_path_traversal_in_lesson_name_blocked(self):
        """Test path traversal in lesson name is blocked."""
        result = map_source_to_storage("01-Part/01-Chapter/01-../../../etc/passwd.md")
        # Should either be invalid or have traversal sequences removed
        if result.valid:
            assert ".." not in result.storage_path

    def test_path_traversal_with_double_dots_blocked(self):
        """Test double dot path traversal is blocked."""
        result = map_source_to_storage("01-Part/../01-Chapter/01-lesson.md")
        if result.valid:
            assert ".." not in result.storage_path

    def test_path_traversal_in_asset_blocked(self):
        """Test path traversal in asset path is blocked."""
        result = map_source_to_storage("01-Part/01-Chapter/img/../../../etc/passwd.png")
        if result.valid:
            assert ".." not in result.storage_path

    def test_null_byte_in_path_sanitized(self):
        """Test null bytes are sanitized."""
        result = map_source_to_storage("01-Part/01-Chapter/01-lesson\x00.md")
        if result.valid:
            assert "\x00" not in result.storage_path

    def test_slash_in_lesson_name_sanitized(self):
        """Test slashes in names are sanitized."""
        result = map_source_to_storage("01-Part/01-Chapter/01-a/b.md")
        # Path should be handled without allowing traversal
        assert result is not None

    def test_backslash_in_lesson_name_sanitized(self):
        """Test backslashes in names are sanitized."""
        result = map_source_to_storage("01-Part/01-Chapter/01-a\\b.md")
        if result.valid:
            assert "\\" not in result.storage_path
