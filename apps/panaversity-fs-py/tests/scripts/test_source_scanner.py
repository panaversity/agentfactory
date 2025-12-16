"""Tests for source_scanner module."""

import pytest
import sys
from pathlib import Path

# Add scripts to path for imports
scripts_dir = Path(__file__).parent.parent.parent / "scripts"
sys.path.insert(0, str(scripts_dir.parent))

from scripts.ingest.source_scanner import (
    SourceFile,
    ScanResult,
    compute_hash,
    is_content_file,
    should_skip_directory,
    scan_source_directory,
    filter_by_content_type,
    get_valid_files,
)
from scripts.ingest.path_mapper import ContentType


class TestComputeHash:
    """Tests for compute_hash function."""

    def test_hash_consistency(self, temp_dir: Path):
        """Test hash is consistent for same content."""
        file_path = temp_dir / "test.txt"
        file_path.write_text("Hello, World!")

        hash1 = compute_hash(file_path, ContentType.MARKDOWN)
        hash2 = compute_hash(file_path, ContentType.MARKDOWN)

        assert hash1 == hash2

    def test_hash_different_content(self, temp_dir: Path):
        """Test different content produces different hashes."""
        file1 = temp_dir / "file1.txt"
        file2 = temp_dir / "file2.txt"
        file1.write_text("Content A")
        file2.write_text("Content B")

        hash1 = compute_hash(file1, ContentType.MARKDOWN)
        hash2 = compute_hash(file2, ContentType.MARKDOWN)

        assert hash1 != hash2

    def test_hash_is_sha256(self, temp_dir: Path):
        """Test hash is a valid SHA256 hex string."""
        file_path = temp_dir / "test.txt"
        file_path.write_text("Test content")

        hash_value = compute_hash(file_path, ContentType.MARKDOWN)

        assert len(hash_value) == 64  # SHA256 produces 64 hex chars
        assert all(c in "0123456789abcdef" for c in hash_value)


class TestIsContentFile:
    """Tests for is_content_file function."""

    def test_markdown_files(self, temp_dir: Path):
        """Test markdown files are recognized."""
        md_file = temp_dir / "test.md"
        md_file.touch()
        assert is_content_file(md_file) is True

    def test_image_files(self, temp_dir: Path):
        """Test image files are recognized."""
        for ext in [".png", ".jpg", ".jpeg", ".gif", ".svg", ".webp"]:
            img_file = temp_dir / f"test{ext}"
            img_file.touch()
            assert is_content_file(img_file) is True, f"Failed for {ext}"

    def test_media_files(self, temp_dir: Path):
        """Test media files are recognized."""
        for ext in [".mp4", ".mp3", ".pdf"]:
            media_file = temp_dir / f"test{ext}"
            media_file.touch()
            assert is_content_file(media_file) is True, f"Failed for {ext}"

    def test_non_content_files(self, temp_dir: Path):
        """Test non-content files are rejected."""
        for ext in [".py", ".json", ".txt", ".html", ".css", ".js"]:
            other_file = temp_dir / f"test{ext}"
            other_file.touch()
            assert is_content_file(other_file) is False, f"Should reject {ext}"


class TestShouldSkipDirectory:
    """Tests for should_skip_directory function."""

    def test_hidden_directories(self):
        """Test hidden directories are skipped."""
        assert should_skip_directory(".git") is True
        assert should_skip_directory(".hidden") is True

    def test_common_non_content_dirs(self):
        """Test common non-content directories are skipped."""
        assert should_skip_directory("__pycache__") is True
        assert should_skip_directory("node_modules") is True
        assert should_skip_directory(".docusaurus") is True
        assert should_skip_directory("build") is True
        assert should_skip_directory("dist") is True

    def test_content_directories_not_skipped(self):
        """Test content directories are not skipped."""
        assert should_skip_directory("Part-01") is False
        assert should_skip_directory("Chapter-01") is False
        assert should_skip_directory("img") is False
        assert should_skip_directory("slides") is False


class TestScanSourceDirectory:
    """Tests for scan_source_directory function."""

    def test_scan_sample_structure(self, sample_book_structure: Path):
        """Test scanning a sample book structure."""
        result = scan_source_directory(sample_book_structure)

        assert isinstance(result, ScanResult)
        assert result.source_root == sample_book_structure.resolve()
        assert len(result.files) > 0

    def test_scan_counts(self, sample_book_structure: Path):
        """Test scan counts are accurate."""
        result = scan_source_directory(sample_book_structure)

        # We expect:
        # - 01-introduction.md (valid)
        # - 02-getting-started.md (valid)
        # - 02-getting-started.summary.md (valid)
        # - diagram.png (valid)
        # - 01-basics.md (valid)
        # - README.md (invalid - skipped)
        # - 01-advanced.md (valid)
        assert result.valid_count >= 5
        assert result.invalid_count >= 1  # README

    def test_scan_nonexistent_directory(self, temp_dir: Path):
        """Test scanning nonexistent directory raises ValueError."""
        with pytest.raises(ValueError) as exc_info:
            scan_source_directory(temp_dir / "nonexistent")
        assert "does not exist" in str(exc_info.value)

    def test_scan_file_instead_of_directory(self, temp_dir: Path):
        """Test scanning a file instead of directory raises ValueError."""
        file_path = temp_dir / "test.txt"
        file_path.write_text("test")

        with pytest.raises(ValueError) as exc_info:
            scan_source_directory(file_path)
        assert "not a directory" in str(exc_info.value)

    def test_scan_empty_directory(self, temp_dir: Path):
        """Test scanning an empty directory returns empty result."""
        result = scan_source_directory(temp_dir)

        assert result.valid_count == 0
        assert result.invalid_count == 0
        assert len(result.files) == 0

    def test_source_file_properties(self, sample_book_structure: Path):
        """Test SourceFile objects have correct properties."""
        result = scan_source_directory(sample_book_structure)

        for source_file in result.files:
            assert isinstance(source_file, SourceFile)
            assert source_file.relative_path
            assert source_file.absolute_path.exists()
            assert len(source_file.content_hash) == 64
            assert source_file.size_bytes >= 0
            assert source_file.mapped is not None


class TestFilterByContentType:
    """Tests for filter_by_content_type function."""

    def test_filter_markdown(self, sample_book_structure: Path):
        """Test filtering by markdown type."""
        result = scan_source_directory(sample_book_structure)
        markdown_files = filter_by_content_type(result.files, ContentType.MARKDOWN)

        assert len(markdown_files) > 0
        for f in markdown_files:
            assert f.mapped.content_type == ContentType.MARKDOWN

    def test_filter_summary(self, sample_book_structure: Path):
        """Test filtering by summary type."""
        result = scan_source_directory(sample_book_structure)
        summary_files = filter_by_content_type(result.files, ContentType.SUMMARY)

        assert len(summary_files) >= 1
        for f in summary_files:
            assert f.mapped.content_type == ContentType.SUMMARY

    def test_filter_asset(self, sample_book_structure: Path):
        """Test filtering by asset type."""
        result = scan_source_directory(sample_book_structure)
        asset_files = filter_by_content_type(result.files, ContentType.ASSET)

        assert len(asset_files) >= 1
        for f in asset_files:
            assert f.mapped.content_type == ContentType.ASSET


class TestGetValidFiles:
    """Tests for get_valid_files function."""

    def test_get_valid_only(self, sample_book_structure: Path):
        """Test getting only valid files."""
        result = scan_source_directory(sample_book_structure)
        valid_files = get_valid_files(result.files)

        assert len(valid_files) == result.valid_count
        for f in valid_files:
            assert f.mapped.valid is True

    def test_readme_excluded(self, sample_book_structure: Path):
        """Test README files are excluded from valid files."""
        result = scan_source_directory(sample_book_structure)
        valid_files = get_valid_files(result.files)

        for f in valid_files:
            assert "readme" not in f.relative_path.lower()
