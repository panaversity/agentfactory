"""Tests for enhanced tool features (Option B enhancements).

Tests for:
- read_content with scope parameter (file/chapter/part)
- get_asset with include_binary parameter
- list_assets with static/ path (ADR-0018 alignment)
"""

import pytest
import json
import base64
import os
from pathlib import Path

from panaversity_fs.tools.content import read_content
from panaversity_fs.tools.assets import get_asset, list_assets, upload_asset
from panaversity_fs.models import (
    ReadContentInput, GetAssetInput, ListAssetsInput, UploadAssetInput,
    ContentScope, AssetType
)
from panaversity_fs.storage import get_operator
from panaversity_fs.config import get_config


@pytest.fixture
def temp_storage(tmp_path, monkeypatch):
    """Create temporary storage with test structure."""
    monkeypatch.setenv("PANAVERSITY_STORAGE_ROOT", str(tmp_path))
    monkeypatch.setenv("PANAVERSITY_STORAGE_BACKEND", "fs")

    # Clear cached config and operator
    from panaversity_fs import config, storage
    config._config = None
    storage._operator = None

    return tmp_path


@pytest.fixture
def book_with_chapters(temp_storage):
    """Create a book with multi-chapter structure."""
    book_path = temp_storage / "books" / "test-book"

    # Create content structure
    content_path = book_path / "content" / "01-Introduction"

    # Part README
    (content_path).mkdir(parents=True, exist_ok=True)
    (content_path / "README.md").write_text("# Part 1: Introduction\n\nPart intro content.")

    # Chapter 1
    chapter1 = content_path / "01-Getting-Started"
    chapter1.mkdir(parents=True, exist_ok=True)
    (chapter1 / "README.md").write_text("# Chapter 1: Getting Started\n\nChapter overview.")
    (chapter1 / "01-lesson.md").write_text("# Lesson 1\n\nFirst lesson content.")
    (chapter1 / "02-lesson.md").write_text("# Lesson 2\n\nSecond lesson content.")
    (chapter1 / "01-lesson.summary.md").write_text("# Summary\n\nLesson 1 summary.")

    # Chapter 2
    chapter2 = content_path / "02-Core-Concepts"
    chapter2.mkdir(parents=True, exist_ok=True)
    (chapter2 / "README.md").write_text("# Chapter 2: Core Concepts\n\nSecond chapter.")
    (chapter2 / "01-basics.md").write_text("# Basics\n\nCore basics.")

    return book_path


@pytest.fixture
def book_with_assets(temp_storage):
    """Create a book with static assets."""
    book_path = temp_storage / "books" / "asset-book"

    # Create static structure (ADR-0018 aligned)
    static_path = book_path / "static"

    # Images
    images_path = static_path / "images"
    images_path.mkdir(parents=True, exist_ok=True)
    # Create a simple PNG (1x1 red pixel)
    png_data = base64.b64decode(
        "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mP8z8BQDwAEhQGAhKmMIQAAAABJRU5ErkJggg=="
    )
    (images_path / "test-image.png").write_bytes(png_data)
    (images_path / "diagram.png").write_bytes(png_data)

    # Slides
    slides_path = static_path / "slides"
    slides_path.mkdir(parents=True, exist_ok=True)
    (slides_path / "intro.pdf").write_bytes(b"%PDF-1.4 fake pdf content")

    return book_path


# =============================================================================
# Tests for read_content scope parameter
# =============================================================================

class TestReadContentScope:
    """Tests for read_content scope parameter."""

    @pytest.mark.asyncio
    async def test_scope_file_default(self, book_with_chapters):
        """Test scope=file (default) reads single file."""
        params = ReadContentInput(
            book_id="test-book",
            path="content/01-Introduction/01-Getting-Started/01-lesson.md"
        )

        result = await read_content(params)
        data = json.loads(result)

        assert "content" in data
        assert "# Lesson 1" in data["content"]
        assert "file_size" in data
        assert "file_hash_sha256" in data
        # Single file returns object, not array
        assert "path" not in data  # Single file doesn't include path

    @pytest.mark.asyncio
    async def test_scope_file_explicit(self, book_with_chapters):
        """Test scope=file explicitly set."""
        params = ReadContentInput(
            book_id="test-book",
            path="content/01-Introduction/01-Getting-Started/01-lesson.md",
            scope=ContentScope.FILE
        )

        result = await read_content(params)
        data = json.loads(result)

        assert "content" in data
        assert "# Lesson 1" in data["content"]

    @pytest.mark.asyncio
    async def test_scope_chapter_reads_all_md_in_chapter(self, book_with_chapters):
        """Test scope=chapter reads all .md files in chapter directory."""
        params = ReadContentInput(
            book_id="test-book",
            path="content/01-Introduction/01-Getting-Started",
            scope=ContentScope.CHAPTER
        )

        result = await read_content(params)
        data = json.loads(result)

        # Should return array
        assert isinstance(data, list)
        # Should have 4 files: README.md, 01-lesson.md, 02-lesson.md, 01-lesson.summary.md
        assert len(data) == 4

        # Check paths are included
        paths = [item["path"] for item in data]
        assert "content/01-Introduction/01-Getting-Started/README.md" in paths
        assert "content/01-Introduction/01-Getting-Started/01-lesson.md" in paths
        assert "content/01-Introduction/01-Getting-Started/02-lesson.md" in paths

        # Each item should have content and metadata
        for item in data:
            assert "path" in item
            assert "content" in item
            assert "file_size" in item
            assert "file_hash_sha256" in item

    @pytest.mark.asyncio
    async def test_scope_chapter_excludes_subdirectories(self, book_with_chapters):
        """Test scope=chapter does NOT include files from subdirectories."""
        # Chapter scope should only include files directly in the chapter,
        # not files in nested subdirectories
        params = ReadContentInput(
            book_id="test-book",
            path="content/01-Introduction/01-Getting-Started",
            scope=ContentScope.CHAPTER
        )

        result = await read_content(params)
        data = json.loads(result)

        # All paths should be directly in the chapter directory
        for item in data:
            rel_path = item["path"].replace("content/01-Introduction/01-Getting-Started/", "")
            assert "/" not in rel_path, f"Found nested file: {item['path']}"

    @pytest.mark.asyncio
    async def test_scope_part_reads_all_md_in_part(self, book_with_chapters):
        """Test scope=part reads all .md files in part directory (recursive)."""
        params = ReadContentInput(
            book_id="test-book",
            path="content/01-Introduction",
            scope=ContentScope.PART
        )

        result = await read_content(params)
        data = json.loads(result)

        # Should return array
        assert isinstance(data, list)
        # Should have: Part README + Chapter 1 (4 files) + Chapter 2 (2 files) = 7 total
        assert len(data) == 7

        # Check paths include both chapters
        paths = [item["path"] for item in data]
        assert "content/01-Introduction/README.md" in paths
        assert "content/01-Introduction/01-Getting-Started/01-lesson.md" in paths
        assert "content/01-Introduction/02-Core-Concepts/01-basics.md" in paths

    @pytest.mark.asyncio
    async def test_scope_chapter_sorted_by_path(self, book_with_chapters):
        """Test scope=chapter results are sorted by path."""
        params = ReadContentInput(
            book_id="test-book",
            path="content/01-Introduction/01-Getting-Started",
            scope=ContentScope.CHAPTER
        )

        result = await read_content(params)
        data = json.loads(result)

        paths = [item["path"] for item in data]
        assert paths == sorted(paths)

    @pytest.mark.asyncio
    async def test_scope_chapter_empty_directory(self, temp_storage):
        """Test scope=chapter on empty directory returns empty array."""
        # Create empty chapter
        empty_chapter = temp_storage / "books" / "empty-book" / "content" / "01-Part" / "01-Chapter"
        empty_chapter.mkdir(parents=True, exist_ok=True)
        (empty_chapter / ".gitkeep").write_text("")

        params = ReadContentInput(
            book_id="empty-book",
            path="content/01-Part/01-Chapter",
            scope=ContentScope.CHAPTER
        )

        result = await read_content(params)
        data = json.loads(result)

        assert isinstance(data, list)
        assert len(data) == 0


# =============================================================================
# Tests for get_asset include_binary parameter
# =============================================================================

class TestGetAssetIncludeBinary:
    """Tests for get_asset include_binary parameter."""

    @pytest.mark.asyncio
    async def test_include_binary_false_default(self, book_with_assets):
        """Test include_binary=false (default) returns metadata only."""
        params = GetAssetInput(
            book_id="asset-book",
            asset_type=AssetType.IMAGES,
            filename="test-image.png"
        )

        result = await get_asset(params)
        data = json.loads(result)

        assert "cdn_url" in data
        assert "file_size" in data
        assert "mime_type" in data
        assert data["mime_type"] == "image/png"
        # binary_data should be null by default
        assert data.get("binary_data") is None

    @pytest.mark.asyncio
    async def test_include_binary_true_returns_base64(self, book_with_assets):
        """Test include_binary=true returns base64-encoded binary data."""
        params = GetAssetInput(
            book_id="asset-book",
            asset_type=AssetType.IMAGES,
            filename="test-image.png",
            include_binary=True
        )

        result = await get_asset(params)
        data = json.loads(result)

        assert "cdn_url" in data
        assert "binary_data" in data
        assert data["binary_data"] is not None

        # Verify it's valid base64 and decodes to PNG
        binary_content = base64.b64decode(data["binary_data"])
        assert binary_content.startswith(b'\x89PNG')  # PNG magic bytes

    @pytest.mark.asyncio
    async def test_include_binary_pdf_asset(self, book_with_assets):
        """Test include_binary works for non-image assets (PDF)."""
        params = GetAssetInput(
            book_id="asset-book",
            asset_type=AssetType.SLIDES,
            filename="intro.pdf",
            include_binary=True
        )

        result = await get_asset(params)
        data = json.loads(result)

        assert data["mime_type"] == "application/pdf"
        assert data["binary_data"] is not None

        # Verify it's valid base64 and decodes to PDF-like content
        binary_content = base64.b64decode(data["binary_data"])
        assert b"PDF" in binary_content


# =============================================================================
# Tests for list_assets static/ path alignment
# =============================================================================

class TestListAssetsStaticPath:
    """Tests for list_assets using static/ path (ADR-0018)."""

    @pytest.mark.asyncio
    async def test_list_assets_finds_images(self, book_with_assets):
        """Test list_assets finds images in static/images/."""
        params = ListAssetsInput(
            book_id="asset-book",
            asset_type=AssetType.IMAGES
        )

        result = await list_assets(params)
        data = json.loads(result)

        assert isinstance(data, list)
        assert len(data) == 2  # test-image.png and diagram.png

        filenames = [item["filename"] for item in data]
        assert "test-image.png" in filenames
        assert "diagram.png" in filenames

    @pytest.mark.asyncio
    async def test_list_assets_finds_slides(self, book_with_assets):
        """Test list_assets finds slides in static/slides/."""
        params = ListAssetsInput(
            book_id="asset-book",
            asset_type=AssetType.SLIDES
        )

        result = await list_assets(params)
        data = json.loads(result)

        assert isinstance(data, list)
        assert len(data) == 1

        assert data[0]["filename"] == "intro.pdf"
        assert data[0]["mime_type"] == "application/pdf"

    @pytest.mark.asyncio
    async def test_list_assets_all_types(self, book_with_assets):
        """Test list_assets without filter returns all asset types."""
        params = ListAssetsInput(
            book_id="asset-book"
        )

        result = await list_assets(params)
        data = json.loads(result)

        assert isinstance(data, list)
        # 2 images + 1 slide = 3 total
        assert len(data) == 3

    @pytest.mark.asyncio
    async def test_list_assets_empty_type(self, book_with_assets):
        """Test list_assets for asset type with no files."""
        params = ListAssetsInput(
            book_id="asset-book",
            asset_type=AssetType.VIDEOS  # No videos uploaded
        )

        result = await list_assets(params)
        data = json.loads(result)

        assert isinstance(data, list)
        assert len(data) == 0


# =============================================================================
# Tests for upload_asset static/ path alignment
# =============================================================================

class TestUploadAssetStaticPath:
    """Tests for upload_asset using static/ path (ADR-0018)."""

    @pytest.mark.asyncio
    async def test_upload_asset_to_static_path(self, temp_storage):
        """Test upload_asset writes to static/ (not assets/)."""
        # Create book directory
        (temp_storage / "books" / "upload-test").mkdir(parents=True, exist_ok=True)

        # Small PNG for upload
        png_data = base64.b64decode(
            "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mP8z8BQDwAEhQGAhKmMIQAAAABJRU5ErkJggg=="
        )

        params = UploadAssetInput(
            book_id="upload-test",
            asset_type=AssetType.IMAGES,
            filename="new-image.png",
            binary_data=base64.b64encode(png_data).decode('ascii')
        )

        result = await upload_asset(params)
        data = json.loads(result)

        assert data["status"] == "success"
        # Path should use static/, not assets/
        assert "static/images/new-image.png" in data["path"]
        assert "assets/" not in data["path"]

        # Verify file exists at correct location
        expected_path = temp_storage / "books" / "upload-test" / "static" / "images" / "new-image.png"
        assert expected_path.exists()
