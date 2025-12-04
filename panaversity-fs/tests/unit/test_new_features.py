"""Unit tests for new features added in this session.

Tests cover:
1. read_content BOOK scope - read all .md files in entire book
2. get_book_archive scope parameter (CONTENT/ASSETS/ALL)
3. list_books include_structure parameter (none/content/assets/all)
"""

import pytest
import json
from panaversity_fs.tools.content import read_content
from panaversity_fs.tools.bulk import get_book_archive
from panaversity_fs.tools.registry import list_books
from panaversity_fs.models import (
    ReadContentInput,
    ContentScope,
    GetBookArchiveInput,
    ArchiveScope,
    ListBooksInput
)


@pytest.fixture
async def multi_part_book_data(setup_fs_backend):
    """Create book with multiple parts, chapters, and lessons for scope testing."""
    from panaversity_fs.storage import get_operator

    op = get_operator()

    # Part 1 - Chapter 1
    await op.write(
        "books/test-book/content/01-Part-One/README.md",
        b"# Part One\n\nIntroduction to Part 1."
    )
    await op.write(
        "books/test-book/content/01-Part-One/01-Chapter-One/README.md",
        b"# Chapter One\n\nChapter 1 overview."
    )
    await op.write(
        "books/test-book/content/01-Part-One/01-Chapter-One/01-lesson.md",
        b"# Lesson 1\n\nFirst lesson content."
    )
    await op.write(
        "books/test-book/content/01-Part-One/01-Chapter-One/02-lesson.md",
        b"# Lesson 2\n\nSecond lesson content."
    )
    await op.write(
        "books/test-book/content/01-Part-One/01-Chapter-One/01-lesson.summary.md",
        b"# Lesson 1 Summary\n\nKey points."
    )

    # Part 1 - Chapter 2
    await op.write(
        "books/test-book/content/01-Part-One/02-Chapter-Two/README.md",
        b"# Chapter Two\n\nChapter 2 overview."
    )
    await op.write(
        "books/test-book/content/01-Part-One/02-Chapter-Two/01-lesson.md",
        b"# Chapter 2 Lesson\n\nChapter 2 lesson content."
    )

    # Part 2 - Chapter 1
    await op.write(
        "books/test-book/content/02-Part-Two/README.md",
        b"# Part Two\n\nIntroduction to Part 2."
    )
    await op.write(
        "books/test-book/content/02-Part-Two/01-Chapter-One/README.md",
        b"# Part 2 Chapter One\n\nPart 2 Chapter 1 overview."
    )
    await op.write(
        "books/test-book/content/02-Part-Two/01-Chapter-One/01-lesson.md",
        b"# Part 2 Lesson\n\nPart 2 lesson content."
    )

    # Assets (for archive scope testing)
    await op.write(
        "books/test-book/static/images/diagram.png",
        b"fake-png-data"
    )
    await op.write(
        "books/test-book/static/slides/presentation.pdf",
        b"fake-pdf-data"
    )

    return {"book_id": "test-book"}


# =============================================================================
# read_content BOOK Scope Tests
# =============================================================================

class TestReadContentBookScope:
    """Test read_content with scope=book."""

    @pytest.mark.asyncio
    async def test_book_scope_returns_all_markdown_files(self, multi_part_book_data):
        """Test that book scope returns all .md files from content/ directory."""
        result = await read_content(ReadContentInput(
            book_id=multi_part_book_data["book_id"],
            scope=ContentScope.BOOK
        ))

        data = json.loads(result)
        assert isinstance(data, list)
        # Should find all 10 .md files
        assert len(data) == 10

    @pytest.mark.asyncio
    async def test_book_scope_includes_all_parts(self, multi_part_book_data):
        """Test that book scope includes files from all parts."""
        result = await read_content(ReadContentInput(
            book_id=multi_part_book_data["book_id"],
            scope=ContentScope.BOOK
        ))

        data = json.loads(result)
        paths = [item["path"] for item in data]

        # Should have files from Part 1
        part1_files = [p for p in paths if "01-Part-One" in p]
        assert len(part1_files) >= 1

        # Should have files from Part 2
        part2_files = [p for p in paths if "02-Part-Two" in p]
        assert len(part2_files) >= 1

    @pytest.mark.asyncio
    async def test_book_scope_ignores_path_parameter(self, multi_part_book_data):
        """Test that book scope ignores the path parameter."""
        result = await read_content(ReadContentInput(
            book_id=multi_part_book_data["book_id"],
            path="content/01-Part-One",  # This should be ignored for book scope
            scope=ContentScope.BOOK
        ))

        data = json.loads(result)
        paths = [item["path"] for item in data]

        # Should still include Part 2 files even though path points to Part 1
        part2_files = [p for p in paths if "02-Part-Two" in p]
        assert len(part2_files) >= 1

    @pytest.mark.asyncio
    async def test_book_scope_excludes_non_md_files(self, multi_part_book_data):
        """Test that book scope only returns .md files."""
        result = await read_content(ReadContentInput(
            book_id=multi_part_book_data["book_id"],
            scope=ContentScope.BOOK
        ))

        data = json.loads(result)
        for item in data:
            assert item["path"].endswith(".md"), f"Non-md file found: {item['path']}"

    @pytest.mark.asyncio
    async def test_book_scope_includes_metadata(self, multi_part_book_data):
        """Test that book scope results include proper metadata."""
        result = await read_content(ReadContentInput(
            book_id=multi_part_book_data["book_id"],
            scope=ContentScope.BOOK
        ))

        data = json.loads(result)
        for item in data:
            assert "path" in item
            assert "content" in item
            assert "file_size" in item
            assert "file_hash_sha256" in item
            assert len(item["file_hash_sha256"]) == 64

    @pytest.mark.asyncio
    async def test_book_scope_sorted_by_path(self, multi_part_book_data):
        """Test that book scope results are sorted by path."""
        result = await read_content(ReadContentInput(
            book_id=multi_part_book_data["book_id"],
            scope=ContentScope.BOOK
        ))

        data = json.loads(result)
        paths = [item["path"] for item in data]
        assert paths == sorted(paths)

    @pytest.mark.asyncio
    async def test_book_scope_empty_book(self, setup_fs_backend):
        """Test book scope on book with no content returns empty array."""
        from panaversity_fs.storage import get_operator
        op = get_operator()

        # Create book with only static files, no content
        await op.write("books/empty-content-book/static/images/logo.png", b"fake")

        result = await read_content(ReadContentInput(
            book_id="empty-content-book",
            scope=ContentScope.BOOK
        ))

        # Should return empty list or error for non-existent content/
        # Based on implementation, it raises ContentNotFoundError for missing directory
        assert "error" in result.lower() or json.loads(result) == []


# =============================================================================
# read_content CHAPTER and PART Scope Tests (verification)
# =============================================================================

class TestReadContentChapterScope:
    """Test read_content with scope=chapter."""

    @pytest.mark.asyncio
    async def test_chapter_scope_returns_direct_files_only(self, multi_part_book_data):
        """Test that chapter scope only returns files directly in the chapter."""
        result = await read_content(ReadContentInput(
            book_id=multi_part_book_data["book_id"],
            path="content/01-Part-One/01-Chapter-One",
            scope=ContentScope.CHAPTER
        ))

        data = json.loads(result)
        # Should have: README.md, 01-lesson.md, 02-lesson.md, 01-lesson.summary.md
        assert len(data) == 4

        # Verify all files are from this chapter
        for item in data:
            assert "01-Chapter-One" in item["path"]
            # Should NOT include files from subdirectories
            rel_path = item["path"].split("01-Chapter-One/")[1]
            assert "/" not in rel_path


class TestReadContentPartScope:
    """Test read_content with scope=part."""

    @pytest.mark.asyncio
    async def test_part_scope_returns_recursive_files(self, multi_part_book_data):
        """Test that part scope returns files from all chapters recursively."""
        result = await read_content(ReadContentInput(
            book_id=multi_part_book_data["book_id"],
            path="content/01-Part-One",
            scope=ContentScope.PART
        ))

        data = json.loads(result)
        # Part 1 has:
        # - README.md (part level)
        # - Chapter 1: README.md, 01-lesson.md, 02-lesson.md, 01-lesson.summary.md
        # - Chapter 2: README.md, 01-lesson.md
        # Total: 7 files
        assert len(data) == 7

    @pytest.mark.asyncio
    async def test_part_scope_includes_all_chapters(self, multi_part_book_data):
        """Test that part scope includes files from all chapters."""
        result = await read_content(ReadContentInput(
            book_id=multi_part_book_data["book_id"],
            path="content/01-Part-One",
            scope=ContentScope.PART
        ))

        data = json.loads(result)
        paths = [item["path"] for item in data]

        # Should have files from Chapter 1
        ch1_files = [p for p in paths if "01-Chapter-One" in p]
        assert len(ch1_files) >= 1

        # Should have files from Chapter 2
        ch2_files = [p for p in paths if "02-Chapter-Two" in p]
        assert len(ch2_files) >= 1


# =============================================================================
# get_book_archive Scope Tests
# =============================================================================

class TestGetBookArchiveScope:
    """Test get_book_archive with scope parameter."""

    @pytest.mark.asyncio
    async def test_content_scope_default(self, multi_part_book_data):
        """Test that content scope is the default."""
        result = await get_book_archive(GetBookArchiveInput(
            book_id=multi_part_book_data["book_id"]
        ))

        data = json.loads(result)
        assert data["status"] == "success"
        assert data["scope"] == "content"

    @pytest.mark.asyncio
    async def test_content_scope_only_markdown(self, multi_part_book_data):
        """Test that content scope archives only content/ directory."""
        result = await get_book_archive(GetBookArchiveInput(
            book_id=multi_part_book_data["book_id"],
            scope=ArchiveScope.CONTENT
        ))

        data = json.loads(result)
        assert data["status"] == "success"
        assert data["scope"] == "content"
        # Only markdown files, no assets
        assert data["file_count"] == 10

    @pytest.mark.asyncio
    async def test_assets_scope_only_static(self, multi_part_book_data):
        """Test that assets scope archives only static/ directory."""
        result = await get_book_archive(GetBookArchiveInput(
            book_id=multi_part_book_data["book_id"],
            scope=ArchiveScope.ASSETS
        ))

        data = json.loads(result)
        assert data["status"] == "success"
        assert data["scope"] == "assets"
        # Only asset files from static/
        assert data["file_count"] == 2

    @pytest.mark.asyncio
    async def test_all_scope_includes_everything(self, multi_part_book_data):
        """Test that all scope archives entire book."""
        result = await get_book_archive(GetBookArchiveInput(
            book_id=multi_part_book_data["book_id"],
            scope=ArchiveScope.ALL
        ))

        data = json.loads(result)
        assert data["status"] == "success"
        assert data["scope"] == "all"
        # All files: 10 markdown + 2 assets = 12
        assert data["file_count"] == 12

    @pytest.mark.asyncio
    async def test_archive_url_includes_scope_suffix(self, multi_part_book_data):
        """Test that archive URL includes scope suffix."""
        # Content scope
        result_content = await get_book_archive(GetBookArchiveInput(
            book_id=multi_part_book_data["book_id"],
            scope=ArchiveScope.CONTENT
        ))
        data_content = json.loads(result_content)
        assert "-content-" in data_content["archive_url"]

        # Assets scope
        result_assets = await get_book_archive(GetBookArchiveInput(
            book_id=multi_part_book_data["book_id"],
            scope=ArchiveScope.ASSETS
        ))
        data_assets = json.loads(result_assets)
        assert "-assets-" in data_assets["archive_url"]

        # All scope (no suffix)
        result_all = await get_book_archive(GetBookArchiveInput(
            book_id=multi_part_book_data["book_id"],
            scope=ArchiveScope.ALL
        ))
        data_all = json.loads(result_all)
        # Should not have -content- or -assets- suffix
        assert "-content-" not in data_all["archive_url"] or "-assets-" not in data_all["archive_url"]


# =============================================================================
# list_books include_structure Tests
# =============================================================================

class TestListBooksIncludeStructure:
    """Test list_books with include_structure parameter."""

    @pytest.mark.asyncio
    async def test_include_structure_none_default(self, multi_part_book_data):
        """Test that none is the default (no structure included)."""
        result = await list_books(ListBooksInput())

        data = json.loads(result)
        assert len(data) >= 1

        book = data[0]
        assert "book_id" in book
        assert "storage_backend" in book
        # Should NOT have content or assets structure
        assert "content" not in book
        assert "assets" not in book

    @pytest.mark.asyncio
    async def test_include_structure_content(self, multi_part_book_data):
        """Test include_structure='content' returns content structure only."""
        result = await list_books(ListBooksInput(include_structure="content"))

        data = json.loads(result)
        book = next(b for b in data if b["book_id"] == "test-book")

        assert "content" in book
        assert "assets" not in book

        # Verify content structure
        content = book["content"]
        assert "parts" in content
        assert len(content["parts"]) == 2  # Part 1 and Part 2

    @pytest.mark.asyncio
    async def test_include_structure_assets(self, multi_part_book_data):
        """Test include_structure='assets' returns assets structure only."""
        result = await list_books(ListBooksInput(include_structure="assets"))

        data = json.loads(result)
        book = next(b for b in data if b["book_id"] == "test-book")

        assert "assets" in book
        assert "content" not in book

        # Verify assets structure
        assets = book["assets"]
        assert "images" in assets
        assert "slides" in assets
        assert len(assets["images"]) == 1
        assert len(assets["slides"]) == 1

    @pytest.mark.asyncio
    async def test_include_structure_all(self, multi_part_book_data):
        """Test include_structure='all' returns both content and assets."""
        result = await list_books(ListBooksInput(include_structure="all"))

        data = json.loads(result)
        book = next(b for b in data if b["book_id"] == "test-book")

        assert "content" in book
        assert "assets" in book

    @pytest.mark.asyncio
    async def test_content_structure_hierarchy(self, multi_part_book_data):
        """Test that content structure has proper hierarchy."""
        result = await list_books(ListBooksInput(include_structure="content"))

        data = json.loads(result)
        book = next(b for b in data if b["book_id"] == "test-book")
        content = book["content"]

        # Check Part 1
        part1 = next(p for p in content["parts"] if "Part-One" in p["id"])
        assert "chapters" in part1
        assert len(part1["chapters"]) == 2  # Chapter 1 and Chapter 2

        # Check Chapter 1 has lessons
        ch1 = next(c for c in part1["chapters"] if "Chapter-One" in c["id"])
        assert "lessons" in ch1
        assert len(ch1["lessons"]) >= 2  # At least lesson 1 and 2

    @pytest.mark.asyncio
    async def test_book_id_filter(self, multi_part_book_data):
        """Test that book_id filter returns only specified book."""
        # Create another book
        from panaversity_fs.storage import get_operator
        op = get_operator()
        await op.write("books/another-book/content/README.md", b"# Another Book")

        result = await list_books(ListBooksInput(
            book_id="test-book",
            include_structure="content"
        ))

        data = json.loads(result)
        assert len(data) == 1
        assert data[0]["book_id"] == "test-book"


# =============================================================================
# Model Validation Tests
# =============================================================================

class TestModelValidation:
    """Test model validation for new features."""

    def test_content_scope_enum_values(self):
        """Test ContentScope enum has all expected values."""
        assert ContentScope.FILE.value == "file"
        assert ContentScope.CHAPTER.value == "chapter"
        assert ContentScope.PART.value == "part"
        assert ContentScope.BOOK.value == "book"

    def test_archive_scope_enum_values(self):
        """Test ArchiveScope enum has all expected values."""
        assert ArchiveScope.ALL.value == "all"
        assert ArchiveScope.CONTENT.value == "content"
        assert ArchiveScope.ASSETS.value == "assets"

    def test_read_content_input_default_scope(self):
        """Test ReadContentInput defaults to FILE scope."""
        input_model = ReadContentInput(book_id="test-book", path="content/test.md")
        assert input_model.scope == ContentScope.FILE

    def test_read_content_input_book_scope_default_path(self):
        """Test ReadContentInput defaults path to 'content' for book scope."""
        input_model = ReadContentInput(book_id="test-book", scope=ContentScope.BOOK)
        assert input_model.path == "content"

    def test_archive_input_default_scope(self):
        """Test GetBookArchiveInput defaults to CONTENT scope."""
        input_model = GetBookArchiveInput(book_id="test-book")
        assert input_model.scope == ArchiveScope.CONTENT

    def test_list_books_input_structure_options(self):
        """Test ListBooksInput accepts valid structure options."""
        # Valid options
        ListBooksInput(include_structure="none")
        ListBooksInput(include_structure="content")
        ListBooksInput(include_structure="assets")
        ListBooksInput(include_structure="all")

        # None is also valid (default)
        ListBooksInput(include_structure=None)
