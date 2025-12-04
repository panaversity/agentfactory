"""Unit tests for content operation tools.

Updated for ADR-0018: Uses Docusaurus-aligned content/ structure.
Updated for FR-002/003/004/005: Journal-backed conflict detection.
"""

import pytest
import json
from panaversity_fs.tools.content import read_content, write_content, delete_content
from panaversity_fs.models import ReadContentInput, WriteContentInput, DeleteContentInput
from panaversity_fs.errors import ContentNotFoundError, ConflictError, HashRequiredError


class TestReadContent:
    """Test read_content tool."""

    @pytest.mark.asyncio
    async def test_read_existing_content(self, sample_book_data):
        """Test reading existing lesson content."""
        result = await read_content(ReadContentInput(
            book_id=sample_book_data["book_id"],
            path=sample_book_data["lesson_path"]
        ))

        data = json.loads(result)
        assert "content" in data
        assert "file_hash_sha256" in data
        assert "file_size" in data
        assert "Test Lesson" in data["content"]
        assert len(data["file_hash_sha256"]) == 64

    @pytest.mark.asyncio
    async def test_read_nonexistent_content(self, setup_fs_backend):
        """Test reading non-existent content returns error string."""
        result = await read_content(ReadContentInput(
            book_id="test-book",
            path="content/01-Part/01-Chapter/nonexistent.md"
        ))

        # MCP tools return error strings instead of raising exceptions
        assert isinstance(result, str)
        assert "error" in result.lower() or "not found" in result.lower()

    @pytest.mark.asyncio
    async def test_read_content_includes_metadata(self, sample_book_data):
        """Test that read_content includes all metadata fields."""
        result = await read_content(ReadContentInput(
            book_id=sample_book_data["book_id"],
            path=sample_book_data["lesson_path"]
        ))

        data = json.loads(result)
        required_fields = ["content", "file_size", "last_modified",
                          "storage_backend", "file_hash_sha256"]
        for field in required_fields:
            assert field in data, f"Missing field: {field}"


class TestWriteContent:
    """Test write_content tool."""

    @pytest.mark.asyncio
    async def test_create_new_content(self, setup_fs_backend, sample_lesson_content):
        """Test creating new content."""
        result = await write_content(WriteContentInput(
            book_id="test-book",
            path="content/01-Part/01-Chapter/new-lesson.md",
            content=sample_lesson_content
        ))

        data = json.loads(result)
        assert data["status"] == "success"
        assert data["mode"] == "created"
        assert "file_hash" in data

    @pytest.mark.asyncio
    async def test_update_with_correct_hash(self, sample_book_data, sample_lesson_content):
        """Test updating content with correct expected_hash (FR-003)."""
        # First read to get hash
        read_result = await read_content(ReadContentInput(
            book_id=sample_book_data["book_id"],
            path=sample_book_data["lesson_path"]
        ))
        read_data = json.loads(read_result)
        file_hash = read_data["file_hash_sha256"]

        # Update with correct expected_hash
        new_content = "# Updated Lesson\n\nNew content."
        result = await write_content(WriteContentInput(
            book_id=sample_book_data["book_id"],
            path=sample_book_data["lesson_path"],
            content=new_content,
            expected_hash=file_hash
        ))

        data = json.loads(result)
        assert data["status"] == "success"
        assert data["mode"] == "updated"

    @pytest.mark.asyncio
    async def test_conflict_detection_wrong_hash(self, sample_book_data):
        """Test conflict detection with wrong expected_hash (FR-003)."""
        with pytest.raises(ConflictError) as exc_info:
            await write_content(WriteContentInput(
                book_id=sample_book_data["book_id"],
                path=sample_book_data["lesson_path"],
                content="New content",
                expected_hash="0" * 64  # Wrong hash
            ))

        assert "Conflict detected" in str(exc_info.value)

    @pytest.mark.asyncio
    async def test_update_without_hash_rejected(self, sample_book_data, sample_lesson_content):
        """Test that updating existing file without expected_hash is rejected (FR-004)."""
        # Try to update without expected_hash - should be rejected
        with pytest.raises(HashRequiredError) as exc_info:
            await write_content(WriteContentInput(
                book_id=sample_book_data["book_id"],
                path=sample_book_data["lesson_path"],
                content="# Updated content"
            ))

        assert "Hash required" in str(exc_info.value)
        # Error should include the current hash for retry
        assert sample_book_data["book_id"] in str(exc_info.value) or "current" in str(exc_info.value).lower()

    @pytest.mark.asyncio
    async def test_create_without_hash_succeeds(self, setup_fs_backend, sample_lesson_content):
        """Test creating new file without expected_hash succeeds (FR-005)."""
        result = await write_content(WriteContentInput(
            book_id="test-book",
            path="content/01-Part/01-Chapter/new-file.md",
            content=sample_lesson_content
        ))
        data = json.loads(result)
        assert data["mode"] == "created"
        assert data["status"] == "success"

    @pytest.mark.asyncio
    async def test_update_nonexistent_with_hash_rejected(self, setup_fs_backend):
        """Test that providing expected_hash for non-existent file is rejected."""
        with pytest.raises(ContentNotFoundError):
            await write_content(WriteContentInput(
                book_id="test-book",
                path="content/01-Part/01-Chapter/nonexistent.md",
                content="# New content",
                expected_hash="a" * 64  # Hash for non-existent file
            ))


class TestDeleteContent:
    """Test delete_content tool."""

    @pytest.mark.asyncio
    async def test_delete_existing_content(self, sample_book_data):
        """Test deleting existing content."""
        result = await delete_content(DeleteContentInput(
            book_id=sample_book_data["book_id"],
            path=sample_book_data["lesson_path"]
        ))

        data = json.loads(result)
        assert data["status"] == "success"
        assert data["existed"] is True

        # Verify deletion - should return error string
        verify_result = await read_content(ReadContentInput(
            book_id=sample_book_data["book_id"],
            path=sample_book_data["lesson_path"]
        ))
        assert isinstance(verify_result, str)
        assert "error" in verify_result.lower() or "not found" in verify_result.lower()

    @pytest.mark.asyncio
    async def test_delete_nonexistent_content_idempotent(self, setup_fs_backend):
        """Test that deleting non-existent content is idempotent."""
        result = await delete_content(DeleteContentInput(
            book_id="test-book",
            path="content/01-Part/01-Chapter/nonexistent.md"
        ))

        data = json.loads(result)
        assert data["status"] == "success"
        assert data["existed"] is False

    @pytest.mark.asyncio
    async def test_delete_twice_idempotent(self, sample_book_data):
        """Test that deleting twice is idempotent."""
        # First delete
        result1 = await delete_content(DeleteContentInput(
            book_id=sample_book_data["book_id"],
            path=sample_book_data["lesson_path"]
        ))
        data1 = json.loads(result1)
        assert data1["existed"] is True

        # Second delete
        result2 = await delete_content(DeleteContentInput(
            book_id=sample_book_data["book_id"],
            path=sample_book_data["lesson_path"]
        ))
        data2 = json.loads(result2)
        assert data2["existed"] is False
