"""Integration tests for content workflows.

Updated for FR-002/003/004/005: Journal-backed conflict detection.
"""

import pytest
import json
from panaversity_fs.tools.content import read_content, write_content, delete_content
from panaversity_fs.models import ReadContentInput, WriteContentInput, DeleteContentInput
from panaversity_fs.errors import ConflictError, HashRequiredError


class TestContentCRUDWorkflow:
    """Test complete CRUD workflow for content."""

    @pytest.mark.asyncio
    async def test_complete_crud_workflow(self, setup_fs_backend, mock_context):
        """Test create -> read -> update -> delete workflow."""
        book_id = "test-book"
        path = "content/01-Part/01-Chapter/01-workflow-test.md"
        content_v1 = "# Version 1\n\nOriginal content."
        content_v2 = "# Version 2\n\nUpdated content."

        # 1. CREATE
        create_result = await write_content(WriteContentInput(
            book_id=book_id,
            path=path,
            content=content_v1
        ), mock_context)
        create_data = json.loads(create_result)
        assert create_data["status"] == "success"
        assert create_data["mode"] == "created"
        hash_v1 = create_data["file_hash"]

        # 2. READ
        read_result = await read_content(ReadContentInput(
            book_id=book_id,
            path=path
        ), mock_context)
        read_data = json.loads(read_result)
        assert read_data["content"] == content_v1
        assert read_data["file_hash_sha256"] == hash_v1

        # 3. UPDATE (with conflict detection - expected_hash required per FR-004)
        update_result = await write_content(WriteContentInput(
            book_id=book_id,
            path=path,
            content=content_v2,
            expected_hash=hash_v1
        ), mock_context)
        update_data = json.loads(update_result)
        assert update_data["status"] == "success"
        assert update_data["mode"] == "updated"
        hash_v2 = update_data["file_hash"]
        assert hash_v2 != hash_v1

        # 4. VERIFY UPDATE
        verify_result = await read_content(ReadContentInput(
            book_id=book_id,
            path=path
        ), mock_context)
        verify_data = json.loads(verify_result)
        assert verify_data["content"] == content_v2

        # 5. DELETE
        delete_result = await delete_content(DeleteContentInput(
            book_id=book_id,
            path=path
        ), mock_context)
        delete_data = json.loads(delete_result)
        assert delete_data["existed"] is True

        # 6. VERIFY DELETION - should return error string
        verify_delete_result = await read_content(ReadContentInput(
            book_id=book_id,
            path=path
        ), mock_context)
        assert isinstance(verify_delete_result, str)
        assert "error" in verify_delete_result.lower() or "not found" in verify_delete_result.lower()


class TestConcurrentModificationDetection:
    """Test conflict detection in concurrent scenarios."""

    @pytest.mark.asyncio
    async def test_concurrent_update_detection(self, setup_fs_backend, mock_context):
        """Test that concurrent updates are detected (FR-003, FR-004)."""
        book_id = "test-book"
        path = "content/01-Part/01-Chapter/01-concurrent-test.md"

        # Initial write
        initial_content = "# Initial\n\nContent."
        write1 = await write_content(WriteContentInput(
            book_id=book_id,
            path=path,
            content=initial_content
        ), mock_context)
        data1 = json.loads(write1)
        hash1 = data1["file_hash"]

        # Simulate User A reads
        read_a = await read_content(ReadContentInput(book_id=book_id, path=path), mock_context)
        data_a = json.loads(read_a)
        hash_a = data_a["file_hash_sha256"]

        # Simulate User B updates (with expected_hash per FR-004)
        user_b_content = "# User B Update\n\nB's changes."
        write_b = await write_content(WriteContentInput(
            book_id=book_id,
            path=path,
            content=user_b_content,
            expected_hash=hash1  # B has the current hash
        ), mock_context)
        data_b = json.loads(write_b)
        hash_b = data_b["file_hash"]

        # User A tries to update with stale hash - should fail (FR-003)
        user_a_content = "# User A Update\n\nA's changes."
        with pytest.raises(ConflictError) as exc_info:
            await write_content(WriteContentInput(
                book_id=book_id,
                path=path,
                content=user_a_content,
                expected_hash=hash_a  # Stale hash
            ), mock_context)

        assert "Conflict detected" in str(exc_info.value)

        # Verify User B's content is preserved
        final_read = await read_content(ReadContentInput(book_id=book_id, path=path), mock_context)
        final_data = json.loads(final_read)
        assert "User B Update" in final_data["content"]

    @pytest.mark.asyncio
    async def test_update_without_hash_rejected(self, setup_fs_backend, mock_context):
        """Test that updates without expected_hash are rejected (FR-004)."""
        book_id = "test-book"
        path = "content/01-Part/01-Chapter/01-hash-required-test.md"

        # Create initial file
        await write_content(WriteContentInput(
            book_id=book_id,
            path=path,
            content="# Initial"
        ), mock_context)

        # Try to update without expected_hash - should be rejected
        with pytest.raises(HashRequiredError) as exc_info:
            await write_content(WriteContentInput(
                book_id=book_id,
                path=path,
                content="# Updated without hash"
            ), mock_context)

        assert "Hash required" in str(exc_info.value)


class TestBulkContentOperations:
    """Test operations on multiple content files."""

    @pytest.mark.asyncio
    async def test_create_multiple_lessons(self, setup_fs_backend, mock_context):
        """Test creating multiple lessons in sequence."""
        book_id = "test-book"
        lessons = [
            ("content/01-Part/01-Chapter/01-lesson.md", "# Lesson 1\n\nFirst lesson."),
            ("content/01-Part/01-Chapter/02-lesson.md", "# Lesson 2\n\nSecond lesson."),
            ("content/01-Part/01-Chapter/03-lesson.md", "# Lesson 3\n\nThird lesson."),
        ]

        # Create all lessons
        hashes = []
        for path, content in lessons:
            result = await write_content(WriteContentInput(
                book_id=book_id,
                path=path,
                content=content
            ), mock_context)
            data = json.loads(result)
            assert data["status"] == "success"
            hashes.append(data["file_hash"])

        # Verify all hashes are unique
        assert len(set(hashes)) == len(hashes)

        # Read all lessons
        for path, expected_content in lessons:
            result = await read_content(ReadContentInput(
                book_id=book_id,
                path=path
            ), mock_context)
            data = json.loads(result)
            assert data["content"] == expected_content
