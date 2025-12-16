"""Integration tests for conflict detection (T032).

Tests concurrent write conflicts and journal-based detection per FR-002, FR-003, FR-004.
"""

import pytest
import json
import asyncio
from panaversity_fs.tools.content import read_content, write_content
from panaversity_fs.models import ReadContentInput, WriteContentInput
from panaversity_fs.errors import ConflictError, HashRequiredError, ContentNotFoundError


class TestConflictDetection:
    """Integration tests for hash-based conflict detection (FR-003)."""

    @pytest.mark.asyncio
    async def test_concurrent_writes_one_wins(self, setup_fs_backend, mock_context):
        """Two concurrent updates - first wins, second gets conflict."""
        # Create initial file
        create_result = await write_content(WriteContentInput(
            book_id="test-book",
            path="content/01-Part/01-Chapter/01-concurrent-test.md",
            content="# Initial Content"
        ), mock_context)
        create_data = json.loads(create_result)
        initial_hash = create_data["file_hash"]

        # Simulate two agents reading simultaneously
        agent1_hash = initial_hash
        agent2_hash = initial_hash

        # Agent 1 writes first (succeeds)
        result1 = await write_content(WriteContentInput(
            book_id="test-book",
            path="content/01-Part/01-Chapter/01-concurrent-test.md",
            content="# Agent 1 Update",
            expected_hash=agent1_hash
        ), mock_context)
        data1 = json.loads(result1)
        assert data1["status"] == "success"
        assert data1["mode"] == "updated"

        # Agent 2 tries to write with stale hash (conflict)
        with pytest.raises(ConflictError) as exc_info:
            await write_content(WriteContentInput(
                book_id="test-book",
                path="content/01-Part/01-Chapter/01-concurrent-test.md",
                content="# Agent 2 Update",
                expected_hash=agent2_hash  # Stale hash
            ), mock_context)

        # Conflict error contains the current hash for retry
        assert data1["file_hash"] in str(exc_info.value) or "Conflict" in str(exc_info.value)

    @pytest.mark.asyncio
    async def test_hash_required_for_existing_file(self, setup_fs_backend, mock_context):
        """FR-004: Updates to existing files require expected_hash."""
        # Create file
        await write_content(WriteContentInput(
            book_id="test-book",
            path="content/01-Part/01-Chapter/01-hash-required.md",
            content="# Initial"
        ), mock_context)

        # Try to update without expected_hash
        with pytest.raises(HashRequiredError) as exc_info:
            await write_content(WriteContentInput(
                book_id="test-book",
                path="content/01-Part/01-Chapter/01-hash-required.md",
                content="# Updated without hash"
            ), mock_context)

        # Error should include current hash for reference
        assert "Hash required" in str(exc_info.value)

    @pytest.mark.asyncio
    async def test_create_without_hash_succeeds(self, setup_fs_backend, mock_context):
        """FR-005: New files don't need expected_hash."""
        result = await write_content(WriteContentInput(
            book_id="test-book",
            path="content/01-Part/01-Chapter/01-new-file.md",
            content="# New File Content"
        ), mock_context)

        data = json.loads(result)
        assert data["status"] == "success"
        assert data["mode"] == "created"

    @pytest.mark.asyncio
    async def test_update_with_expected_hash_nonexistent(self, setup_fs_backend, mock_context):
        """Providing expected_hash for non-existent file should fail."""
        with pytest.raises(ContentNotFoundError):
            await write_content(WriteContentInput(
                book_id="test-book",
                path="content/01-Part/01-Chapter/01-does-not-exist.md",
                content="# Content",
                expected_hash="a" * 64
            ), mock_context)

    @pytest.mark.asyncio
    async def test_sequential_updates_with_hash_chain(self, setup_fs_backend, mock_context):
        """Multiple sequential updates maintain hash chain."""
        # Create
        r1 = await write_content(WriteContentInput(
            book_id="test-book",
            path="content/01-Part/01-Chapter/01-chain.md",
            content="# Version 1"
        ), mock_context)
        d1 = json.loads(r1)
        hash1 = d1["file_hash"]

        # Update 1
        r2 = await write_content(WriteContentInput(
            book_id="test-book",
            path="content/01-Part/01-Chapter/01-chain.md",
            content="# Version 2",
            expected_hash=hash1
        ), mock_context)
        d2 = json.loads(r2)
        hash2 = d2["file_hash"]
        assert hash2 != hash1  # Different content = different hash

        # Update 2
        r3 = await write_content(WriteContentInput(
            book_id="test-book",
            path="content/01-Part/01-Chapter/01-chain.md",
            content="# Version 3",
            expected_hash=hash2
        ), mock_context)
        d3 = json.loads(r3)
        hash3 = d3["file_hash"]
        assert hash3 != hash2

        # Verify final content
        read_result = await read_content(ReadContentInput(
            book_id="test-book",
            path="content/01-Part/01-Chapter/01-chain.md"
        ), mock_context)
        read_data = json.loads(read_result)
        assert "Version 3" in read_data["content"]
        assert read_data["file_hash_sha256"] == hash3


class TestJournalStorageAtomic:
    """Integration tests for atomic journal+storage operations (T031, FR-002)."""

    @pytest.mark.asyncio
    async def test_write_creates_journal_and_storage(self, setup_fs_backend, mock_context):
        """Write operation creates both journal entry and storage file."""
        from panaversity_fs.database.connection import get_session
        from panaversity_fs.database.models import FileJournal
        from panaversity_fs.storage import get_operator
        from sqlalchemy import select

        result = await write_content(WriteContentInput(
            book_id="test-book",
            path="content/01-Part/01-Chapter/01-atomic-test.md",
            content="# Atomic Test"
        ), mock_context)
        data = json.loads(result)

        # Verify journal entry exists
        async with get_session() as session:
            stmt = select(FileJournal).where(
                FileJournal.book_id == "test-book",
                FileJournal.path == "content/01-Part/01-Chapter/01-atomic-test.md",
                FileJournal.user_id == "__base__"
            )
            result = await session.execute(stmt)
            entry = result.scalar_one_or_none()

            assert entry is not None
            assert entry.sha256 == data["file_hash"]

        # Verify storage file exists
        op = get_operator()
        content = await op.read("books/test-book/content/01-Part/01-Chapter/01-atomic-test.md")
        assert b"Atomic Test" in content

    @pytest.mark.asyncio
    async def test_update_modifies_journal_hash(self, setup_fs_backend, mock_context):
        """Update operation modifies journal hash to new value."""
        from panaversity_fs.database.connection import get_session
        from panaversity_fs.database.models import FileJournal
        from sqlalchemy import select

        # Create
        r1 = await write_content(WriteContentInput(
            book_id="test-book",
            path="content/01-Part/01-Chapter/01-update-journal.md",
            content="# Original"
        ), mock_context)
        d1 = json.loads(r1)
        original_hash = d1["file_hash"]

        # Update
        r2 = await write_content(WriteContentInput(
            book_id="test-book",
            path="content/01-Part/01-Chapter/01-update-journal.md",
            content="# Modified",
            expected_hash=original_hash
        ), mock_context)
        d2 = json.loads(r2)
        new_hash = d2["file_hash"]

        # Verify journal has new hash
        async with get_session() as session:
            stmt = select(FileJournal).where(
                FileJournal.book_id == "test-book",
                FileJournal.path == "content/01-Part/01-Chapter/01-update-journal.md",
                FileJournal.user_id == "__base__"
            )
            result = await session.execute(stmt)
            entry = result.scalar_one_or_none()

            assert entry.sha256 == new_hash
            assert entry.sha256 != original_hash

    @pytest.mark.asyncio
    async def test_conflict_does_not_modify_storage(self, setup_fs_backend, mock_context):
        """Failed conflict check leaves storage unchanged."""
        from panaversity_fs.storage import get_operator

        # Create file
        r1 = await write_content(WriteContentInput(
            book_id="test-book",
            path="content/01-Part/01-Chapter/01-conflict-no-change.md",
            content="# Original Content"
        ), mock_context)

        # Try to update with wrong hash (should fail)
        with pytest.raises(ConflictError):
            await write_content(WriteContentInput(
                book_id="test-book",
                path="content/01-Part/01-Chapter/01-conflict-no-change.md",
                content="# Should Not Be Written",
                expected_hash="0" * 64
            ), mock_context)

        # Verify storage still has original content
        op = get_operator()
        content = await op.read("books/test-book/content/01-Part/01-Chapter/01-conflict-no-change.md")
        assert b"Original Content" in content
        assert b"Should Not Be Written" not in content
