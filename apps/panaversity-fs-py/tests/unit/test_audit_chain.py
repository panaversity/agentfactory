"""Unit tests for audit hash chain integrity (T039).

Tests FR-022: entry[n].new_hash == entry[n+1].prev_hash
"""

import pytest
import json
from datetime import datetime, timezone
from panaversity_fs.audit import log_operation, query_audit_log, verify_hash_chain
from panaversity_fs.models import OperationType, OperationStatus
from panaversity_fs.database.connection import get_session
from panaversity_fs.database.models import AuditLog
from sqlalchemy import select


class TestAuditHashChain:
    """Test hash chain integrity for audit log entries (FR-022)."""

    @pytest.mark.asyncio
    async def test_first_entry_has_no_prev_hash(self, setup_fs_backend):
        """First audit entry for a path should have prev_hash=None."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        path = f"content/01-Part/01-Chapter/{unique_id}-first.md"
        book_id = "test-book"

        # Log first operation
        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=f"books/{book_id}/{path}",
            agent_id="test-agent-1",
            status=OperationStatus.SUCCESS,
            new_hash="a" * 64,
            book_id=book_id
        )

        # Verify prev_hash is None for first entry
        async with get_session() as session:
            stmt = select(AuditLog).where(
                AuditLog.book_id == book_id,
                AuditLog.path == path
            )
            result = await session.execute(stmt)
            entry = result.scalar_one_or_none()

            assert entry is not None
            assert entry.prev_hash is None
            assert entry.new_hash == "a" * 64

    @pytest.mark.asyncio
    async def test_second_entry_links_to_first(self, setup_fs_backend):
        """Second audit entry should have prev_hash == first entry's new_hash."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        path = f"content/01-Part/01-Chapter/{unique_id}-chain.md"
        book_id = "test-book"
        full_path = f"books/{book_id}/{path}"

        first_hash = "a" * 64
        second_hash = "b" * 64

        # Log first operation
        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=full_path,
            agent_id="test-agent-1",
            status=OperationStatus.SUCCESS,
            new_hash=first_hash,
            book_id=book_id
        )

        # Log second operation
        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=full_path,
            agent_id="test-agent-2",
            status=OperationStatus.SUCCESS,
            new_hash=second_hash,
            book_id=book_id
        )

        # Query entries in order
        async with get_session() as session:
            stmt = select(AuditLog).where(
                AuditLog.book_id == book_id,
                AuditLog.path == path
            ).order_by(AuditLog.timestamp, AuditLog.id)
            result = await session.execute(stmt)
            entries = result.scalars().all()

            assert len(entries) == 2
            assert entries[0].prev_hash is None
            assert entries[0].new_hash == first_hash
            assert entries[1].prev_hash == first_hash  # FR-022: chain links
            assert entries[1].new_hash == second_hash

    @pytest.mark.asyncio
    async def test_chain_integrity_across_multiple_operations(self, setup_fs_backend):
        """Verify hash chain integrity across 5 consecutive operations."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        path = f"content/01-Part/01-Chapter/{unique_id}-multi.md"
        book_id = "test-book"
        full_path = f"books/{book_id}/{path}"

        hashes = [f"{chr(97 + i)}" * 64 for i in range(5)]  # a*64, b*64, c*64, d*64, e*64

        # Log 5 operations
        for i, hash_val in enumerate(hashes):
            await log_operation(
                operation=OperationType.WRITE_CONTENT,
                path=full_path,
                agent_id=f"test-agent-{i}",
                status=OperationStatus.SUCCESS,
                new_hash=hash_val,
                book_id=book_id
            )

        # Verify chain integrity using verify_hash_chain
        chain_result = await verify_hash_chain(book_id, path)

        assert chain_result["valid"] is True
        assert chain_result["entries"] == 5
        assert chain_result["breaks"] == []

    @pytest.mark.asyncio
    async def test_delete_operation_has_null_new_hash(self, setup_fs_backend):
        """Delete operations should have new_hash=None in the chain."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        path = f"content/01-Part/01-Chapter/{unique_id}-delete.md"
        book_id = "test-book"
        full_path = f"books/{book_id}/{path}"

        # Create file
        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=full_path,
            agent_id="test-agent-1",
            status=OperationStatus.SUCCESS,
            new_hash="a" * 64,
            book_id=book_id
        )

        # Delete file (new_hash=None)
        await log_operation(
            operation=OperationType.DELETE_CONTENT,
            path=full_path,
            agent_id="test-agent-2",
            status=OperationStatus.SUCCESS,
            new_hash=None,
            book_id=book_id
        )

        # Query entries
        async with get_session() as session:
            stmt = select(AuditLog).where(
                AuditLog.book_id == book_id,
                AuditLog.path == path
            ).order_by(AuditLog.timestamp, AuditLog.id)
            result = await session.execute(stmt)
            entries = result.scalars().all()

            assert len(entries) == 2
            assert entries[1].prev_hash == "a" * 64  # Links to previous
            assert entries[1].new_hash is None  # Delete has no new hash

    @pytest.mark.asyncio
    async def test_different_paths_have_independent_chains(self, setup_fs_backend):
        """Different paths should maintain separate hash chains."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        path1 = f"content/01-Part/01-Chapter/{unique_id}-path1.md"
        path2 = f"content/01-Part/01-Chapter/{unique_id}-path2.md"
        book_id = "test-book"

        # Log to path1
        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=f"books/{book_id}/{path1}",
            agent_id="test-agent-1",
            status=OperationStatus.SUCCESS,
            new_hash="a" * 64,
            book_id=book_id
        )

        # Log to path2
        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=f"books/{book_id}/{path2}",
            agent_id="test-agent-2",
            status=OperationStatus.SUCCESS,
            new_hash="b" * 64,
            book_id=book_id
        )

        # Second operation on path1
        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=f"books/{book_id}/{path1}",
            agent_id="test-agent-3",
            status=OperationStatus.SUCCESS,
            new_hash="c" * 64,
            book_id=book_id
        )

        # Verify path1 chain links correctly (not to path2)
        chain1 = await verify_hash_chain(book_id, path1)
        chain2 = await verify_hash_chain(book_id, path2)

        assert chain1["valid"] is True
        assert chain1["entries"] == 2
        assert chain2["valid"] is True
        assert chain2["entries"] == 1


class TestAuditQueryFilters:
    """Test audit log query filters (FR-024)."""

    @pytest.mark.asyncio
    async def test_query_by_agent_id(self, setup_fs_backend):
        """Query audit entries filtered by agent_id."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"

        # Log operations from different agents
        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=f"books/{book_id}/content/01-Part/01-Chapter/{unique_id}-a.md",
            agent_id="agent-alice",
            status=OperationStatus.SUCCESS,
            new_hash="a" * 64,
            book_id=book_id
        )

        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=f"books/{book_id}/content/01-Part/01-Chapter/{unique_id}-b.md",
            agent_id="agent-bob",
            status=OperationStatus.SUCCESS,
            new_hash="b" * 64,
            book_id=book_id
        )

        # Query only alice's entries
        entries = await query_audit_log(agent_id="agent-alice")
        alice_entries = [e for e in entries if e.agent_id == "agent-alice"]

        assert len(alice_entries) >= 1
        assert all(e.agent_id == "agent-alice" for e in alice_entries)

    @pytest.mark.asyncio
    async def test_query_by_operation_type(self, setup_fs_backend):
        """Query audit entries filtered by operation type."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"

        # Log different operation types
        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=f"books/{book_id}/content/{unique_id}-write.md",
            agent_id="test-agent",
            status=OperationStatus.SUCCESS,
            new_hash="a" * 64,
            book_id=book_id
        )

        await log_operation(
            operation=OperationType.READ_CONTENT,
            path=f"books/{book_id}/content/{unique_id}-read.md",
            agent_id="test-agent",
            status=OperationStatus.SUCCESS,
            book_id=book_id
        )

        # Query only write operations
        entries = await query_audit_log(operation=OperationType.WRITE_CONTENT)

        assert all(e.operation == "write_content" for e in entries)

    @pytest.mark.asyncio
    async def test_query_by_book_id(self, setup_fs_backend):
        """Query audit entries filtered by book_id."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]

        # Log to different books
        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=f"books/book-a/content/{unique_id}.md",
            agent_id="test-agent",
            status=OperationStatus.SUCCESS,
            new_hash="a" * 64,
            book_id="book-a"
        )

        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=f"books/book-b/content/{unique_id}.md",
            agent_id="test-agent",
            status=OperationStatus.SUCCESS,
            new_hash="b" * 64,
            book_id="book-b"
        )

        # Query only book-a
        entries = await query_audit_log(book_id="book-a")

        assert all(e.book_id == "book-a" for e in entries)

    @pytest.mark.asyncio
    async def test_query_limit(self, setup_fs_backend):
        """Query respects limit parameter."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"

        # Log 10 operations
        for i in range(10):
            await log_operation(
                operation=OperationType.WRITE_CONTENT,
                path=f"books/{book_id}/content/{unique_id}-{i}.md",
                agent_id="test-agent",
                status=OperationStatus.SUCCESS,
                new_hash=f"{chr(97 + i)}" * 64,
                book_id=book_id
            )

        # Query with limit=5
        entries = await query_audit_log(book_id=book_id, limit=5)

        assert len(entries) <= 5

    @pytest.mark.asyncio
    async def test_query_returns_most_recent_first(self, setup_fs_backend):
        """Query results are ordered by timestamp descending."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"test-book-{unique_id}"

        # Log operations
        for i in range(3):
            await log_operation(
                operation=OperationType.WRITE_CONTENT,
                path=f"books/{book_id}/content/{unique_id}-{i}.md",
                agent_id="test-agent",
                status=OperationStatus.SUCCESS,
                new_hash=f"{chr(97 + i)}" * 64,
                book_id=book_id
            )

        # Query
        entries = await query_audit_log(book_id=book_id)

        # Verify descending order
        for i in range(len(entries) - 1):
            assert entries[i].timestamp >= entries[i + 1].timestamp
