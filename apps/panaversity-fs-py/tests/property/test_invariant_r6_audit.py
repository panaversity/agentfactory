"""Property-based tests for audit hash chain integrity (T040).

Tests R6 invariant: entry[n].new_hash == entry[n+1].prev_hash
Uses hypothesis for property-based testing.
"""

import pytest
from hypothesis import given, strategies as st, settings, HealthCheck
from panaversity_fs.audit import log_operation, verify_hash_chain
from panaversity_fs.models import OperationType, OperationStatus
from panaversity_fs.database.connection import get_session
from panaversity_fs.database.models import AuditLog
from sqlalchemy import select


# Each test generates unique UUIDs so fixture state doesn't matter
HYPOTHESIS_SETTINGS = {
    "max_examples": 10,
    "deadline": None,
    "suppress_health_check": [HealthCheck.function_scoped_fixture]
}


# Strategies for generating test data
hash_strategy = st.text(
    alphabet="0123456789abcdef",
    min_size=64,
    max_size=64
)

operation_count_strategy = st.integers(min_value=3, max_value=5)


class TestAuditChainInvariantR6:
    """Property tests for R6: Audit hash chain integrity.

    R6 invariant: For any file's audit trail, consecutive entries satisfy:
    entry[n].new_hash == entry[n+1].prev_hash
    """

    @pytest.mark.asyncio
    @given(
        op_count=operation_count_strategy,
        hashes=st.lists(hash_strategy, min_size=3, max_size=5)
    )
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_chain_integrity_maintained(self, setup_fs_backend, op_count, hashes):
        """R6: Hash chain is maintained across multiple operations."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        path = f"content/01-Part/01-Chapter/r6-{unique_id}.md"
        book_id = "test-book"
        full_path = f"books/{book_id}/{path}"

        # Use exactly op_count hashes
        test_hashes = hashes[:op_count]

        # Log multiple operations
        for i, hash_val in enumerate(test_hashes):
            await log_operation(
                operation=OperationType.WRITE_CONTENT,
                path=full_path,
                agent_id=f"test-agent-{i}",
                status=OperationStatus.SUCCESS,
                new_hash=hash_val,
                book_id=book_id
            )

        # Verify chain integrity
        chain_result = await verify_hash_chain(book_id, path)

        assert chain_result["valid"] is True, \
            f"Hash chain broken at indices: {chain_result['breaks']}"
        assert chain_result["entries"] == len(test_hashes)

    @pytest.mark.asyncio
    @given(hash1=hash_strategy, hash2=hash_strategy)
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_two_operations_link_correctly(self, setup_fs_backend, hash1, hash2):
        """R6: Two consecutive operations are linked by prev_hash."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        path = f"content/01-Part/01-Chapter/r6-two-{unique_id}.md"
        book_id = "test-book"
        full_path = f"books/{book_id}/{path}"

        # First operation
        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=full_path,
            agent_id="agent-1",
            status=OperationStatus.SUCCESS,
            new_hash=hash1,
            book_id=book_id
        )

        # Second operation
        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=full_path,
            agent_id="agent-2",
            status=OperationStatus.SUCCESS,
            new_hash=hash2,
            book_id=book_id
        )

        # Verify the specific link
        async with get_session() as session:
            stmt = select(AuditLog).where(
                AuditLog.book_id == book_id,
                AuditLog.path == path
            ).order_by(AuditLog.timestamp, AuditLog.id)
            result = await session.execute(stmt)
            entries = result.scalars().all()

            assert len(entries) == 2
            # R6 assertion: entry[0].new_hash == entry[1].prev_hash
            assert entries[0].new_hash == entries[1].prev_hash

    @pytest.mark.asyncio
    @given(
        create_hash=hash_strategy,
        update_hashes=st.lists(hash_strategy, min_size=1, max_size=3)
    )
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_create_update_chain(self, setup_fs_backend, create_hash, update_hashes):
        """R6: Create followed by updates maintains chain."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        path = f"content/01-Part/01-Chapter/r6-create-update-{unique_id}.md"
        book_id = "test-book"
        full_path = f"books/{book_id}/{path}"

        # Create operation (first entry)
        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=full_path,
            agent_id="agent-create",
            status=OperationStatus.SUCCESS,
            new_hash=create_hash,
            book_id=book_id
        )

        # Multiple updates
        for i, hash_val in enumerate(update_hashes):
            await log_operation(
                operation=OperationType.WRITE_CONTENT,
                path=full_path,
                agent_id=f"agent-update-{i}",
                status=OperationStatus.SUCCESS,
                new_hash=hash_val,
                book_id=book_id
            )

        # Verify entire chain
        chain_result = await verify_hash_chain(book_id, path)

        assert chain_result["valid"] is True
        assert chain_result["entries"] == 1 + len(update_hashes)

    @pytest.mark.asyncio
    @given(hash_before_delete=hash_strategy)
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_delete_preserves_chain(self, setup_fs_backend, hash_before_delete):
        """R6: Delete operation (new_hash=None) still maintains chain link."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        path = f"content/01-Part/01-Chapter/r6-delete-{unique_id}.md"
        book_id = "test-book"
        full_path = f"books/{book_id}/{path}"

        # Create
        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=full_path,
            agent_id="agent-create",
            status=OperationStatus.SUCCESS,
            new_hash=hash_before_delete,
            book_id=book_id
        )

        # Delete (new_hash=None)
        await log_operation(
            operation=OperationType.DELETE_CONTENT,
            path=full_path,
            agent_id="agent-delete",
            status=OperationStatus.SUCCESS,
            new_hash=None,
            book_id=book_id
        )

        # Verify chain: delete should link to create
        async with get_session() as session:
            stmt = select(AuditLog).where(
                AuditLog.book_id == book_id,
                AuditLog.path == path
            ).order_by(AuditLog.timestamp, AuditLog.id)
            result = await session.execute(stmt)
            entries = result.scalars().all()

            assert len(entries) == 2
            # Delete's prev_hash should link to create's new_hash
            assert entries[1].prev_hash == hash_before_delete
            assert entries[1].new_hash is None  # Delete has no new hash


class TestChainVerificationFunction:
    """Test the verify_hash_chain utility function."""

    @pytest.mark.asyncio
    @given(hashes=st.lists(hash_strategy, min_size=2, max_size=4))
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_verify_detects_valid_chain(self, setup_fs_backend, hashes):
        """verify_hash_chain correctly identifies valid chains."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        path = f"content/01-Part/01-Chapter/verify-{unique_id}.md"
        book_id = "test-book"
        full_path = f"books/{book_id}/{path}"

        # Log operations
        for i, hash_val in enumerate(hashes):
            await log_operation(
                operation=OperationType.WRITE_CONTENT,
                path=full_path,
                agent_id=f"agent-{i}",
                status=OperationStatus.SUCCESS,
                new_hash=hash_val,
                book_id=book_id
            )

        # Verify
        result = await verify_hash_chain(book_id, path)

        assert result["valid"] is True
        assert result["entries"] == len(hashes)
        assert result["breaks"] == []

    @pytest.mark.asyncio
    async def test_verify_single_entry_is_valid(self, setup_fs_backend):
        """A single audit entry is considered a valid chain."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        path = f"content/01-Part/01-Chapter/single-{unique_id}.md"
        book_id = "test-book"

        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=f"books/{book_id}/{path}",
            agent_id="agent-solo",
            status=OperationStatus.SUCCESS,
            new_hash="a" * 64,
            book_id=book_id
        )

        result = await verify_hash_chain(book_id, path)

        assert result["valid"] is True
        assert result["entries"] == 1

    @pytest.mark.asyncio
    async def test_verify_empty_returns_valid(self, setup_fs_backend):
        """No audit entries for a path returns valid chain (vacuously true)."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        path = f"content/nonexistent-{unique_id}.md"

        result = await verify_hash_chain("no-such-book", path)

        assert result["valid"] is True
        assert result["entries"] == 0
