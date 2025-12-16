"""Property-based tests for agent provenance (T041).

Tests R7 invariant: agent_id != 'system' and agent_id != ''
All audit entries must have real agent identification.
"""

import pytest
from hypothesis import given, strategies as st, settings, HealthCheck, assume
from panaversity_fs.audit import log_operation, query_audit_log
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


# Strategies for generating valid agent IDs
valid_agent_id_strategy = st.text(
    alphabet=st.characters(whitelist_categories=('L', 'N'), whitelist_characters='-_'),
    min_size=3,
    max_size=50
).filter(lambda s: s not in ("system", "") and len(s.strip()) > 0)

# Strategy for hash values
hash_strategy = st.text(
    alphabet="0123456789abcdef",
    min_size=64,
    max_size=64
)

# Strategy for operation types
operation_strategy = st.sampled_from([
    OperationType.READ_CONTENT,
    OperationType.WRITE_CONTENT,
    OperationType.DELETE_CONTENT,
])


class TestAgentProvenanceInvariantR7:
    """Property tests for R7: Agent provenance.

    R7 invariant: All audit entries have valid agent_id:
    - agent_id != 'system'
    - agent_id != ''
    - agent_id is not None
    """

    @pytest.mark.asyncio
    @given(agent_id=valid_agent_id_strategy, hash_val=hash_strategy)
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_valid_agent_id_recorded(self, setup_fs_backend, agent_id, hash_val):
        """R7: Valid agent IDs are recorded correctly."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        path = f"content/01-Part/01-Chapter/r7-{unique_id}.md"
        book_id = "test-book"
        full_path = f"books/{book_id}/{path}"

        # Log operation with valid agent_id
        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=full_path,
            agent_id=agent_id,
            status=OperationStatus.SUCCESS,
            new_hash=hash_val,
            book_id=book_id
        )

        # Verify agent_id recorded correctly
        async with get_session() as session:
            stmt = select(AuditLog).where(
                AuditLog.book_id == book_id,
                AuditLog.path == path
            ).order_by(AuditLog.id.desc()).limit(1)
            result = await session.execute(stmt)
            entry = result.scalar_one_or_none()

            assert entry is not None
            assert entry.agent_id == agent_id
            # R7: Not system or empty
            assert entry.agent_id != "system"
            assert entry.agent_id != ""
            assert entry.agent_id is not None

    @pytest.mark.asyncio
    @given(
        agent_ids=st.lists(valid_agent_id_strategy, min_size=3, max_size=5, unique=True)
    )
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_multiple_agents_distinguishable(self, setup_fs_backend, agent_ids):
        """R7: Different agents create distinguishable audit entries."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"test-book-{unique_id}"

        # Log operations from different agents
        for i, agent_id in enumerate(agent_ids):
            await log_operation(
                operation=OperationType.WRITE_CONTENT,
                path=f"books/{book_id}/content/{unique_id}-{i}.md",
                agent_id=agent_id,
                status=OperationStatus.SUCCESS,
                new_hash=f"{chr(97 + i)}" * 64,
                book_id=book_id
            )

        # Query all entries for this book
        entries = await query_audit_log(book_id=book_id)

        # Verify all agent_ids are preserved
        recorded_agent_ids = {e.agent_id for e in entries}

        # All our agent IDs should be recorded
        for agent_id in agent_ids:
            assert agent_id in recorded_agent_ids

        # R7: None should be system or empty
        for entry in entries:
            assert entry.agent_id != "system"
            assert entry.agent_id != ""

    @pytest.mark.asyncio
    @given(
        agent_id=valid_agent_id_strategy,
        operation=operation_strategy
    )
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_agent_id_preserved_across_operation_types(
        self, setup_fs_backend, agent_id, operation
    ):
        """R7: Agent ID is recorded correctly regardless of operation type."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        path = f"content/01-Part/01-Chapter/r7-op-{unique_id}.md"
        book_id = "test-book"
        full_path = f"books/{book_id}/{path}"

        await log_operation(
            operation=operation,
            path=full_path,
            agent_id=agent_id,
            status=OperationStatus.SUCCESS,
            new_hash="a" * 64 if operation != OperationType.DELETE_CONTENT else None,
            book_id=book_id
        )

        async with get_session() as session:
            stmt = select(AuditLog).where(
                AuditLog.book_id == book_id,
                AuditLog.path == path
            ).order_by(AuditLog.id.desc()).limit(1)
            result = await session.execute(stmt)
            entry = result.scalar_one_or_none()

            assert entry is not None
            assert entry.agent_id == agent_id
            assert entry.operation == operation.value


class TestAgentIdQueryFiltering:
    """Test that queries can filter by agent_id."""

    @pytest.mark.asyncio
    @given(
        agent_a=valid_agent_id_strategy,
        agent_b=valid_agent_id_strategy
    )
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_query_filters_by_agent(self, setup_fs_backend, agent_a, agent_b):
        """Queries correctly filter by agent_id."""
        assume(agent_a != agent_b)

        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"test-book-{unique_id}"

        # Log operations from both agents
        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=f"books/{book_id}/content/{unique_id}-a.md",
            agent_id=agent_a,
            status=OperationStatus.SUCCESS,
            new_hash="a" * 64,
            book_id=book_id
        )

        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=f"books/{book_id}/content/{unique_id}-b.md",
            agent_id=agent_b,
            status=OperationStatus.SUCCESS,
            new_hash="b" * 64,
            book_id=book_id
        )

        # Query for agent_a only
        entries_a = await query_audit_log(agent_id=agent_a)

        # All returned entries should be from agent_a
        for entry in entries_a:
            if entry.book_id == book_id:  # Filter to our test entries
                assert entry.agent_id == agent_a

    @pytest.mark.asyncio
    @given(agent_id=valid_agent_id_strategy)
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_agent_id_in_all_queried_entries(self, setup_fs_backend, agent_id):
        """All queried entries have non-empty, non-system agent_id."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = f"test-book-{unique_id}"

        # Log multiple operations
        for i in range(3):
            await log_operation(
                operation=OperationType.WRITE_CONTENT,
                path=f"books/{book_id}/content/{unique_id}-{i}.md",
                agent_id=agent_id,
                status=OperationStatus.SUCCESS,
                new_hash=f"{chr(97 + i)}" * 64,
                book_id=book_id
            )

        # Query all
        entries = await query_audit_log(book_id=book_id)

        # R7 assertion: all entries have valid agent_id
        assert len(entries) >= 3
        for entry in entries:
            assert entry.agent_id is not None
            assert entry.agent_id != ""
            assert entry.agent_id != "system"


class TestAgentIdEdgeCases:
    """Test edge cases for agent ID handling."""

    @pytest.mark.asyncio
    async def test_numeric_agent_id(self, setup_fs_backend):
        """Agent IDs can be numeric strings."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"
        agent_id = "12345"

        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=f"books/{book_id}/content/{unique_id}.md",
            agent_id=agent_id,
            status=OperationStatus.SUCCESS,
            new_hash="a" * 64,
            book_id=book_id
        )

        entries = await query_audit_log(agent_id=agent_id, book_id=book_id)
        assert len(entries) >= 1
        assert entries[0].agent_id == "12345"

    @pytest.mark.asyncio
    async def test_hyphenated_agent_id(self, setup_fs_backend):
        """Agent IDs can contain hyphens (common format)."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"
        agent_id = "claude-lesson-writer-7"

        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=f"books/{book_id}/content/{unique_id}.md",
            agent_id=agent_id,
            status=OperationStatus.SUCCESS,
            new_hash="a" * 64,
            book_id=book_id
        )

        entries = await query_audit_log(agent_id=agent_id, book_id=book_id)
        assert len(entries) >= 1
        assert entries[0].agent_id == "claude-lesson-writer-7"

    @pytest.mark.asyncio
    async def test_uuid_agent_id(self, setup_fs_backend):
        """Agent IDs can be UUIDs."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "test-book"
        agent_id = str(uuid.uuid4())

        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path=f"books/{book_id}/content/{unique_id}.md",
            agent_id=agent_id,
            status=OperationStatus.SUCCESS,
            new_hash="a" * 64,
            book_id=book_id
        )

        entries = await query_audit_log(agent_id=agent_id, book_id=book_id)
        assert len(entries) >= 1
        assert entries[0].agent_id == agent_id
