"""Property-based tests for journal-storage consistency (T033).

Tests R2 invariant: Journal and storage always reflect the same hash for any file.
Uses hypothesis for property-based testing.
"""

import pytest
import json
from hypothesis import given, strategies as st, settings, assume, HealthCheck
from panaversity_fs.tools.content import read_content, write_content
from panaversity_fs.models import ReadContentInput, WriteContentInput
from panaversity_fs.database.connection import get_session
from panaversity_fs.database.models import FileJournal
from panaversity_fs.storage_utils import compute_sha256
from panaversity_fs.storage import get_operator
from sqlalchemy import select


# Each test generates unique UUIDs so fixture state doesn't matter
HYPOTHESIS_SETTINGS = {
    "max_examples": 10,
    "deadline": None,
    "suppress_health_check": [HealthCheck.function_scoped_fixture]
}


# Strategies for generating test data
content_strategy = st.text(
    alphabet=st.characters(whitelist_categories=('L', 'N', 'P', 'Zs')),
    min_size=10,
    max_size=1000
).map(lambda s: f"# Test Content\n\n{s}")

lesson_number = st.integers(min_value=1, max_value=99).map(lambda n: f"{n:02d}")


class TestJournalStorageConsistency:
    """Property tests for R2: Journal-storage hash consistency."""

    @pytest.mark.asyncio
    @given(content=content_strategy)
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_create_ensures_journal_storage_match(self, setup_fs_backend, content):
        """R2: After create, journal hash == storage hash."""
        # Create unique path for this test
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        path = f"content/01-Part/01-Chapter/{unique_id}.md"

        # Create file
        result = await write_content(WriteContentInput(
            book_id="test-book",
            path=path,
            content=content
        ))
        data = json.loads(result)

        # Get journal hash
        async with get_session() as session:
            stmt = select(FileJournal).where(
                FileJournal.book_id == "test-book",
                FileJournal.path == path,
                FileJournal.user_id == "__base__"
            )
            result = await session.execute(stmt)
            entry = result.scalar_one_or_none()

            assert entry is not None, "Journal entry should exist"
            journal_hash = entry.sha256

        # Get storage hash
        op = get_operator()
        storage_content = await op.read(f"books/test-book/{path}")
        storage_hash = compute_sha256(storage_content)

        # R2 invariant: hashes must match
        assert journal_hash == storage_hash, \
            f"Journal hash {journal_hash} != storage hash {storage_hash}"
        assert journal_hash == data["file_hash"], \
            "Response hash should match both journal and storage"

    @pytest.mark.asyncio
    @given(
        original_content=content_strategy,
        updated_content=content_strategy
    )
    @settings(max_examples=5, deadline=None, suppress_health_check=[HealthCheck.function_scoped_fixture])
    async def test_update_maintains_journal_storage_match(
        self, setup_fs_backend, original_content, updated_content
    ):
        """R2: After update, journal hash == storage hash."""
        assume(original_content != updated_content)

        import uuid
        unique_id = str(uuid.uuid4())[:8]
        path = f"content/01-Part/01-Chapter/update-{unique_id}.md"

        # Create
        r1 = await write_content(WriteContentInput(
            book_id="test-book",
            path=path,
            content=original_content
        ))
        d1 = json.loads(r1)

        # Update
        r2 = await write_content(WriteContentInput(
            book_id="test-book",
            path=path,
            content=updated_content,
            expected_hash=d1["file_hash"]
        ))
        d2 = json.loads(r2)

        # Verify R2 after update
        async with get_session() as session:
            stmt = select(FileJournal).where(
                FileJournal.book_id == "test-book",
                FileJournal.path == path,
                FileJournal.user_id == "__base__"
            )
            result = await session.execute(stmt)
            entry = result.scalar_one_or_none()
            journal_hash = entry.sha256

        op = get_operator()
        storage_content = await op.read(f"books/test-book/{path}")
        storage_hash = compute_sha256(storage_content)

        assert journal_hash == storage_hash
        assert journal_hash == d2["file_hash"]

    @pytest.mark.asyncio
    @given(content=content_strategy)
    @settings(max_examples=5, deadline=None, suppress_health_check=[HealthCheck.function_scoped_fixture])
    async def test_read_returns_journal_consistent_hash(self, setup_fs_backend, content):
        """Read returns hash consistent with journal."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        path = f"content/01-Part/01-Chapter/read-{unique_id}.md"

        # Create
        await write_content(WriteContentInput(
            book_id="test-book",
            path=path,
            content=content
        ))

        # Read
        read_result = await read_content(ReadContentInput(
            book_id="test-book",
            path=path
        ))
        read_data = json.loads(read_result)

        # Verify journal consistency
        async with get_session() as session:
            stmt = select(FileJournal).where(
                FileJournal.book_id == "test-book",
                FileJournal.path == path,
                FileJournal.user_id == "__base__"
            )
            result = await session.execute(stmt)
            entry = result.scalar_one_or_none()

            assert entry.sha256 == read_data["file_hash_sha256"]


class TestHashDeterminism:
    """Property tests for hash computation determinism."""

    @pytest.mark.asyncio
    @given(content=content_strategy)
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_same_content_same_hash(self, setup_fs_backend, content):
        """Same content always produces same hash."""
        import uuid
        unique1 = str(uuid.uuid4())[:8]
        unique2 = str(uuid.uuid4())[:8]
        path1 = f"content/01-Part/01-Chapter/same1-{unique1}.md"
        path2 = f"content/01-Part/01-Chapter/same2-{unique2}.md"

        # Create two files with same content
        r1 = await write_content(WriteContentInput(
            book_id="test-book",
            path=path1,
            content=content
        ))
        r2 = await write_content(WriteContentInput(
            book_id="test-book",
            path=path2,
            content=content
        ))

        d1 = json.loads(r1)
        d2 = json.loads(r2)

        # Same content = same hash
        assert d1["file_hash"] == d2["file_hash"]

    @pytest.mark.asyncio
    @given(
        content1=content_strategy,
        content2=content_strategy
    )
    @settings(**HYPOTHESIS_SETTINGS)
    async def test_different_content_different_hash(self, setup_fs_backend, content1, content2):
        """Different content produces different hash (with high probability)."""
        assume(content1 != content2)

        import uuid
        unique1 = str(uuid.uuid4())[:8]
        unique2 = str(uuid.uuid4())[:8]
        path1 = f"content/01-Part/01-Chapter/diff1-{unique1}.md"
        path2 = f"content/01-Part/01-Chapter/diff2-{unique2}.md"

        r1 = await write_content(WriteContentInput(
            book_id="test-book",
            path=path1,
            content=content1
        ))
        r2 = await write_content(WriteContentInput(
            book_id="test-book",
            path=path2,
            content=content2
        ))

        d1 = json.loads(r1)
        d2 = json.loads(r2)

        # Different content = different hash (SHA256 collision extremely unlikely)
        assert d1["file_hash"] != d2["file_hash"]
